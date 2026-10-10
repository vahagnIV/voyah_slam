#include <support/synthetic_sequence.h>
#include <support/test_measurements.h>
#include <support/trajectory_error.h>
#include <voyahslam/bow/vocabulary.h>
#include <voyahslam/events/async_observer.h>
#include <voyahslam/events/events.h>
#include <voyahslam/features/orb_feature_extractor.h>
#include <voyahslam/features/sift_feature_extractor.h>
#include <voyahslam/slam/slam.h>
#include <voyahslam/tracking/atlas_replica.h>
#include <voyahslam/tracking/map_replica.h>

#include <gtest/gtest.h>

#include <Eigen/Geometry>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <iterator>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <thread>
#include <utility>
#include <variant>
#include <vector>

namespace voyahslam::slam {
namespace {

using OrbSlam = Slam<features::OrbFeatureExtractor>;

constexpr double kFrameInterval = 0.05;  // SyntheticSequenceOptions::frame_interval

size_t FrameIndexOf(double timestamp) {
    return static_cast<size_t>(std::lround(timestamp / kFrameInterval));
}

// A pose published for a map: an added Frame (Mapper) or FrameTracked (Tracker).
struct StampedPose {
    uint32_t map;
    geometry::RigidTransformation pose;
    double timestamp;
    events::TrackingStage stage = events::TrackingStage::MotionModel;  // FrameTracked
    map::FrameId frame{};  // an added Frame's id
};

TEST(SlamTest, WiresComponentsAsynchronously) {
    OrbSlam slam;
    EXPECT_EQ(slam.tracker().async_observer_count(), 1u);      // → Mapper
    EXPECT_EQ(slam.mapper().async_observer_count(), 2u);       // → Tracker, LoopCloser
    EXPECT_EQ(slam.loop_closer().async_observer_count(), 1u);  // → Mapper
}

TEST(SlamTest, ExternalObserversCanBeAddedToEveryComponent) {
    OrbSlam slam;
    events::AsyncObserver visualizer;
    slam.tracker().AddObserver(visualizer);
    slam.mapper().AddObserver(visualizer);
    slam.loop_closer().AddObserver(visualizer);

    EXPECT_EQ(slam.tracker().async_observer_count(), 2u);
    EXPECT_EQ(slam.mapper().async_observer_count(), 3u);
    EXPECT_EQ(slam.loop_closer().async_observer_count(), 2u);
}

TEST(SlamTest, StartStopAndDestroy) {
    OrbSlam slam;
    slam.Start();
    slam.Stop();
}

TEST(SlamTest, TrackEnqueuesMonocularAndBinocularMeasurements) {
    using TTracker = Slam<features::SIFTFeatureExtractor>::TTracker;
    Slam<features::SIFTFeatureExtractor> slam;
    EXPECT_EQ(slam.tracker().pending_measurements(), 0u);

    slam.tracker().Track(TTracker::TMonocularMeasurement{});
    TTracker::TBinocularMeasurement binocular;
    slam.tracker().Track(binocular);             // copy
    slam.tracker().Track(std::move(binocular));  // move

    EXPECT_EQ(slam.tracker().pending_measurements(), 3u);
}

// A LoopDetected's content.
struct LoopRecord {
    map::FrameId query_frame;
    map::FrameId match_frame;
    geometry::Sim3 S_query_match;

    explicit LoopRecord(const events::LoopDetected& loop)
        : query_frame(loop.query_frame), match_frame(loop.match_frame), S_query_match(loop.S_query_match) {}
};

// Records the map ids of the protocol events published by the Mapper and the
// Tracker, and the LoopCloser's loops, read through external observers, and
// keeps a replica of the Mapper's Atlas.
class ProtocolRecorder {
public:
    // Before slam.Start().
    void Attach(OrbSlam& slam) {
        slam.mapper().AddObserver(mapper_events_);
        slam.tracker().AddObserver(tracker_events_);
        slam.loop_closer().AddObserver(loop_closer_events_);
    }

    // Reads every pending event; call only from the test thread.
    void Poll() {
        while (events::Event* event = mapper_listener_.GetNextEvent()) {
            if (event->type != events::EventType::MapTransaction) {
                continue;
            }
            const auto& transaction =
                *static_cast<events::MapTransaction<features::OrbFeatureExtractor>*>(event);
            atlas.Apply(transaction);
            const uint32_t map = event->map_id.value;
            const bool announced = !map_created.empty() && map_created.back() == map;
            if (transaction.new_map) {
                map_created.push_back(map);
            }
            for (const auto& frame : transaction.added_frames) {
                keyframes.push_back({map, frame.T_cw,
                                     std::visit([](const auto& m) { return m.timestamp(); },
                                                *frame.measurement),
                                     events::TrackingStage::MotionModel, frame.id});
            }
            transaction_new_points.push_back(transaction.num_new_points());
            if (transaction.map_epoch > map_epoch) {
                map_epoch = transaction.map_epoch;
                corrections.push_back(keyframes.size());
                epoch_from_loop = !transaction.added_loop_edges.empty();
            } else if (!transaction.new_map && transaction.active_map) {
                // A Frame's creation, or its refinement (local BA, culling);
                // not the Mapper discarding or deactivating a lost map.
                ++(transaction.added_frames.empty() ? num_refinements : num_creations);
            }
            if (transaction.num_new_points() > 0) {
                new_points.push_back(map);
                // Map points not after the transaction that announces their map.
                new_points_not_after_new_map.push_back(!announced);
            }
        }
        while (events::Event* event = tracker_listener_.GetNextEvent()) {
            if (event->type == events::EventType::FrameTracked) {
                const auto& tracked =
                    *static_cast<events::FrameTracked<features::OrbFeatureExtractor>*>(event);
                frame_tracked.push_back(event->map_id.value);
                positions.push_back(
                    {event->map_id.value, tracked.pose, tracked.timestamp, tracked.stage});
            } else if (event->type == events::EventType::TrackingLost) {
                tracking_lost.push_back(event->map_id.value);
                tracking_lost_final.push_back(static_cast<const events::TrackingLost*>(event)->final);
            }
        }
        while (events::Event* event = loop_closer_listener_.GetNextEvent()) {
            if (event->type == events::EventType::LoopDetected) {
                loops.emplace_back(*static_cast<const events::LoopDetected*>(event));
            }
        }
    }

    // Polls until `condition` holds or 30 s pass; returns the final result.
    // Generous: a two-view reconstruction takes ~0.5 s in a Debug build.
    template <typename TCondition>
    bool WaitFor(TCondition condition,
                 std::chrono::milliseconds timeout = std::chrono::seconds(30)) {
        const auto deadline = std::chrono::steady_clock::now() + timeout;
        while (true) {
            Poll();
            if (condition()) {
                return true;
            }
            if (std::chrono::steady_clock::now() >= deadline) {
                return false;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    }

    std::vector<uint32_t> map_created;  // map ids of the new-map transactions
    // Map ids of the transactions that carry new points.
    std::vector<uint32_t> new_points;
    std::vector<bool> new_points_not_after_new_map;
    std::vector<uint32_t> frame_tracked;
    std::vector<uint32_t> tracking_lost;
    std::vector<bool> tracking_lost_final;  // per TrackingLost
    std::vector<StampedPose> keyframes;  // added Frames, in publication order
    std::vector<size_t> transaction_new_points;  // per transaction, in publication order
    std::vector<StampedPose> positions;
    // The Mapper's Atlas, rebuilt from its transactions: every live map, the
    // inactive ones included.
    tracking::AtlasReplica<features::OrbFeatureExtractor> atlas;
    // Map `id` of the Atlas; empty if it is not live.
    [[nodiscard]] const tracking::MapReplica<features::OrbFeatureExtractor>& live_map(uint32_t id) const {
        static const tracking::MapReplica<features::OrbFeatureExtractor> kEmpty;
        const auto* found = atlas.map(map::MapId{id});
        return found != nullptr ? *found : kEmpty;
    }
    // The LoopCloser's LoopDetected, copied (not pooled).
    std::vector<LoopRecord> loops;
    // The latest transaction's map epoch; per loop correction or global BA
    // merge (a transaction with a new epoch), the number of Frames added
    // before it.
    uint32_t map_epoch = 0;
    std::vector<size_t> corrections;
    // Whether the latest epoch is a loop correction's (it adds a loop edge),
    // not a global BA merge's.
    bool epoch_from_loop = false;
    // Transactions creating a Frame, and refining around one.
    size_t num_creations = 0;
    size_t num_refinements = 0;

private:
    // Listeners after their observers: destroyed first, releasing the
    // event they hold.
    events::AsyncObserver mapper_events_;
    events::AsyncObserver tracker_events_;
    events::AsyncObserver loop_closer_events_;
    events::Listener mapper_listener_ = mapper_events_.GetListener();
    events::Listener tracker_listener_ = tracker_events_.GetListener();
    events::Listener loop_closer_listener_ = loop_closer_events_.GetListener();
};

TEST(SlamTest, ForwardsTrackerOptions) {
    // nn_ratio 0 fails every ratio test: no depths, so no stereo map. The same
    // frame under default options does create a map.
    const auto map_created_for = [](const tracking::TrackerOptions& tracker_options,
                                    std::chrono::milliseconds wait) {
        ProtocolRecorder recorder;
        OrbSlam slam({}, tracker_options);
        recorder.Attach(slam);
        slam.Start();
        slam.tracker().Track(test::MakeSyntheticStereoFrame().measurement);
        const bool created = recorder.WaitFor([&] { return !recorder.map_created.empty(); }, wait);
        slam.Stop();
        return created;
    };
    tracking::TrackerOptions no_matches;
    no_matches.stereo_matcher.nn_ratio = 0.0;
    EXPECT_FALSE(map_created_for(no_matches, std::chrono::milliseconds(300)));
    EXPECT_TRUE(map_created_for({}, std::chrono::seconds(30)));
}

// Expects every FrameTracked of `map` to be near ground truth (`truth`,
// indexed by frame), aligned through the map's keyframes. Monocular: scale
// from the second keyframe's baseline, rotation < 0.5 deg and centre error
// < 2 % of the distance travelled since the map's first keyframe. Stereo:
// metric, rotation < 0.5 deg and centre error < 2 cm. Returns the number of
// poses checked.
size_t ExpectPositionsNearTruth(const ProtocolRecorder& recorder, uint32_t map,
                                const std::vector<geometry::RigidTransformation>& truth,
                                bool monocular) {
    std::vector<StampedPose> keyframes;
    for (const StampedPose& keyframe : recorder.keyframes) {
        if (keyframe.map == map) keyframes.push_back(keyframe);
    }
    if (keyframes.size() != (monocular ? 2u : 1u)) {
        ADD_FAILURE() << "map " << map << ": " << keyframes.size() << " keyframes";
        return 0;
    }
    const size_t first = FrameIndexOf(keyframes[0].timestamp);
    const test::MapAlignment alignment =
        monocular ? test::MapAlignment(truth[first], truth[FrameIndexOf(keyframes[1].timestamp)],
                                       keyframes[1].pose)
                  : test::MapAlignment(truth[first]);
    size_t checked = 0;
    for (const StampedPose& position : recorder.positions) {
        if (position.map != map) continue;
        const size_t k = FrameIndexOf(position.timestamp);
        EXPECT_LT(alignment.RotationErrorDeg(position.pose, truth[k]), 0.5) << "frame " << k;
        const double error = alignment.CentreError(position.pose, truth[k]);
        if (monocular) {
            const double travelled = (test::MapAlignment::Centre(truth[k]) -
                                      test::MapAlignment::Centre(truth[first]))
                                         .norm();
            EXPECT_LT(error, 0.02 * travelled) << "frame " << k;
        } else {
            EXPECT_LT(error, 0.02) << "frame " << k;
        }
        ++checked;
    }
    return checked;
}

TEST(SlamTest, EndToEndInitializeTrackLoseReinitialize) {
    // Map 1 from frames 0-14, tracking on frames 15-16, a featureless frame
    // loses tracking, map 2 from frames 17-29, tracking on frames 30-39.
    // Timestamps increase throughout.
    constexpr size_t kMap1End = 14;
    constexpr size_t kMap2Begin = kMap1End + 3;
    constexpr size_t kMap2End = 29;
    test::SyntheticSequenceOptions options;
    options.num_frames = 40;
    const test::SyntheticSequence sequence = test::MakeSyntheticSequence(options);
    std::vector<geometry::RigidTransformation> truth;
    for (const auto& frame : sequence.frames) truth.push_back(frame.T_cw);
    const auto frame = [&](size_t i) { return sequence.frames[i].measurement; };

    // Mono accuracy is measured on a well-conditioned initial map (Ruling T):
    // the default 1 deg minimum parallax initializes at ~1.4 deg, whose map has
    // ~12 % depth error that the tracked translation inherits.
    mapping::MapperOptions mapper_options;
    mapper_options.initializer.reconstruction.min_parallax_deg = 4.0;

    // Lifetimes: the recorder outlives the Slam (nothing is published into a
    // destroyed queue), and `stop_and_drain` stops the Slam and releases every
    // queued event before the Slam's pools are destroyed, also when an
    // assertion returns early.
    ProtocolRecorder recorder;
    OrbSlam slam(mapper_options);
    recorder.Attach(slam);
    struct StopAndDrain {
        OrbSlam& slam;
        ProtocolRecorder& recorder;
        ~StopAndDrain() {
            slam.Stop();
            recorder.Poll();
        }
    } stop_and_drain{slam, recorder};
    auto& tracker = slam.tracker();
    auto& mapper = slam.mapper();
    slam.Start();

    // Monocular initialization: frame 0 is the reference, the map is built
    // once a later frame has enough parallax.
    for (size_t i = 0; i <= kMap1End; ++i) {
        tracker.Track(frame(i));
    }
    ASSERT_TRUE(recorder.WaitFor([&] { return !recorder.map_created.empty(); }));
    EXPECT_EQ(recorder.map_created, std::vector<uint32_t>{1});
    EXPECT_EQ(recorder.new_points, std::vector<uint32_t>{1});
    EXPECT_EQ(recorder.new_points_not_after_new_map, std::vector<bool>{true});
    ASSERT_TRUE(recorder.WaitFor([&] {
        return tracker.state() == tracking::TrackerState::Tracking &&
               tracker.current_map_id() == map::MapId{1};
    }));
    EXPECT_EQ(mapper.state(), mapping::MapperState::Mapping);
    EXPECT_EQ(mapper.current_map_id(), map::MapId{1});

    // Tracking on map 1.
    tracker.Track(frame(kMap1End + 1));
    tracker.Track(frame(kMap1End + 2));
    ASSERT_TRUE(recorder.WaitFor([&] { return recorder.frame_tracked.size() >= 2; }));
    for (uint32_t map : recorder.frame_tracked) {
        EXPECT_EQ(map, 1u);
    }
    EXPECT_TRUE(recorder.tracking_lost.empty());

    // Too few features: tracking is lost, the Mapper drops map 1.
    const double lost_timestamp =
        0.5 * (frame(kMap1End + 2).timestamp() + frame(kMap2Begin).timestamp());
    tracker.Track(test::MakeMonocular(0, lost_timestamp));
    ASSERT_TRUE(recorder.WaitFor([&] { return !recorder.tracking_lost.empty(); }));
    EXPECT_EQ(recorder.tracking_lost, std::vector<uint32_t>{1});
    EXPECT_EQ(tracker.state(), tracking::TrackerState::Lost);

    // Re-initialization from the later frames builds map 2.
    for (size_t i = kMap2Begin; i <= kMap2End; ++i) {
        tracker.Track(frame(i));
    }
    ASSERT_TRUE(recorder.WaitFor([&] { return recorder.map_created.size() >= 2; }));
    EXPECT_EQ(recorder.map_created, (std::vector<uint32_t>{1, 2}));
    EXPECT_EQ(recorder.new_points, (std::vector<uint32_t>{1, 2}));
    EXPECT_EQ(recorder.new_points_not_after_new_map, (std::vector<bool>{true, true}));
    ASSERT_TRUE(recorder.WaitFor([&] {
        return tracker.state() == tracking::TrackerState::Tracking &&
               tracker.current_map_id() == map::MapId{2};
    }));
    EXPECT_EQ(mapper.state(), mapping::MapperState::Mapping);
    EXPECT_EQ(mapper.current_map_id(), map::MapId{2});

    // Tracking on map 2: every remaining frame.
    const size_t before = recorder.frame_tracked.size();
    for (size_t i = kMap2End + 1; i < sequence.frames.size(); ++i) {
        tracker.Track(frame(i));
    }
    const size_t expected = before + sequence.frames.size() - kMap2End - 1;
    ASSERT_TRUE(recorder.WaitFor([&] { return recorder.frame_tracked.size() >= expected; }));
    EXPECT_EQ(recorder.frame_tracked.back(), 2u);
    EXPECT_EQ(recorder.tracking_lost, std::vector<uint32_t>{1});

    EXPECT_GE(ExpectPositionsNearTruth(recorder, 1, truth, /*monocular=*/true), 2u);
    EXPECT_GE(ExpectPositionsNearTruth(recorder, 2, truth, /*monocular=*/true),
              sequence.frames.size() - kMap2End - 1);
}

TEST(SlamTest, EndToEndStereoInitializeTrackLoseReinitialize) {
    // Map 1 from frame 0 of a synthetic stereo sequence (the Tracker
    // stereo-matches it), tracking on frame 1, a featureless frame loses
    // tracking, map 2 from frame 3, tracking on frame 4. Timestamps increase
    // throughout.
    test::SyntheticStereoSequenceOptions options;
    options.trajectory.num_points = 600;  // a stereo map needs > 500 left features
    const test::SyntheticStereoSequence sequence = test::MakeSyntheticStereoSequence(options);
    std::vector<geometry::RigidTransformation> truth;
    for (const auto& frame : sequence.frames) truth.push_back(frame.T_cw);
    const auto frame = [&](size_t i) { return sequence.frames[i].measurement; };

    // Lifetimes as in EndToEndInitializeTrackLoseReinitialize.
    ProtocolRecorder recorder;
    OrbSlam slam;
    recorder.Attach(slam);
    struct StopAndDrain {
        OrbSlam& slam;
        ProtocolRecorder& recorder;
        ~StopAndDrain() {
            slam.Stop();
            recorder.Poll();
        }
    } stop_and_drain{slam, recorder};
    auto& tracker = slam.tracker();
    auto& mapper = slam.mapper();
    slam.Start();

    // Stereo initialization from a single frame.
    tracker.Track(frame(0));
    ASSERT_TRUE(recorder.WaitFor([&] { return !recorder.map_created.empty(); }));
    EXPECT_EQ(recorder.map_created, std::vector<uint32_t>{1});
    EXPECT_EQ(recorder.new_points, std::vector<uint32_t>{1});
    EXPECT_EQ(recorder.new_points_not_after_new_map, std::vector<bool>{true});
    EXPECT_GE(mapper.num_map_points(), 100u);
    ASSERT_TRUE(recorder.WaitFor([&] {
        return tracker.state() == tracking::TrackerState::Tracking &&
               tracker.current_map_id() == map::MapId{1};
    }));
    EXPECT_EQ(mapper.state(), mapping::MapperState::Mapping);
    EXPECT_EQ(mapper.current_map_id(), map::MapId{1});

    // Tracking on map 1.
    tracker.Track(frame(1));
    ASSERT_TRUE(recorder.WaitFor([&] { return !recorder.frame_tracked.empty(); }));
    for (uint32_t map : recorder.frame_tracked) {
        EXPECT_EQ(map, 1u);
    }

    // Too few features: tracking is lost, the Mapper drops map 1.
    tracker.Track(test::MakeBinocular(0, 0.5 * (frame(1).timestamp() + frame(3).timestamp())));
    ASSERT_TRUE(recorder.WaitFor([&] { return !recorder.tracking_lost.empty(); }));
    EXPECT_EQ(recorder.tracking_lost, std::vector<uint32_t>{1});
    EXPECT_EQ(tracker.state(), tracking::TrackerState::Lost);

    // Re-initialization from a later stereo frame builds map 2.
    tracker.Track(frame(3));
    ASSERT_TRUE(recorder.WaitFor([&] { return recorder.map_created.size() >= 2; }));
    EXPECT_EQ(recorder.map_created, (std::vector<uint32_t>{1, 2}));
    EXPECT_EQ(recorder.new_points, (std::vector<uint32_t>{1, 2}));
    EXPECT_EQ(recorder.new_points_not_after_new_map, (std::vector<bool>{true, true}));
    EXPECT_GE(mapper.num_map_points(), 100u);
    ASSERT_TRUE(recorder.WaitFor([&] {
        return tracker.state() == tracking::TrackerState::Tracking &&
               tracker.current_map_id() == map::MapId{2};
    }));
    EXPECT_EQ(mapper.state(), mapping::MapperState::Mapping);
    EXPECT_EQ(mapper.current_map_id(), map::MapId{2});

    tracker.Track(frame(4));
    ASSERT_TRUE(recorder.WaitFor([&] { return recorder.frame_tracked.back() == 2u; }));
    EXPECT_EQ(recorder.tracking_lost, std::vector<uint32_t>{1});

    EXPECT_EQ(ExpectPositionsNearTruth(recorder, 1, truth, /*monocular=*/false), 1u);
    EXPECT_EQ(ExpectPositionsNearTruth(recorder, 2, truth, /*monocular=*/false), 1u);
}

// Synthetic sequence for the map-growth tests: the camera moves 6 m along +x
// through a 10 m room and turns 90 deg towards +x, so the last frames see
// only a wall the first frames never saw.
//
// Error bounds: 1.5x the largest of three measured runs with local BA
// (2026-10-07): mono 0.159 / 0.129 / 0.166 m (2.8 % of the 5.95 m path at
// worst; 1.21 m without local BA), stereo 0.0247 / 0.0248 / 0.0248 m
// (0.15 m without).
test::SyntheticSequenceOptions LeavingInitialViewOptions() {
    test::SyntheticSequenceOptions options;
    options.num_frames = 120;
    options.turn_deg = 90.0;
    options.surround_scene = true;
    options.num_points = 1000;
    return options;
}

// Largest camera-centre error, in true units, of `map`'s FrameTracked poses
// from frame `first_frame` on, aligned through the map's first keyframe
// (stereo) or first two keyframes (mono: scale from the second keyframe's
// baseline).
double MaxCentreError(const ProtocolRecorder& recorder, uint32_t map,
                      const std::vector<geometry::RigidTransformation>& truth, bool monocular,
                      size_t first_frame = 0) {
    std::vector<StampedPose> keyframes;
    for (const StampedPose& keyframe : recorder.keyframes) {
        if (keyframe.map == map) keyframes.push_back(keyframe);
    }
    const size_t first = FrameIndexOf(keyframes.at(0).timestamp);
    const test::MapAlignment alignment =
        monocular ? test::MapAlignment(truth[first],
                                       truth[FrameIndexOf(keyframes.at(1).timestamp)],
                                       keyframes[1].pose)
                  : test::MapAlignment(truth[first]);
    double max_error = 0.0;
    for (const StampedPose& position : recorder.positions) {
        if (position.map != map) continue;
        const size_t k = FrameIndexOf(position.timestamp);
        if (k < first_frame) continue;
        max_error = std::max(max_error, alignment.CentreError(position.pose, truth[k]));
    }
    return max_error;
}

// What a run left behind, read after the Slam stopped.
struct RunCounts {
    size_t tracked_before = 0;  // FrameTracked before the frames to track were fed
    size_t num_keyframes = 0;   // the Mapper's
    size_t num_map_points = 0;  // the Mapper's
    bool has_global_ba = false;  // Slam::global_ba() is not null
    // The GlobalBundleAdjuster's, if any.
    size_t global_ba_runs_started = 0;
    size_t global_ba_runs_interrupted = 0;
    size_t global_ba_results_dropped = 0;
    // The Mapper's GlobalBAFinished, merged and rejected.
    size_t global_ba_merges = 0;
    size_t global_ba_rejected = 0;
};

// Frames [begin, end] of a run arrive as measurements without features (a
// camera blackout), at their own timestamps.
struct Blackout {
    size_t begin = 0;
    size_t end = 0;
};

// A measurement without features at `timestamp`, of `sequence`'s kind.
features::TMonocularMeasurement<features::OrbFeatureExtractor> EmptyMeasurement(
    const test::SyntheticSequence& /*sequence*/, double timestamp) {
    return test::MakeMonocular(0, timestamp);
}
features::TBinocularMeasurement<features::OrbFeatureExtractor> EmptyMeasurement(
    const test::SyntheticStereoSequence& /*sequence*/, double timestamp) {
    return test::MakeBinocular(0, timestamp);
}

// How RunSequence sets up and feeds a run, beyond the Mapper's options.
struct RunSetup {
    tracking::TrackerOptions tracker_options;
    std::shared_ptr<const OrbSlam::TMapper::TVocabulary> vocabulary;
    std::optional<Blackout> blackout;
    loop_closing::LoopCloserOptions loop_closer_options;
    global_ba::GlobalBaOptions global_ba_options;
    // Each frame to track is fed once the previous one is tracked, the Mapper
    // has handled every FrameTracked so far and has refined around every Frame
    // it created (a Mapper that keeps up, as at camera rate). The Tracker and
    // the Mapper then no longer depend on thread timing; the LoopCloser is not
    // gated, so when its LoopDetected reaches the Mapper still does (in
    // practice runs agree to four digits). With global BA, a loop correction's
    // run is also awaited (merged) before the next frame, as a GlobalBA that
    // keeps up. Otherwise all at once, as fast as the Tracker goes.
    bool lock_step = false;
};

// Feeds a sequence to a new Slam: the first `init_frames` frames build the
// map, then every later frame is fed once it tracks (the frames of
// `setup.blackout`, if any, without features). Returns when the last frame is
// tracked or tracking is lost for good (TrackingLost with `final`), with the
// Slam stopped and `out_recorder` drained.
template <typename TSequence>
void RunSequence(const TSequence& sequence, size_t init_frames,
                 const mapping::MapperOptions& mapper_options, ProtocolRecorder& out_recorder,
                 RunCounts& out_counts, const RunSetup& setup = {}) {
    // Lifetimes as in EndToEndInitializeTrackLoseReinitialize.
    OrbSlam slam(mapper_options, setup.tracker_options, setup.vocabulary, setup.loop_closer_options,
                 setup.global_ba_options);
    out_recorder.Attach(slam);
    struct StopAndDrain {
        OrbSlam& slam;
        ProtocolRecorder& recorder;
        ~StopAndDrain() {
            slam.Stop();
            recorder.Poll();
        }
    } stop_and_drain{slam, out_recorder};
    slam.Start();

    // The Tracker forwards frames to the Mapper until the map arrives, so the
    // frames to track are fed only once it tracks.
    for (size_t i = 0; i < init_frames; ++i) {
        slam.tracker().Track(sequence.frames[i].measurement);
    }
    ASSERT_TRUE(out_recorder.WaitFor(
        [&] { return slam.tracker().state() == tracking::TrackerState::Tracking; },
        std::chrono::seconds(120)));
    out_counts.tracked_before = out_recorder.positions.size();
    const auto& mapper = slam.mapper();
    const auto mapper_handled = [&] {
        return mapper.num_frame_evaluations() + mapper.backlog_skipped() + mapper.stale_epoch_skipped();
    };
    // With global BA: until the latest loop correction's run is merged (a
    // merge raises the epoch again), at most 60 s. A rejected result (the
    // Mapper's count grows) or a timeout (e.g. a dropped result) ends the
    // wait for that correction for good: it fails the test with the counts
    // after the Slam stopped, rather than waiting 60 s per later frame.
    std::optional<size_t> given_up_correction;  // its index in out_recorder.corrections
    std::string given_up_reason;
    const auto await_global_ba = [&] {
        if (slam.global_ba() == nullptr || !out_recorder.epoch_from_loop) return;
        const size_t correction = out_recorder.corrections.size() - 1;
        if (given_up_correction == correction) return;
        const size_t rejected_before = slam.mapper().num_global_ba_rejected();
        const bool ended = out_recorder.WaitFor(
            [&] {
                return !out_recorder.epoch_from_loop || slam.mapper().num_global_ba_rejected() > rejected_before;
            },
            std::chrono::seconds(60));
        // Still this correction's epoch: not merged.
        if (out_recorder.epoch_from_loop && out_recorder.corrections.size() - 1 == correction) {
            given_up_correction = correction;
            given_up_reason = ended ? "a result was rejected" : "no merge within 60 s";
        }
    };
    for (size_t i = init_frames; i < sequence.frames.size(); ++i) {
        const auto& blackout = setup.blackout;
        if (blackout && i >= blackout->begin && i <= blackout->end) {
            slam.tracker().Track(EmptyMeasurement(sequence, sequence.frames[i].measurement.timestamp()));
        } else {
            slam.tracker().Track(sequence.frames[i].measurement);
        }
        if (setup.lock_step) {
            // A frame that fails to track publishes nothing: then go on after 2 s.
            static_cast<void>(out_recorder.WaitFor(
                [&] {
                    return !out_recorder.positions.empty() &&
                           FrameIndexOf(out_recorder.positions.back().timestamp) == i;
                },
                std::chrono::seconds(2)));
            ASSERT_TRUE(out_recorder.WaitFor([&] {
                return mapper_handled() >= out_recorder.positions.size() &&
                       out_recorder.num_refinements >= out_recorder.num_creations;
            }));
            await_global_ba();
        }
    }
    const size_t last = sequence.frames.size() - 1;
    ASSERT_TRUE(out_recorder.WaitFor(
        [&] {
            const auto& finals = out_recorder.tracking_lost_final;
            return std::find(finals.begin(), finals.end(), true) != finals.end() ||
                   (!out_recorder.positions.empty() &&
                    FrameIndexOf(out_recorder.positions.back().timestamp) == last);
        },
        std::chrono::seconds(120)));
    // A correction after the last frame's gate.
    await_global_ba();
    slam.Stop();
    out_recorder.Poll();
    out_counts.num_keyframes = slam.mapper().num_keyframes();
    out_counts.num_map_points = slam.mapper().num_map_points();
    out_counts.global_ba_merges = slam.mapper().num_global_ba_merges();
    out_counts.global_ba_rejected = slam.mapper().num_global_ba_rejected();
    if (const auto* global_ba = slam.global_ba()) {
        out_counts.has_global_ba = true;
        out_counts.global_ba_runs_started = global_ba->num_runs_started();
        out_counts.global_ba_runs_interrupted = global_ba->num_runs_interrupted();
        out_counts.global_ba_results_dropped = global_ba->num_results_dropped();
    }
    if (given_up_correction) {
        ADD_FAILURE() << "Global BA of loop correction " << *given_up_correction + 1 << " not merged: "
                      << given_up_reason << " (runs started " << out_counts.global_ba_runs_started
                      << ", interrupted " << out_counts.global_ba_runs_interrupted << ", results dropped "
                      << out_counts.global_ba_results_dropped << "; merged " << out_counts.global_ba_merges
                      << ", rejected " << out_counts.global_ba_rejected << ")";
    }
}

// Runs RunSequence and expects map 1 only, no TrackingLost, more than the
// initial Frames and points, and a centre error within `max_error`.
template <typename TSequence>
void ExpectMapGrowsWhileLeavingInitialView(const TSequence& sequence, size_t init_frames,
                                           bool monocular,
                                           const mapping::MapperOptions& mapper_options,
                                           double max_error) {
    std::vector<geometry::RigidTransformation> truth;
    for (const auto& frame : sequence.frames) truth.push_back(frame.T_cw);

    ProtocolRecorder recorder;
    RunCounts counts;
    RunSequence(sequence, init_frames, mapper_options, recorder, counts);
    if (::testing::Test::HasFatalFailure()) return;

    EXPECT_TRUE(recorder.tracking_lost.empty());
    EXPECT_EQ(recorder.positions.size() - counts.tracked_before,
              sequence.frames.size() - init_frames);
    EXPECT_EQ(recorder.map_created, std::vector<uint32_t>{1});
    ASSERT_GE(recorder.keyframes.size(), monocular ? 2u : 1u);
    // The new-map transaction carries every initial point.
    const size_t initial_points = recorder.transaction_new_points.at(0);
    EXPECT_GT(counts.num_keyframes, 2u);
    EXPECT_GT(counts.num_map_points, initial_points);
    EXPECT_LE(MaxCentreError(recorder, 1, truth, monocular), max_error);
}

TEST(SlamTest, MonoMapGrowsWhileLeavingInitialView) {
    const test::SyntheticSequenceOptions options = LeavingInitialViewOptions();
    const test::SyntheticSequence sequence = test::MakeSyntheticSequence(options);
    // As in EndToEndInitializeTrackLoseReinitialize: a well-conditioned initial map.
    mapping::MapperOptions mapper_options;
    mapper_options.initializer.reconstruction.min_parallax_deg = 4.0;
    // The map is initialized within the first 30 frames. Error bound: see
    // LeavingInitialViewOptions.
    ExpectMapGrowsWhileLeavingInitialView(sequence, /*init_frames=*/30, /*monocular=*/true,
                                          mapper_options, 1.5 * 0.166);
}

TEST(SlamTest, StereoMapGrowsWhileLeavingInitialView) {
    test::SyntheticStereoSequenceOptions options;
    options.trajectory = LeavingInitialViewOptions();
    // Fewer points than mono: Debug stereo tracking costs about 0.5 s per
    // frame at 1000.
    options.trajectory.num_points = 700;
    const test::SyntheticStereoSequence sequence = test::MakeSyntheticStereoSequence(options);
    mapping::MapperOptions mapper_options;
    // The room's first view has about 200 features (a real stereo map needs
    // 500).
    mapper_options.initializer.stereo_min_features = 150;
    // Error bound: see LeavingInitialViewOptions.
    ExpectMapGrowsWhileLeavingInitialView(sequence, /*init_frames=*/1, /*monocular=*/false,
                                          mapper_options, 1.5 * 0.0248);
}

// k = 2, L = 1: two words of weight 1 under the root (as the Tracker tests'
// MakeRootVocabulary). The Tracker looks features up 4 levels up, so every
// FeatureVector files all features under the root and SearchByBoW compares
// every pair (the ratio test decides); the Frame database returns every Frame.
std::shared_ptr<const OrbSlam::TMapper::TVocabulary> RootVocabulary() {
    using TVocabulary = OrbSlam::TMapper::TVocabulary;
    using TDescriptors = features::OrbFeatureExtractor::TDescriptors;
    TDescriptors descriptors =
        TDescriptors::Zero(features::OrbFeatureExtractor::DescriptorSize, 3);
    descriptors(0, 2) = 0xFF;
    return std::make_shared<const TVocabulary>(
        TVocabulary::FromTree(2, 1, {0, 0, 0}, std::move(descriptors), {0.0, 1.0, 1.0}));
}

// A Frame's measurement is one immutable object (spec section 1): the
// Mapper's Frame, the Tracker's replica and the LoopCloser's replica hold the
// same one, not copies. The first 40 frames of the stereo leave-the-view run,
// each fed once the previous one is tracked; then every Frame the replicas
// hold is compared.
TEST(SlamTest, MeasurementsAreShared) {
    test::SyntheticStereoSequenceOptions options;
    options.trajectory = LeavingInitialViewOptions();
    options.trajectory.num_points = 700;  // as StereoMapGrowsWhileLeavingInitialView
    const test::SyntheticStereoSequence sequence = test::MakeSyntheticStereoSequence(options);
    mapping::MapperOptions mapper_options;
    mapper_options.initializer.stereo_min_features = 150;
    // The LoopCloser keeps a replica only with loop closing in effect; no
    // detection runs (the map never reaches min_map_frames), no global BA.
    loop_closing::LoopCloserOptions loop_closer_options;
    loop_closer_options.enabled = true;
    loop_closer_options.min_map_frames = 1000;
    global_ba::GlobalBaOptions global_ba_options;
    global_ba_options.enabled = false;
    constexpr size_t kNumFrames = 40;

    // Lifetimes as in EndToEndInitializeTrackLoseReinitialize.
    ProtocolRecorder recorder;
    OrbSlam slam(mapper_options, {}, RootVocabulary(), loop_closer_options, global_ba_options);
    recorder.Attach(slam);
    struct StopAndDrain {
        OrbSlam& slam;
        ProtocolRecorder& recorder;
        ~StopAndDrain() {
            slam.Stop();
            recorder.Poll();
        }
    } stop_and_drain{slam, recorder};
    slam.Start();
    // Frame 0 builds the map.
    slam.tracker().Track(sequence.frames[0].measurement);
    ASSERT_TRUE(recorder.WaitFor([&] { return slam.tracker().state() == tracking::TrackerState::Tracking; },
                                 std::chrono::seconds(120)));
    for (size_t i = 1; i < kNumFrames; ++i) {
        slam.tracker().Track(sequence.frames[i].measurement);
        ASSERT_TRUE(recorder.WaitFor(
            [&] {
                return !recorder.positions.empty() && FrameIndexOf(recorder.positions.back().timestamp) == i;
            },
            std::chrono::seconds(120)))
            << "frame " << i;
    }
    slam.Stop();
    // What the Mapper published before it stopped reaches both replicas.
    while (slam.tracker().ProcessPending()) {
    }
    while (slam.loop_closer().ProcessPending()) {
    }

    const auto& tracker_replica = slam.tracker().replica();
    const auto& loop_closer_replica = slam.loop_closer().replica();
    const std::vector<map::FrameId> frames = tracker_replica.keyframe_ids();
    ASSERT_GE(frames.size(), 2u);
    EXPECT_EQ(loop_closer_replica.keyframe_ids(), frames);
    for (const map::FrameId id : frames) {
        SCOPED_TRACE("Frame " + std::to_string(id.value));
        const auto* mapper_frame = slam.mapper().KeyFrame(id);
        const auto* tracker_frame = tracker_replica.keyframe(id);
        const auto* loop_closer_frame = loop_closer_replica.keyframe(id);
        ASSERT_NE(mapper_frame, nullptr);
        ASSERT_NE(loop_closer_frame, nullptr);
        EXPECT_EQ(&tracker_frame->measurement(), &mapper_frame->measurement());
        EXPECT_EQ(&loop_closer_frame->measurement(), &mapper_frame->measurement());
        EXPECT_GE(mapper_frame->shared_measurement().use_count(), 3);
    }
}

// The leave-the-view run with a vocabulary and a 1 s blackout (frames
// 31-50) soon after the Mapper's latest Frame (mono: frame 30, stereo:
// frame 26), so the camera reappears in the mapped area: the Tracker becomes
// RecentlyLost (one non-final TrackingLost), relocalizes into map 1 and
// tracks to the end. Expects no other map, the first frame tracked after the
// blackout with stage Relocalization, and a centre error from the recovery on
// within `max_error`.
//
// Error bounds: 1.5x the largest of three measured runs (2026-10-08): mono
// 0.0931 / 0.0931 / 0.0931 m (recovery at frame 52: frame 51 reaches only 41
// of the 50 inliers), stereo 0.0137 / 0.0217 / 0.0217 m (recovery at frame 51).
template <typename TSequence>
void ExpectRelocalizesAfterBlackout(const TSequence& sequence, size_t init_frames,
                                    bool monocular, const mapping::MapperOptions& mapper_options,
                                    double max_error) {
    std::vector<geometry::RigidTransformation> truth;
    for (const auto& frame : sequence.frames) truth.push_back(frame.T_cw);
    const Blackout blackout{.begin = 31, .end = 50};

    // The synthetic run makes few Frames (3-4 at the blackout), so any map may
    // relocalize.
    tracking::TrackerOptions tracker_options;
    tracker_options.relocalization.min_map_frames_to_relocalize = 0;

    ProtocolRecorder recorder;
    RunCounts counts;
    RunSequence(sequence, init_frames, mapper_options, recorder, counts,
                {.tracker_options = tracker_options, .vocabulary = RootVocabulary(), .blackout = blackout});
    if (::testing::Test::HasFatalFailure()) return;

    EXPECT_EQ(recorder.map_created, std::vector<uint32_t>{1});
    EXPECT_EQ(recorder.tracking_lost, std::vector<uint32_t>{1});
    EXPECT_EQ(recorder.tracking_lost_final, std::vector<bool>{false});
    for (uint32_t map : recorder.frame_tracked) EXPECT_EQ(map, 1u);
    const auto recovered = std::find_if(
        recorder.positions.begin(), recorder.positions.end(),
        [&](const StampedPose& position) { return FrameIndexOf(position.timestamp) > blackout.end; });
    if (recovered == recorder.positions.end()) {
        ADD_FAILURE() << "no frame tracked after the blackout";
        return;
    }
    EXPECT_EQ(recovered->stage, events::TrackingStage::Relocalization);
    const size_t recovery_frame = FrameIndexOf(recovered->timestamp);
    EXPECT_EQ(FrameIndexOf(recorder.positions.back().timestamp), sequence.frames.size() - 1);
    const double error = MaxCentreError(recorder, 1, truth, monocular, recovery_frame);
    EXPECT_LE(error, max_error);
    ::testing::Test::RecordProperty("recovery_frame", static_cast<int>(recovery_frame));
    ::testing::Test::RecordProperty("max_centre_error_m", std::to_string(error));
}

TEST(SlamTest, MonoRelocalizesAfterBlackout) {
    const test::SyntheticSequence sequence = test::MakeSyntheticSequence(LeavingInitialViewOptions());
    // As in MonoMapGrowsWhileLeavingInitialView.
    mapping::MapperOptions mapper_options;
    mapper_options.initializer.reconstruction.min_parallax_deg = 4.0;
    // Error bound: see ExpectRelocalizesAfterBlackout.
    ExpectRelocalizesAfterBlackout(sequence, /*init_frames=*/30, /*monocular=*/true,
                                   mapper_options, 1.5 * 0.0931);
}

TEST(SlamTest, StereoRelocalizesAfterBlackout) {
    // As StereoMapGrowsWhileLeavingInitialView.
    test::SyntheticStereoSequenceOptions options;
    options.trajectory = LeavingInitialViewOptions();
    options.trajectory.num_points = 700;
    const test::SyntheticStereoSequence sequence = test::MakeSyntheticStereoSequence(options);
    mapping::MapperOptions mapper_options;
    mapper_options.initializer.stereo_min_features = 150;
    // Error bound: see ExpectRelocalizesAfterBlackout.
    ExpectRelocalizesAfterBlackout(sequence, /*init_frames=*/1, /*monocular=*/false,
                                   mapper_options, 1.5 * 0.0217);
}

// The stereo leave-the-view run: every Frame adds close points for the
// stereo features the Tracker did not match, and pixel noise makes it miss
// some, so the map would gather duplicates and points seen by too few Frames
// (measured without fusion: 43 % of the points with >= 3 observations, 112
// point pairs with descriptors within 10 bits; with it 78-86 % and about 18
// such pairs, which fusion's gates leave, see docs/TODO.md). Fusion merges
// them. With pixel noise, twins lie centimetres apart, so the 1 mm
// test only catches exact copies; without noise the run creates none.
TEST(SlamTest, DuplicatesFusedInLongSequence) {
    test::SyntheticStereoSequenceOptions options;
    options.trajectory = LeavingInitialViewOptions();
    options.trajectory.num_points = 700;  // as StereoMapGrowsWhileLeavingInitialView
    const test::SyntheticStereoSequence sequence = test::MakeSyntheticStereoSequence(options);
    mapping::MapperOptions mapper_options;
    mapper_options.initializer.stereo_min_features = 150;

    ProtocolRecorder recorder;
    RunCounts counts;
    RunSequence(sequence, /*init_frames=*/1, mapper_options, recorder, counts);
    if (HasFatalFailure()) return;
    EXPECT_TRUE(recorder.tracking_lost.empty());
    EXPECT_EQ(recorder.map_created, std::vector<uint32_t>{1});

    const auto& replica = recorder.live_map(1);
    ASSERT_TRUE(replica.CheckConsistency());
    std::vector<map::MapPointId> alive;
    for (uint32_t id = 0; id < replica.num_points(); ++id) {
        if (replica.IsAlive(map::MapPointId{id})) alive.push_back(map::MapPointId{id});
    }
    ASSERT_EQ(alive.size(), counts.num_map_points);
    const auto well_observed = std::count_if(alive.begin(), alive.end(), [&](map::MapPointId point) {
        return replica.covisibility().NumObservations(point) >= 3;
    });
    EXPECT_GE(2 * static_cast<size_t>(well_observed), alive.size());

    size_t duplicates = 0;
    for (size_t i = 0; i < alive.size(); ++i) {
        for (size_t j = i + 1; j < alive.size(); ++j) {
            if ((replica.Position(alive[i]) - replica.Position(alive[j])).norm() < 1e-3 &&
                features::OrbFeatureExtractor::Distance(replica.Descriptor(alive[i]),
                                                        replica.Descriptor(alive[j])) <= 10) {
                ++duplicates;
            }
        }
    }
    EXPECT_EQ(duplicates, 0u) << "of " << alive.size() << " points";
}

// 64 words, one per (byte 0, byte 1) pair of test::kSyntheticWordCodes, so a
// place_words scene's places differ in their BoW vectors (as the LoopCloser
// tests' MakeRegionVocabulary). Declared with L = 5, so levels_up 4 files
// every feature under its word and SearchByBoW compares only features of one
// word.
std::shared_ptr<const OrbSlam::TMapper::TVocabulary> PlaceVocabulary() {
    using TVocabulary = OrbSlam::TMapper::TVocabulary;
    using TDescriptors = features::OrbFeatureExtractor::TDescriptors;
    constexpr size_t kNumCodes = std::size(test::kSyntheticWordCodes);
    constexpr size_t kNumWords = kNumCodes * kNumCodes;
    TDescriptors descriptors = TDescriptors::Zero(features::OrbFeatureExtractor::DescriptorSize,
                                                  static_cast<Eigen::Index>(1 + kNumWords));
    std::vector<double> weights(1 + kNumWords, 1.0);
    weights[0] = 0.0;
    for (size_t word = 0; word < kNumWords; ++word) {
        descriptors(0, static_cast<Eigen::Index>(1 + word)) = test::kSyntheticWordCodes[word / kNumCodes];
        descriptors(1, static_cast<Eigen::Index>(1 + word)) = test::kSyntheticWordCodes[word % kNumCodes];
    }
    return std::make_shared<const TVocabulary>(TVocabulary::FromTree(
        static_cast<int>(kNumWords), 5, std::vector<bow::NodeId>(1 + kNumWords, 0), std::move(descriptors),
        std::move(weights)));
}

// A loop in the room: the camera circles 1.5 m about its centre looking
// outward, 1.5 deg per frame, turning 450 deg: frame k + kLapFrames repeats
// frame k's true pose, so the last 90 deg revisit the first lap's view.
constexpr size_t kLapFrames = 240;
test::SyntheticSequenceOptions LoopOptions() {
    test::SyntheticSequenceOptions options;
    options.surround_scene = true;
    options.loop_radius = 1.5;
    options.place_words = true;
    options.turn_deg = 450.0;
    options.num_frames = 301;
    options.num_points = 1000;
    return options;
}

// What a loop run measured.
struct LoopRun {
    // Mean over the last kEndFrames tracked frames of the camera-centre error,
    // in true units, relative to the first-lap Frame nearest to the same true
    // pose, as the final map holds it: the drift between where the camera
    // ends and the map of where it started.
    double end_error = 0.0;
    size_t num_end_pairs = 0;  // end frames with a first-lap reference: end_error's count
    // Mean over those frames of the map's scale there (the distance moved
    // over kScaleSpan frames, per true metre) over its scale at the
    // reference: the scale drift between the start and the end of the run,
    // 1 if none.
    double end_scale_ratio = 1.0;
    size_t num_loops = 0;    // LoopDetected
    size_t num_corrections = 0;
    size_t num_lost = 0;     // TrackingLost
    size_t num_maps = 0;     // maps created
    size_t loop_frame = 0;   // the first loop's query Frame, as a frame index
    double loop_scale = 1.0;  // its Sim3 scale
    bool tracked_to_end = false;
    // RMS distance of the final map's points to their true positions, in
    // true units (PointsRmsError), and the number of points it covers.
    double points_rms = 0.0;
    size_t num_rms_points = 0;
    RunCounts counts;
};

// How a loop run closes loops: not at all (the default options), by the
// essential graph only (global BA disabled), or by the essential graph and
// then global BA (the default GlobalBaOptions).
enum class LoopMode { kOpen, kEssentialGraph, kGlobalBa };

// The RMS distance, in true units, of the replica's alive points to their
// true positions, after the transform that best aligns the two sets (Umeyama,
// least squares): a similarity for a monocular map, rigid for a stereo one.
// A point's true position is that of the scene point most of its observations
// see; a point whose observations mostly see extras is skipped. Sets
// `out_count` to the number of points used.
template <typename TSequence>
double PointsRmsError(const TSequence& sequence, const ProtocolRecorder& recorder, bool monocular,
                      size_t& out_count) {
    const auto& replica = recorder.live_map(1);  // a loop run's one map
    // Per point id: the scene points its observations see, with their counts.
    std::map<uint32_t, std::map<Eigen::Index, size_t>> votes;
    for (const map::FrameId id : replica.keyframe_ids()) {
        const auto* frame = replica.keyframe(id);
        const double timestamp = std::visit([](const auto& m) { return m.timestamp(); }, frame->measurement());
        const auto& point_of_feature = sequence.frames.at(FrameIndexOf(timestamp)).point_of_feature;
        for (const auto& [feature, point] : frame->observations) {
            if (replica.IsAlive(point)) ++votes[point.value][point_of_feature.at(feature)];
        }
    }
    std::vector<std::pair<geometry::TPoint3DF, geometry::TPoint3DF>> pairs;  // (estimated, true)
    for (const auto& [point, counts] : votes) {
        const auto best = std::max_element(counts.begin(), counts.end(),
                                           [](const auto& a, const auto& b) { return a.second < b.second; });
        if (best->first < 0) continue;
        pairs.emplace_back(replica.Position(map::MapPointId{point}), sequence.points_w.col(best->first));
    }
    out_count = pairs.size();
    if (pairs.size() < 3) return 0.0;
    Eigen::Matrix3Xd estimated(3, static_cast<Eigen::Index>(pairs.size()));
    Eigen::Matrix3Xd truth(3, static_cast<Eigen::Index>(pairs.size()));
    for (size_t i = 0; i < pairs.size(); ++i) {
        estimated.col(static_cast<Eigen::Index>(i)) = pairs[i].first;
        truth.col(static_cast<Eigen::Index>(i)) = pairs[i].second;
    }
    const Eigen::Matrix4d T_truth_estimated = Eigen::umeyama(estimated, truth, /*with_scaling=*/monocular);
    const Eigen::Matrix3Xd aligned =
        (T_truth_estimated.topLeftCorner<3, 3>() * estimated).colwise() + T_truth_estimated.topRightCorner<3, 1>();
    return std::sqrt((aligned - truth).colwise().squaredNorm().mean());
}

constexpr size_t kEndFrames = 20;
constexpr size_t kScaleSpan = 15;

template <typename TSequence>
LoopRun RunLoop(const TSequence& sequence, size_t init_frames, bool monocular,
                const mapping::MapperOptions& mapper_options, LoopMode mode) {
    std::vector<geometry::RigidTransformation> truth;
    for (const auto& frame : sequence.frames) truth.push_back(frame.T_cw);
    // Without loop closing: the default options, which leave it off (opt-in)
    // although the run has a vocabulary, and global BA with it.
    loop_closing::LoopCloserOptions loop_closer_options;
    global_ba::GlobalBaOptions global_ba_options;
    if (mode != LoopMode::kOpen) loop_closer_options.enabled = true;
    if (mode == LoopMode::kEssentialGraph) global_ba_options.enabled = false;
    ProtocolRecorder recorder;
    RunCounts counts;
    RunSequence(sequence, init_frames, mapper_options, recorder, counts,
                {.vocabulary = PlaceVocabulary(),
                 .loop_closer_options = loop_closer_options,
                 .global_ba_options = global_ba_options,
                 .lock_step = true});
    LoopRun run;
    run.counts = counts;
    run.points_rms = PointsRmsError(sequence, recorder, monocular, run.num_rms_points);
    run.num_loops = recorder.loops.size();
    run.num_corrections = recorder.corrections.size();
    run.num_lost = recorder.tracking_lost.size();
    run.num_maps = recorder.map_created.size();
    // The final map's Frames by frame index.
    std::vector<std::optional<geometry::RigidTransformation>> frame_pose(sequence.frames.size());
    std::vector<StampedPose> keyframes;
    for (const StampedPose& keyframe : recorder.keyframes) {
        if (keyframe.map != 1) continue;
        keyframes.push_back(keyframe);
        if (const auto* frame = recorder.live_map(1).keyframe(keyframe.frame)) {
            frame_pose[FrameIndexOf(keyframe.timestamp)] = frame->T_cw;
        }
        if (!recorder.loops.empty() && keyframe.frame == recorder.loops.front().query_frame) {
            run.loop_frame = FrameIndexOf(keyframe.timestamp);
            run.loop_scale = recorder.loops.front().S_query_match.scale;
        }
    }
    if (keyframes.empty() || recorder.positions.empty()) return run;
    const size_t first = FrameIndexOf(keyframes[0].timestamp);
    const size_t last = sequence.frames.size() - 1;
    run.tracked_to_end = FrameIndexOf(recorder.positions.back().timestamp) == last;
    // True positions with the first Frame's camera as the world.
    const geometry::RigidTransformation T_first_w = truth[first];
    const auto true_centre = [&](size_t k) { return test::MapAlignment::Centre(truth[k] * T_first_w.Inverse()); };
    // The first-lap Frame nearest to frame `target`.
    const auto nearest_frame = [&](size_t target) {
        std::optional<size_t> nearest;
        const auto distance = [target](size_t j) { return j > target ? j - target : target - j; };
        for (size_t j = 0; j < kLapFrames / 2; ++j) {
            if (frame_pose[j] && (!nearest || distance(j) < distance(*nearest))) nearest = j;
        }
        return nearest;
    };
    // The latest tracked centre per frame index, map 1.
    std::vector<std::optional<geometry::TVector3DF>> tracked(sequence.frames.size());
    for (const StampedPose& position : recorder.positions) {
        if (position.map == 1) tracked[FrameIndexOf(position.timestamp)] = test::MapAlignment::Centre(position.pose);
    }
    double sum = 0.0, ratio_sum = 0.0;
    size_t n = 0, num_ratios = 0;
    for (const StampedPose& position : recorder.positions) {
        const size_t k = FrameIndexOf(position.timestamp);
        if (position.map != 1 || k + kEndFrames <= last) continue;
        // The first-lap Frame nearest to frame k's true pose (frame k -
        // kLapFrames), and one about kScaleSpan frames before it: their
        // distance gives the final map's scale there (mono: a loop
        // correction rescales the start too).
        const std::optional<size_t> reference = nearest_frame(k - kLapFrames);
        const std::optional<size_t> other = nearest_frame(k - kLapFrames - kScaleSpan);
        if (!reference || !other || *reference == *other) continue;
        const geometry::TVector3DF reference_centre = test::MapAlignment::Centre(*frame_pose[*reference]);
        const double scale =
            monocular ? (reference_centre - test::MapAlignment::Centre(*frame_pose[*other])).norm() /
                            (true_centre(*reference) - true_centre(*other)).norm()
                      : 1.0;
        const geometry::TVector3DF estimated = (test::MapAlignment::Centre(position.pose) - reference_centre) / scale;
        sum += (estimated - (true_centre(k) - true_centre(*reference))).norm();
        ++n;
        if (tracked[k - kScaleSpan]) {
            const double end_scale = (test::MapAlignment::Centre(position.pose) - *tracked[k - kScaleSpan]).norm() /
                                     (true_centre(k) - true_centre(k - kScaleSpan)).norm();
            ratio_sum += end_scale / scale;
            ++num_ratios;
        }
    }
    run.end_error = n > 0 ? sum / static_cast<double>(n) : 0.0;
    run.num_end_pairs = n;
    run.end_scale_ratio = num_ratios > 0 ? ratio_sum / static_cast<double>(num_ratios) : 1.0;
    return run;
}

// Runs `sequence` (a LoopOptions loop) without loop closing (the default
// options: it is opt-in) and with it enabled, by the essential graph only
// (global BA disabled), and expects: without, no
// LoopDetected and tracking to the end; with, the loop detected
// on the return to the first lap's view and corrected, one map, no tracking
// loss across the correction (the Tracker follows a real Mapper correction),
// and an end error, measured, at most a third of the run without. Records
// the numbers under `seed`, the sequence's; returns both runs.
struct LoopRuns {
    LoopRun open;    // without loop closing
    LoopRun closed;  // with it
};
template <typename TSequence>
LoopRuns ExpectClosesLoop(const TSequence& sequence, uint32_t seed, size_t init_frames, bool monocular,
                          const mapping::MapperOptions& mapper_options) {
    const LoopRun open = RunLoop(sequence, init_frames, monocular, mapper_options, LoopMode::kOpen);
    if (::testing::Test::HasFatalFailure()) return {};
    const LoopRun closed = RunLoop(sequence, init_frames, monocular, mapper_options, LoopMode::kEssentialGraph);
    if (::testing::Test::HasFatalFailure()) return {};

    EXPECT_EQ(open.num_loops, 0u);
    EXPECT_EQ(open.num_corrections, 0u);
    // No global BA without loop closing (its default options), nor when disabled.
    EXPECT_FALSE(open.counts.has_global_ba);
    EXPECT_FALSE(closed.counts.has_global_ba);
    EXPECT_EQ(open.counts.global_ba_merges + open.counts.global_ba_rejected, 0u);
    EXPECT_EQ(closed.counts.global_ba_merges + closed.counts.global_ba_rejected, 0u);
    EXPECT_EQ(open.num_maps, 1u);
    EXPECT_EQ(open.num_lost, 0u);
    EXPECT_TRUE(open.tracked_to_end);

    EXPECT_GE(closed.num_loops, 1u);
    EXPECT_GE(closed.num_corrections, 1u);
    EXPECT_GE(closed.loop_frame, kLapFrames - 30);  // on the return
    EXPECT_EQ(closed.num_maps, 1u);
    EXPECT_EQ(closed.num_lost, 0u);
    EXPECT_TRUE(closed.tracked_to_end);
    // Both errors are measured (end frames with a first-lap reference).
    EXPECT_GE(open.num_end_pairs, kEndFrames / 2);
    EXPECT_GE(closed.num_end_pairs, kEndFrames / 2);
    EXPECT_GT(open.end_error, 0.0);
    EXPECT_GT(closed.end_error, 0.0);
    EXPECT_LE(closed.end_error, open.end_error / 3.0);
    if (!monocular) {
        EXPECT_DOUBLE_EQ(closed.loop_scale, 1.0);
    }
    const std::string prefix = "seed_" + std::to_string(seed) + "_";
    ::testing::Test::RecordProperty(prefix + "end_error_without_m", std::to_string(open.end_error));
    ::testing::Test::RecordProperty(prefix + "end_error_with_m", std::to_string(closed.end_error));
    ::testing::Test::RecordProperty(prefix + "loop_frame", static_cast<int>(closed.loop_frame));
    ::testing::Test::RecordProperty(prefix + "loop_scale", std::to_string(closed.loop_scale));
    ::testing::Test::RecordProperty(prefix + "end_scale_ratio_without", std::to_string(open.end_scale_ratio));
    ::testing::Test::RecordProperty(prefix + "end_scale_ratio_with", std::to_string(closed.end_scale_ratio));
    return {open, closed};
}

// Seeds 1-3 of the loop. Mono drift is a random walk whose scale part varies
// by scene, so the scale correction is expected over the range: some seed
// drifts in scale by more than 2 % between the start and the end of the run
// (end_scale_ratio, against truth) without loop closing, and with it every
// such drift is cut to at most a third.
//
// Measured (2026-10-09, three runs each, identical to 4 digits; end error
// without / with loop closing, end-of-run scale drift without / with): seed 1
// 0.0676 / 0.0101 m, 5.5 / 0.4 % (loop at frame 236); seed 2 0.1139 / 0.0157
// m, 0.7 / 1.4 % (235); seed 3 0.1429 / 0.0060 m, 4.6 / 0.5 % (225). One run
// each of seeds 4-5: 0.0876 / 0.0154 and 0.1447 / 0.0044 m.
TEST(SlamTest, MonoClosesLoop) {
    // As in MonoMapGrowsWhileLeavingInitialView: a well-conditioned initial map.
    mapping::MapperOptions mapper_options;
    mapper_options.initializer.reconstruction.min_parallax_deg = 4.0;
    constexpr double kScaleDrift = 0.02;  // log scale
    double max_scale_drift = 0.0;
    for (uint32_t seed = 1; seed <= 3; ++seed) {
        SCOPED_TRACE("seed " + std::to_string(seed));
        test::SyntheticSequenceOptions options = LoopOptions();
        options.seed = seed;
        const LoopRuns runs = ExpectClosesLoop(test::MakeSyntheticSequence(options), seed, /*init_frames=*/30,
                                               /*monocular=*/true, mapper_options);
        if (HasFatalFailure()) return;
        const double drift_without = std::abs(std::log(runs.open.end_scale_ratio));
        const double drift_with = std::abs(std::log(runs.closed.end_scale_ratio));
        max_scale_drift = std::max(max_scale_drift, drift_without);
        if (drift_without > kScaleDrift) {
            EXPECT_LE(drift_with, drift_without / 3.0);
        }
    }
    EXPECT_GT(max_scale_drift, kScaleDrift);
}

// Measured (2026-10-09, end error without / with loop closing, three runs):
// 0.0157 / 0.0023 m each, loop at frame 264. Seeds 2-5 (margin check, one
// run each): 0.0252 / 0.0043, 0.0457 / 0.0115, 0.0186 / 0.0033, 0.0146 /
// 0.0019 m (ratio at most 0.25).
TEST(SlamTest, StereoClosesLoop) {
    test::SyntheticStereoSequenceOptions options;
    options.trajectory = LoopOptions();
    options.trajectory.num_points = 700;  // as StereoMapGrowsWhileLeavingInitialView
    const test::SyntheticStereoSequence sequence = test::MakeSyntheticStereoSequence(options);
    mapping::MapperOptions mapper_options;
    mapper_options.initializer.stereo_min_features = 150;
    ExpectClosesLoop(sequence, options.trajectory.seed, /*init_frames=*/1, /*monocular=*/false, mapper_options);
}

// The GlobalBundleAdjuster exists only with loop closing in effect (a
// vocabulary and LoopCloserOptions::enabled) and global BA enabled: not by
// default (loop closing off), not with global BA disabled, not without a
// vocabulary (no loop can be detected). That no GlobalBAFinished reaches the
// Mapper over a whole run without one is checked by ExpectClosesLoop.
TEST(SlamTest, GlobalBaCreatedOnlyWithLoopClosing) {
    OrbSlam by_default;
    EXPECT_EQ(by_default.global_ba(), nullptr);
    OrbSlam global_ba_disabled({}, {}, PlaceVocabulary(), loop_closing::LoopCloserOptions{.enabled = true},
                               global_ba::GlobalBaOptions{.enabled = false});
    EXPECT_EQ(global_ba_disabled.global_ba(), nullptr);
    OrbSlam no_vocabulary({}, {}, nullptr, loop_closing::LoopCloserOptions{.enabled = true});
    EXPECT_EQ(no_vocabulary.global_ba(), nullptr);
    OrbSlam loop_closing_on({}, {}, PlaceVocabulary(), loop_closing::LoopCloserOptions{.enabled = true});
    EXPECT_NE(loop_closing_on.global_ba(), nullptr);
    // Wired and started like the other components.
    loop_closing_on.Start();
    loop_closing_on.Stop();
}

// Runs `sequence` (a LoopOptions loop) with loop closing by the essential
// graph only and with global BA after each correction, and expects global BA
// to start and be merged, one map and no tracking loss across the merge, the
// map points' RMS error against truth at most `max_points_ratio` times the
// essential graph's and the end error at most `max_end_ratio` times its
// (spec section 5). Records the numbers under `seed`; returns the points RMS
// ratio (global BA over essential graph), 0 if a run failed fatally.
template <typename TSequence>
double ExpectGlobalBaRefinesLoop(const TSequence& sequence, uint32_t seed, size_t init_frames, bool monocular,
                                 const mapping::MapperOptions& mapper_options, double max_points_ratio,
                                 double max_end_ratio) {
    const LoopRun essential = RunLoop(sequence, init_frames, monocular, mapper_options, LoopMode::kEssentialGraph);
    if (::testing::Test::HasFatalFailure()) return 0.0;
    const LoopRun refined = RunLoop(sequence, init_frames, monocular, mapper_options, LoopMode::kGlobalBa);
    if (::testing::Test::HasFatalFailure()) return 0.0;

    EXPECT_FALSE(essential.counts.has_global_ba);
    EXPECT_GE(essential.num_corrections, 1u);
    // A sound baseline: a second map or a loss would inflate its errors.
    EXPECT_EQ(essential.num_maps, 1u);
    EXPECT_EQ(essential.num_lost, 0u);
    EXPECT_TRUE(essential.tracked_to_end);
    EXPECT_TRUE(refined.counts.has_global_ba);
    EXPECT_GE(refined.num_loops, 1u);
    EXPECT_GE(refined.counts.global_ba_runs_started, 1u);
    EXPECT_GE(refined.counts.global_ba_merges, 1u);
    EXPECT_EQ(refined.num_maps, 1u);
    EXPECT_EQ(refined.num_lost, 0u);
    EXPECT_TRUE(refined.tracked_to_end);
    // Both errors are measured.
    EXPECT_GE(essential.num_end_pairs, kEndFrames / 2);
    EXPECT_GE(refined.num_end_pairs, kEndFrames / 2);
    EXPECT_GE(essential.num_rms_points, 100u);
    EXPECT_GE(refined.num_rms_points, 100u);
    EXPECT_GT(essential.points_rms, 0.0);
    EXPECT_LE(refined.points_rms, max_points_ratio * essential.points_rms);
    EXPECT_LE(refined.end_error, max_end_ratio * essential.end_error);

    const std::string prefix = "seed_" + std::to_string(seed) + "_";
    const auto record = [&](const std::string& key, const auto& value) {
        ::testing::Test::RecordProperty(prefix + key, std::to_string(value));
    };
    record("points_rms_essential_m", essential.points_rms);
    record("points_rms_global_ba_m", refined.points_rms);
    record("end_error_essential_m", essential.end_error);
    record("end_error_global_ba_m", refined.end_error);
    record("num_rms_points_essential", essential.num_rms_points);
    record("num_rms_points_global_ba", refined.num_rms_points);
    record("loops_global_ba", refined.num_loops);
    record("global_ba_runs_started", refined.counts.global_ba_runs_started);
    record("global_ba_runs_interrupted", refined.counts.global_ba_runs_interrupted);
    record("global_ba_results_dropped", refined.counts.global_ba_results_dropped);
    record("global_ba_merges", refined.counts.global_ba_merges);
    record("global_ba_rejected", refined.counts.global_ba_rejected);
    return essential.points_rms > 0.0 ? refined.points_rms / essential.points_rms : 0.0;
}

// Seeds 1-3 of MonoClosesLoop's loop. Mono bars (spec section 5, revised
// after measurement): points RMS <= 0.95x the essential graph's per seed and
// <= 0.92x on average, end error <= 1.05x per seed. The RMS is dominated by
// low-parallax points of a 15-16 Frame map and global BA has converged (10
// and 100 iterations give the same result).
//
// Measured (2026-10-10; points RMS essential graph / global BA, end error
// essential graph / global BA): seed 1 0.0734 / 0.0652 m (0.89x), 0.0101 /
// 0.0058 m; seed 2 0.0770 / 0.0673 m (0.87x), 0.0157 / 0.0055 m; seed 3
// 0.0789 / 0.0727 m (0.92x), 0.00601 / 0.00604 m (1.005x). Mean points ratio
// 0.894.
TEST(SlamTest, MonoGlobalBaRefinesLoop) {
    mapping::MapperOptions mapper_options;
    mapper_options.initializer.reconstruction.min_parallax_deg = 4.0;  // as MonoClosesLoop
    constexpr uint32_t kNumSeeds = 3;
    double ratio_sum = 0.0;
    for (uint32_t seed = 1; seed <= kNumSeeds; ++seed) {
        SCOPED_TRACE("seed " + std::to_string(seed));
        test::SyntheticSequenceOptions options = LoopOptions();
        options.seed = seed;
        ratio_sum += ExpectGlobalBaRefinesLoop(test::MakeSyntheticSequence(options), seed, /*init_frames=*/30,
                                               /*monocular=*/true, mapper_options, /*max_points_ratio=*/0.95,
                                               /*max_end_ratio=*/1.05);
        if (HasFatalFailure()) return;
    }
    const double mean_ratio = ratio_sum / kNumSeeds;
    RecordProperty("mean_points_rms_ratio", std::to_string(mean_ratio));
    EXPECT_LE(mean_ratio, 0.92);
}

// Seeds 1-3 of StereoClosesLoop's loop. Stereo bars (spec section 5): points
// RMS <= 0.8x the essential graph's, end error no worse. Measured
// (2026-10-10; points RMS essential graph / global BA, end error essential
// graph / global BA): seed 1 0.0705 / 0.0290 m (0.41x), 0.0023 / 0.0018 m;
// seed 2 0.0713 / 0.0339 m (0.48x), 0.0043 / 0.0029 m; seed 3 0.0782 /
// 0.0341 m (0.44x), 0.0115 / 0.0021 m.
TEST(SlamTest, StereoGlobalBaRefinesLoop) {
    mapping::MapperOptions mapper_options;
    mapper_options.initializer.stereo_min_features = 150;  // as StereoClosesLoop
    for (uint32_t seed = 1; seed <= 3; ++seed) {
        SCOPED_TRACE("seed " + std::to_string(seed));
        test::SyntheticStereoSequenceOptions options;
        options.trajectory = LoopOptions();
        options.trajectory.num_points = 700;
        options.trajectory.seed = seed;
        ExpectGlobalBaRefinesLoop(test::MakeSyntheticStereoSequence(options), seed, /*init_frames=*/1,
                                  /*monocular=*/false, mapper_options, /*max_points_ratio=*/0.8,
                                  /*max_end_ratio=*/1.0);
        if (HasFatalFailure()) return;
    }
}

} // namespace
} // namespace voyahslam::slam
