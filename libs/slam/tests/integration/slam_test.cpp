#include <support/synthetic_sequence.h>
#include <support/test_measurements.h>
#include <support/trajectory_error.h>
#include <voyahslam/bow/vocabulary.h>
#include <voyahslam/events/async_observer.h>
#include <voyahslam/events/events.h>
#include <voyahslam/features/orb_feature_extractor.h>
#include <voyahslam/features/sift_feature_extractor.h>
#include <voyahslam/slam/slam.h>
#include <voyahslam/tracking/map_replica.h>

#include <gtest/gtest.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
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

// Records the map ids of the protocol events published by the Mapper and the
// Tracker, read through external observers, and keeps a replica of the
// Mapper's map.
class ProtocolRecorder {
public:
    // Before slam.Start().
    void Attach(OrbSlam& slam) {
        slam.mapper().AddObserver(mapper_events_);
        slam.tracker().AddObserver(tracker_events_);
    }

    // Reads every pending event; call only from the test thread.
    void Poll() {
        while (events::Event* event = mapper_listener_.GetNextEvent()) {
            if (event->type != events::EventType::MapTransaction) {
                continue;
            }
            const auto& transaction =
                *static_cast<events::MapTransaction<features::OrbFeatureExtractor>*>(event);
            replica.Apply(*event);
            const uint32_t map = event->map_id.value;
            const bool announced = !map_created.empty() && map_created.back() == map;
            if (transaction.new_map) {
                map_created.push_back(map);
            }
            for (const auto& frame : transaction.added_frames) {
                keyframes.push_back({map, frame.T_cw,
                                     std::visit([](const auto& m) { return m.timestamp(); },
                                                frame.measurement)});
            }
            transaction_new_points.push_back(transaction.num_new_points());
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
    // The Mapper's current map, rebuilt from its transactions.
    tracking::MapReplica<features::OrbFeatureExtractor> replica;

private:
    // Listeners after their observers: destroyed first, releasing the
    // event they hold.
    events::AsyncObserver mapper_events_;
    events::AsyncObserver tracker_events_;
    events::Listener mapper_listener_ = mapper_events_.GetListener();
    events::Listener tracker_listener_ = tracker_events_.GetListener();
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

// Feeds a sequence to a new Slam: the first `init_frames` frames build the
// map, then every later frame is fed once it tracks (the frames of
// `blackout`, if any, without features). Returns when the last frame is
// tracked or tracking is lost for good (TrackingLost with `final`), with the
// Slam stopped and `out_recorder` drained.
template <typename TSequence>
void RunSequence(const TSequence& sequence, size_t init_frames,
                 const mapping::MapperOptions& mapper_options, ProtocolRecorder& out_recorder,
                 RunCounts& out_counts, const tracking::TrackerOptions& tracker_options = {},
                 std::shared_ptr<const OrbSlam::TMapper::TVocabulary> vocabulary = nullptr,
                 std::optional<Blackout> blackout = std::nullopt) {
    // Lifetimes as in EndToEndInitializeTrackLoseReinitialize.
    OrbSlam slam(mapper_options, tracker_options, std::move(vocabulary));
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
    for (size_t i = init_frames; i < sequence.frames.size(); ++i) {
        if (blackout && i >= blackout->begin && i <= blackout->end) {
            slam.tracker().Track(EmptyMeasurement(sequence, sequence.frames[i].measurement.timestamp()));
        } else {
            slam.tracker().Track(sequence.frames[i].measurement);
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
    slam.Stop();
    out_recorder.Poll();
    out_counts.num_keyframes = slam.mapper().num_keyframes();
    out_counts.num_map_points = slam.mapper().num_map_points();
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
    RunSequence(sequence, init_frames, mapper_options, recorder, counts, tracker_options,
                RootVocabulary(), blackout);
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

    const auto& replica = recorder.replica;
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

} // namespace
} // namespace voyahslam::slam
