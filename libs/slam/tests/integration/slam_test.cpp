#include <support/synthetic_sequence.h>
#include <support/test_measurements.h>
#include <support/trajectory_error.h>
#include <voyahslam/events/async_observer.h>
#include <voyahslam/events/events.h>
#include <voyahslam/features/orb_feature_extractor.h>
#include <voyahslam/features/sift_feature_extractor.h>
#include <voyahslam/slam/slam.h>

#include <gtest/gtest.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <thread>
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
// Tracker, read through external observers.
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
                positions.push_back({event->map_id.value, tracked.pose, tracked.timestamp});
            } else if (event->type == events::EventType::TrackingLost) {
                tracking_lost.push_back(event->map_id.value);
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
    std::vector<StampedPose> keyframes;  // added Frames, in publication order
    std::vector<size_t> transaction_new_points;  // per transaction, in publication order
    std::vector<StampedPose> positions;

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
// Error bounds: without local BA (sub-project 3) every new Frame and point
// inherits the pose noise, so the error random-walks with the 0.5 px pixel
// noise. Measured: 1.21 m mono (20 % of the 5.95 m path) and 0.15 m stereo;
// 0 for both without pixel noise. The bounds (30 % of the path, 0.3 m) leave
// a 1.5x margin; the plan's 5 % and 2 cm wait for local BA (docs/TODO.md).
test::SyntheticSequenceOptions LeavingInitialViewOptions() {
    test::SyntheticSequenceOptions options;
    options.num_frames = 120;
    options.turn_deg = 90.0;
    options.surround_scene = true;
    options.num_points = 1000;
    return options;
}

// Largest camera-centre error, in true units, of `map`'s FrameTracked poses,
// aligned through the map's first keyframe (stereo) or first two keyframes
// (mono: scale from the second keyframe's baseline).
double MaxCentreError(const ProtocolRecorder& recorder, uint32_t map,
                      const std::vector<geometry::RigidTransformation>& truth, bool monocular) {
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
        max_error = std::max(max_error, alignment.CentreError(position.pose, truth[k]));
    }
    return max_error;
}

// Feeds a sequence to a running Slam: the first `init_frames` frames build
// the map, then every later frame must be tracked. Expects map 1 only, no
// TrackingLost, more than the initial Frames and points, and a centre error
// within `max_error`.
template <typename TSequence>
void ExpectMapGrowsWhileLeavingInitialView(const TSequence& sequence, size_t init_frames,
                                           bool monocular,
                                           const mapping::MapperOptions& mapper_options,
                                           double max_error) {
    std::vector<geometry::RigidTransformation> truth;
    for (const auto& frame : sequence.frames) truth.push_back(frame.T_cw);

    // Lifetimes as in EndToEndInitializeTrackLoseReinitialize.
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
    slam.Start();

    // The Tracker forwards frames to the Mapper until the map arrives, so the
    // frames to track are fed only once it tracks.
    for (size_t i = 0; i < init_frames; ++i) {
        slam.tracker().Track(sequence.frames[i].measurement);
    }
    ASSERT_TRUE(recorder.WaitFor(
        [&] { return slam.tracker().state() == tracking::TrackerState::Tracking; },
        std::chrono::seconds(120)));
    const size_t tracked_before = recorder.positions.size();
    for (size_t i = init_frames; i < sequence.frames.size(); ++i) {
        slam.tracker().Track(sequence.frames[i].measurement);
    }
    const size_t to_track = sequence.frames.size() - init_frames;
    ASSERT_TRUE(recorder.WaitFor(
        [&] {
            return !recorder.tracking_lost.empty() ||
                   recorder.positions.size() >= tracked_before + to_track;
        },
        std::chrono::seconds(120)));
    slam.Stop();
    recorder.Poll();

    EXPECT_TRUE(recorder.tracking_lost.empty());
    EXPECT_EQ(recorder.positions.size() - tracked_before, to_track);
    EXPECT_EQ(recorder.map_created, std::vector<uint32_t>{1});
    ASSERT_GE(recorder.keyframes.size(), monocular ? 2u : 1u);
    // The new-map transaction carries every initial point.
    const size_t initial_points = recorder.transaction_new_points.at(0);
    EXPECT_GT(slam.mapper().num_keyframes(), 2u);
    EXPECT_GT(slam.mapper().num_map_points(), initial_points);
    EXPECT_LE(MaxCentreError(recorder, 1, truth, monocular), max_error);
}

TEST(SlamTest, MonoMapGrowsWhileLeavingInitialView) {
    const test::SyntheticSequenceOptions options = LeavingInitialViewOptions();
    const test::SyntheticSequence sequence = test::MakeSyntheticSequence(options);
    // As in EndToEndInitializeTrackLoseReinitialize: a well-conditioned initial map.
    mapping::MapperOptions mapper_options;
    mapper_options.initializer.reconstruction.min_parallax_deg = 4.0;
    const double path_length = static_cast<double>(options.num_frames - 1) * options.step;
    // The map is initialized within the first 30 frames. Error bound: see
    // LeavingInitialViewOptions.
    ExpectMapGrowsWhileLeavingInitialView(sequence, /*init_frames=*/30, /*monocular=*/true,
                                          mapper_options, 0.3 * path_length);
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
    ExpectMapGrowsWhileLeavingInitialView(sequence, /*init_frames=*/1, /*monocular=*/false,
                                          mapper_options, 0.3);
}

} // namespace
} // namespace voyahslam::slam
