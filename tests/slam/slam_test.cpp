#include <support/synthetic_sequence.h>
#include <support/test_measurements.h>
#include <support/trajectory_error.h>
#include <voyahslam/events/async_observer.h>
#include <voyahslam/events/events.h>
#include <voyahslam/features/orb_feature_extractor.h>
#include <voyahslam/features/sift_feature_extractor.h>
#include <voyahslam/slam/slam.h>

#include <gtest/gtest.h>

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

// A pose published for a map: KeyFrameCreated (Mapper) or PositionUpdated (Tracker).
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
            if (event->type == events::EventType::MapCreated) {
                map_created.push_back(event->map_id.value);
            } else if (event->type == events::EventType::KeyFrameCreated) {
                const auto& created =
                    *static_cast<events::KeyFrameCreated<features::OrbFeatureExtractor>*>(event);
                keyframes.push_back({event->map_id.value, created.pose,
                                     std::visit([](const auto& m) { return m.timestamp(); },
                                                created.measurement)});
            } else if (event->type == events::EventType::MapPointsAdded) {
                map_points_added.push_back(event->map_id.value);
                // Map points before the MapCreated that announces their map.
                map_points_added_before_map_created.push_back(
                    map_created.empty() || map_created.back() != event->map_id.value);
            }
        }
        while (events::Event* event = tracker_listener_.GetNextEvent()) {
            if (event->type == events::EventType::PositionUpdated) {
                const auto& position = *static_cast<events::PositionUpdated*>(event);
                position_updated.push_back(event->map_id.value);
                positions.push_back({event->map_id.value, position.pose, position.timestamp});
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

    std::vector<uint32_t> map_created;
    std::vector<uint32_t> map_points_added;
    std::vector<bool> map_points_added_before_map_created;
    std::vector<uint32_t> position_updated;
    std::vector<uint32_t> tracking_lost;
    std::vector<StampedPose> keyframes;  // in publication order
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

// Expects every PositionUpdated of `map` to be near ground truth (`truth`,
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
    EXPECT_EQ(recorder.map_points_added, std::vector<uint32_t>{1});
    EXPECT_EQ(recorder.map_points_added_before_map_created, std::vector<bool>{true});
    ASSERT_TRUE(recorder.WaitFor([&] {
        return tracker.state() == tracking::TrackerState::Tracking &&
               tracker.current_map_id() == map::MapId{1};
    }));
    EXPECT_EQ(mapper.state(), mapping::MapperState::Mapping);
    EXPECT_EQ(mapper.current_map_id(), map::MapId{1});

    // Tracking on map 1.
    tracker.Track(frame(kMap1End + 1));
    tracker.Track(frame(kMap1End + 2));
    ASSERT_TRUE(recorder.WaitFor([&] { return recorder.position_updated.size() >= 2; }));
    for (uint32_t map : recorder.position_updated) {
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
    EXPECT_EQ(recorder.map_points_added, (std::vector<uint32_t>{1, 2}));
    EXPECT_EQ(recorder.map_points_added_before_map_created, (std::vector<bool>{true, true}));
    ASSERT_TRUE(recorder.WaitFor([&] {
        return tracker.state() == tracking::TrackerState::Tracking &&
               tracker.current_map_id() == map::MapId{2};
    }));
    EXPECT_EQ(mapper.state(), mapping::MapperState::Mapping);
    EXPECT_EQ(mapper.current_map_id(), map::MapId{2});

    // Tracking on map 2: every remaining frame.
    const size_t before = recorder.position_updated.size();
    for (size_t i = kMap2End + 1; i < sequence.frames.size(); ++i) {
        tracker.Track(frame(i));
    }
    const size_t expected = before + sequence.frames.size() - kMap2End - 1;
    ASSERT_TRUE(recorder.WaitFor([&] { return recorder.position_updated.size() >= expected; }));
    EXPECT_EQ(recorder.position_updated.back(), 2u);
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
    EXPECT_EQ(recorder.map_points_added, std::vector<uint32_t>{1});
    EXPECT_EQ(recorder.map_points_added_before_map_created, std::vector<bool>{true});
    EXPECT_GE(mapper.num_map_points(), 100u);
    ASSERT_TRUE(recorder.WaitFor([&] {
        return tracker.state() == tracking::TrackerState::Tracking &&
               tracker.current_map_id() == map::MapId{1};
    }));
    EXPECT_EQ(mapper.state(), mapping::MapperState::Mapping);
    EXPECT_EQ(mapper.current_map_id(), map::MapId{1});

    // Tracking on map 1.
    tracker.Track(frame(1));
    ASSERT_TRUE(recorder.WaitFor([&] { return !recorder.position_updated.empty(); }));
    for (uint32_t map : recorder.position_updated) {
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
    EXPECT_EQ(recorder.map_points_added, (std::vector<uint32_t>{1, 2}));
    EXPECT_EQ(recorder.map_points_added_before_map_created, (std::vector<bool>{true, true}));
    EXPECT_GE(mapper.num_map_points(), 100u);
    ASSERT_TRUE(recorder.WaitFor([&] {
        return tracker.state() == tracking::TrackerState::Tracking &&
               tracker.current_map_id() == map::MapId{2};
    }));
    EXPECT_EQ(mapper.state(), mapping::MapperState::Mapping);
    EXPECT_EQ(mapper.current_map_id(), map::MapId{2});

    tracker.Track(frame(4));
    ASSERT_TRUE(recorder.WaitFor([&] { return recorder.position_updated.back() == 2u; }));
    EXPECT_EQ(recorder.tracking_lost, std::vector<uint32_t>{1});

    EXPECT_EQ(ExpectPositionsNearTruth(recorder, 1, truth, /*monocular=*/false), 1u);
    EXPECT_EQ(ExpectPositionsNearTruth(recorder, 2, truth, /*monocular=*/false), 1u);
}

} // namespace
} // namespace voyahslam::slam
