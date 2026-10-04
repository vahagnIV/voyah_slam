#include <support/synthetic_sequence.h>
#include <support/test_measurements.h>
#include <voyahslam/events/async_observer.h>
#include <voyahslam/events/events.h>
#include <voyahslam/features/orb_feature_extractor.h>
#include <voyahslam/features/sift_feature_extractor.h>
#include <voyahslam/slam/slam.h>

#include <gtest/gtest.h>

#include <chrono>
#include <cstdint>
#include <thread>
#include <vector>

namespace voyahslam::slam {
namespace {

using OrbSlam = Slam<features::OrbFeatureExtractor>;

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
            } else if (event->type == events::EventType::MapPointsAdded) {
                map_points_added.push_back(event->map_id.value);
                // Map points before the MapCreated that announces their map.
                map_points_added_before_map_created.push_back(
                    map_created.empty() || map_created.back() != event->map_id.value);
            }
        }
        while (events::Event* event = tracker_listener_.GetNextEvent()) {
            if (event->type == events::EventType::PositionUpdated) {
                position_updated.push_back(event->map_id.value);
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

TEST(SlamTest, EndToEndInitializeTrackLoseReinitialize) {
    // Map 1 from frames 0-14, tracking on frames 15-16, a featureless frame
    // loses tracking, map 2 from frames 17-29. Timestamps increase throughout.
    constexpr size_t kMap1End = 14;
    constexpr size_t kMap2Begin = kMap1End + 3;
    const test::SyntheticSequence sequence = test::MakeSyntheticSequence();
    ASSERT_EQ(sequence.frames.size(), 30u);
    const auto frame = [&](size_t i) { return sequence.frames[i].measurement; };

    // Lifetimes: the recorder outlives the Slam (nothing is published into a
    // destroyed queue), and `stop_and_drain` stops the Slam and releases every
    // queued event before the Slam's pools are destroyed, also when an
    // assertion returns early.
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
    ASSERT_TRUE(recorder.WaitFor([&] { return !recorder.position_updated.empty(); }));
    for (uint32_t map : recorder.position_updated) {
        EXPECT_EQ(map, 1u);
    }

    // Too few features: tracking is lost, the Mapper drops map 1.
    const double lost_timestamp =
        0.5 * (frame(kMap1End + 2).timestamp() + frame(kMap2Begin).timestamp());
    tracker.Track(test::MakeMonocular(0, lost_timestamp));
    ASSERT_TRUE(recorder.WaitFor([&] { return !recorder.tracking_lost.empty(); }));
    EXPECT_EQ(recorder.tracking_lost, std::vector<uint32_t>{1});
    EXPECT_EQ(tracker.state(), tracking::TrackerState::Lost);

    // Re-initialization from the later frames builds map 2.
    for (size_t i = kMap2Begin; i < sequence.frames.size(); ++i) {
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

    tracker.Track(test::MakeMonocular(200, frame(sequence.frames.size() - 1).timestamp() + 0.05));
    ASSERT_TRUE(recorder.WaitFor([&] { return recorder.position_updated.back() == 2u; }));
}

TEST(SlamTest, EndToEndStereoInitializeTrackLoseReinitialize) {
    // Map 1 from one synthetic stereo frame (the Tracker stereo-matches it),
    // tracking on a second one, a featureless frame loses tracking, map 2 from
    // another synthetic frame. Timestamps increase throughout.
    const auto stereo_frame = [](double timestamp, uint32_t seed) {
        test::SyntheticStereoOptions options;
        options.timestamp = timestamp;
        options.seed = seed;
        return test::MakeSyntheticStereoFrame(options).measurement;
    };

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
    tracker.Track(stereo_frame(0.0, 1));
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
    tracker.Track(stereo_frame(0.1, 2));
    ASSERT_TRUE(recorder.WaitFor([&] { return !recorder.position_updated.empty(); }));
    for (uint32_t map : recorder.position_updated) {
        EXPECT_EQ(map, 1u);
    }

    // Too few features: tracking is lost, the Mapper drops map 1.
    tracker.Track(test::MakeBinocular(0, 0.2));
    ASSERT_TRUE(recorder.WaitFor([&] { return !recorder.tracking_lost.empty(); }));
    EXPECT_EQ(recorder.tracking_lost, std::vector<uint32_t>{1});
    EXPECT_EQ(tracker.state(), tracking::TrackerState::Lost);

    // Re-initialization from a new stereo frame builds map 2.
    tracker.Track(stereo_frame(0.3, 3));
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

    tracker.Track(stereo_frame(0.4, 4));
    ASSERT_TRUE(recorder.WaitFor([&] { return recorder.position_updated.back() == 2u; }));
}

} // namespace
} // namespace voyahslam::slam
