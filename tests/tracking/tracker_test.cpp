#include <gtest/gtest.h>
#include <support/test_measurements.h>
#include <voyahslam/events/async_observer.h>
#include <voyahslam/events/events.h>
#include <voyahslam/events/observable.h>
#include <voyahslam/memory/object_pool.h>
#include <voyahslam/tracking/tracker.h>

#include <chrono>
#include <thread>
#include <variant>

namespace voyahslam::tracking {
namespace {

using Extractor = test::TestExtractor;
using TTracker = Tracker<Extractor>;
using TKeyFrameCreated = events::KeyFrameCreated<Extractor>;
using TInitializationMeasurement = events::InitializationMeasurement<Extractor>;
using Clock = std::chrono::steady_clock;

geometry::TPose NonIdentityPose() {
    geometry::TPose pose = geometry::TPose::Identity();
    pose(0, 3) = 1.5F;
    pose(2, 3) = -0.5F;
    return pose;
}

// Feeds the Tracker's inbox from `mapper_` (stands in for the Mapper) and
// captures what the Tracker publishes in `capture_`.
class TrackerTest : public ::testing::Test {
protected:
    TrackerTest() {
        mapper_.ConnectAsync(tracker_.inbox());
        tracker_.AddObserver(capture_);
    }

    void Drain() {
        while (tracker_.ProcessPending()) {
        }
    }

    void PublishKeyFrameCreated(uint32_t map, uint32_t frame) {
        auto* event = key_frame_pool_.Create();
        event->map_id = map::MapId{map};
        event->frame_id = map::FrameId{frame};
        event->pose = geometry::TPose::Identity();
        mapper_.Notify(event);
    }

    void PublishMapCreated(uint32_t map, uint32_t frame, const geometry::TPose& pose) {
        auto* event = map_created_pool_.Create();
        event->map_id = map::MapId{map};
        event->frame_id = map::FrameId{frame};
        event->pose = pose;
        mapper_.Notify(event);
    }

    // Brings the Tracker to Tracking on `map` from keyframe `frame` at `pose`.
    void EnterTracking(uint32_t map, uint32_t frame, const geometry::TPose& pose) {
        PublishKeyFrameCreated(map, frame);
        PublishMapCreated(map, frame, pose);
        Drain();
        ASSERT_EQ(tracker_.state(), TrackerState::Tracking);
        ASSERT_EQ(tracker_.current_map_id(), map::MapId{map});
    }

    // The next captured event (valid until the next call), or nullptr.
    events::Event* NextCaptured() { return listener_.GetNextEvent(); }

    memory::ObjectPool<TKeyFrameCreated, memory::ThreadSafePolicy> key_frame_pool_{256};
    memory::ObjectPool<events::MapCreated, memory::ThreadSafePolicy> map_created_pool_{256};
    events::Observable mapper_;
    TTracker tracker_;
    events::AsyncObserver capture_;
    events::Listener listener_ = capture_.GetListener();
};

TEST_F(TrackerTest, ForwardsMeasurementsWhileWaitingForMap) {
    tracker_.Track(test::MakeMonocular(200, 0.0));
    Drain();

    auto* event = NextCaptured();
    ASSERT_NE(event, nullptr);
    ASSERT_EQ(event->type, events::EventType::InitializationMeasurement);
    EXPECT_EQ(event->map_id, map::MapId{0});
    const auto& init = *static_cast<TInitializationMeasurement*>(event);
    ASSERT_TRUE(std::holds_alternative<TTracker::TMonocularMeasurement>(init.measurement));
    const auto& mono = std::get<TTracker::TMonocularMeasurement>(init.measurement);
    EXPECT_EQ(mono.points().cols(), 200);
    EXPECT_DOUBLE_EQ(mono.timestamp(), 0.0);
    EXPECT_EQ(NextCaptured(), nullptr);
    EXPECT_EQ(tracker_.state(), TrackerState::WaitingForMap);
}

TEST_F(TrackerTest, InitialMapEventsBeforeMapCreatedAreKept) {
    EXPECT_EQ(tracker_.state(), TrackerState::WaitingForMap);
    EXPECT_EQ(tracker_.current_map_id(), map::MapId{0});
    PublishKeyFrameCreated(1, 1);
    PublishMapCreated(1, 1, NonIdentityPose());
    Drain();

    EXPECT_EQ(tracker_.state(), TrackerState::Tracking);
    EXPECT_EQ(tracker_.current_map_id(), map::MapId{1});
}

TEST_F(TrackerTest, TracksAndPublishesPosition) {
    const geometry::TPose pose = NonIdentityPose();
    EnterTracking(1, 1, pose);

    tracker_.Track(test::MakeMonocular(200, 2.0));
    Drain();

    auto* event = NextCaptured();
    ASSERT_NE(event, nullptr);
    ASSERT_EQ(event->type, events::EventType::PositionUpdated);
    const auto& position = *static_cast<events::PositionUpdated*>(event);
    EXPECT_EQ(position.map_id, map::MapId{1});
    EXPECT_EQ(position.frame_id, map::FrameId{1});
    EXPECT_TRUE(position.pose.isApprox(pose));
    EXPECT_DOUBLE_EQ(position.timestamp, 2.0);
    EXPECT_EQ(NextCaptured(), nullptr);
    EXPECT_EQ(tracker_.state(), TrackerState::Tracking);
}

TEST_F(TrackerTest, LosesTrackingOnFewFeatures) {
    EnterTracking(1, 1, NonIdentityPose());

    tracker_.Track(test::MakeMonocular(0, 3.0));
    Drain();

    auto* event = NextCaptured();
    ASSERT_NE(event, nullptr);
    ASSERT_EQ(event->type, events::EventType::TrackingLost);
    const auto& lost = *static_cast<events::TrackingLost*>(event);
    EXPECT_EQ(lost.map_id, map::MapId{1});
    EXPECT_DOUBLE_EQ(lost.timestamp, 3.0);
    EXPECT_EQ(tracker_.state(), TrackerState::Lost);

    tracker_.Track(test::MakeMonocular(200, 4.0));
    Drain();

    event = NextCaptured();
    ASSERT_NE(event, nullptr);
    EXPECT_EQ(event->type, events::EventType::InitializationMeasurement);
    EXPECT_EQ(event->map_id, map::MapId{0});
    EXPECT_EQ(tracker_.state(), TrackerState::Lost);
}

TEST_F(TrackerTest, StateIsLostWhenTrackingLostIsPublished) {
    EnterTracking(1, 1, NonIdentityPose());
    // A synchronous observer runs on the publishing thread, inside Notify, so
    // it sees the state at the exact moment TrackingLost is published.
    struct Probe {
        TTracker* tracker;
        bool seen = false;
        TrackerState state = TrackerState::WaitingForMap;
    } probe{&tracker_};
    tracker_.AddSyncObserver(
        [](void* ctx, events::Event* event) {
            auto* p = static_cast<Probe*>(ctx);
            if (event->type == events::EventType::TrackingLost) {
                p->seen = true;
                p->state = p->tracker->state();
            }
        },
        &probe);

    tracker_.Track(test::MakeMonocular(0, 3.0));
    Drain();

    ASSERT_TRUE(probe.seen);
    EXPECT_EQ(probe.state, TrackerState::Lost);
}

TEST_F(TrackerTest, IgnoresStaleMapCreated) {
    EnterTracking(2, 5, NonIdentityPose());

    PublishMapCreated(1, 1, geometry::TPose::Identity());
    Drain();

    EXPECT_EQ(tracker_.current_map_id(), map::MapId{2});
    EXPECT_EQ(tracker_.state(), TrackerState::Tracking);

    // The position is still reported against map 2's keyframe and pose.
    tracker_.Track(test::MakeMonocular(200, 1.0));
    Drain();
    auto* event = NextCaptured();
    ASSERT_NE(event, nullptr);
    ASSERT_EQ(event->type, events::EventType::PositionUpdated);
    const auto& position = *static_cast<events::PositionUpdated*>(event);
    EXPECT_EQ(position.map_id, map::MapId{2});
    EXPECT_EQ(position.frame_id, map::FrameId{5});
    EXPECT_TRUE(position.pose.isApprox(NonIdentityPose()));
}

TEST_F(TrackerTest, NewMapAfterLoss) {
    EnterTracking(1, 1, geometry::TPose::Identity());
    tracker_.Track(test::MakeMonocular(0, 3.0));
    Drain();
    ASSERT_EQ(tracker_.state(), TrackerState::Lost);

    PublishKeyFrameCreated(2, 7);
    PublishMapCreated(2, 7, NonIdentityPose());
    Drain();

    EXPECT_EQ(tracker_.state(), TrackerState::Tracking);
    EXPECT_EQ(tracker_.current_map_id(), map::MapId{2});
}

TEST_F(TrackerTest, ThreadProcessesTrackedMeasurements) {
    tracker_.Start();
    for (int i = 0; i < 3; ++i) {
        tracker_.Track(test::MakeMonocular(200, static_cast<double>(i)));
    }

    int received = 0;
    const auto deadline = Clock::now() + std::chrono::seconds(1);
    while (received < 3 && Clock::now() < deadline) {
        if (auto* event = NextCaptured()) {
            EXPECT_EQ(event->type, events::EventType::InitializationMeasurement);
            ++received;
        } else {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    }
    tracker_.Stop();

    EXPECT_EQ(received, 3);
}

} // namespace
} // namespace voyahslam::tracking
