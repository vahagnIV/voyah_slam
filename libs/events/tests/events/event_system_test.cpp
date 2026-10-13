#include "probe_event.h"

#include <voyahslam/events/async_observer.h>
#include <voyahslam/events/events.h>
#include <voyahslam/events/observable.h>
#include <voyahslam/events/subscription.h>
#include <voyahslam/features/binocular_measurement.h>
#include <voyahslam/features/measurement.h>
#include <voyahslam/features/monocular_measurement.h>
#include <voyahslam/features/orb_feature_extractor.h>

#include <gtest/gtest.h>

#include <atomic>
#include <memory>
#include <variant>
#include <thread>
#include <vector>

namespace voyahslam::events {
namespace {

using OrbMonocularMeasurement = features::MonocularMeasurement<uint8_t, 32>;
using OrbBinocularMeasurement = features::BinocularMeasurement<uint8_t, 32>;
using OrbMapTransaction = MapTransaction<features::OrbFeatureExtractor>;
using OrbFrameTracked = FrameTracked<features::OrbFeatureExtractor>;
using OrbInitializationMeasurement = InitializationMeasurement<features::OrbFeatureExtractor>;

TEST(EventSystemTest, EventTypeTag) {
    const auto event = std::make_shared<OrbMapTransaction>();

    EXPECT_EQ(event->type, EventType::MapTransaction);
}

TEST(EventSystemTest, ObservableSyncHandler) {
    Observable observable;
    int counter = 0;

    auto subscription = observable.ConnectSync(
        [](void* ctx, const Event&) { ++*static_cast<int*>(ctx); },
        &counter);

    // A sync-only event is a make_shared object like any other.
    struct TestEvent : Event {
        TestEvent() : Event(EventType::FrameTracked) {}
    };
    observable.Notify(std::make_shared<TestEvent>());
    EXPECT_EQ(counter, 1);
    observable.Notify(std::make_shared<TestEvent>());
    EXPECT_EQ(counter, 2);
}

// One event, two observers: both inboxes hold the very same object, and it is
// freed when the last of them lets go.
TEST(EventSystemTest, ObservableAsyncFanOut) {
    Observable observable;
    AsyncObserver observer1;
    AsyncObserver observer2;

    auto subscription1 = observable.ConnectAsync(observer1);
    auto subscription2 = observable.ConnectAsync(observer2);

    auto event = std::make_shared<OrbMapTransaction>();
    const std::weak_ptr<const Event> watch = event;
    observable.Notify(std::move(event));

    EventPtr got1;
    {
        auto listener1 = observer1.GetListener();
        got1 = listener1.GetNextEvent();
        ASSERT_NE(got1, nullptr);
        EXPECT_EQ(got1->type, EventType::MapTransaction);
        EXPECT_EQ(listener1.GetNextEvent(), nullptr);
    }
    EventPtr got2;
    {
        auto listener2 = observer2.GetListener();
        got2 = listener2.GetNextEvent();
        ASSERT_NE(got2, nullptr);
        EXPECT_EQ(got2->type, EventType::MapTransaction);
    }
    EXPECT_EQ(got1, got2);

    got1.reset();
    EXPECT_FALSE(watch.expired());
    got2.reset();
    EXPECT_TRUE(watch.expired());
}

TEST(EventSystemTest, MapTransactionResizesPointColumnsTogether) {
    const auto e = std::make_shared<OrbMapTransaction>();
    e->ResizeNewPoints(7);
    EXPECT_EQ(e->num_new_points(), 7u);
    EXPECT_EQ(e->new_positions.cols(), 7);
    EXPECT_EQ(e->new_descriptors.cols(), 7);
    EXPECT_EQ(e->new_normals.cols(), 7);
    EXPECT_EQ(e->new_descriptors.matrix().rows(), features::OrbFeatureExtractor::DescriptorSize);
    e->ResizeUpdatedPoints(3);
    EXPECT_EQ(e->updated_ids.size(), 3u);
    EXPECT_EQ(e->updated_positions.cols(), 3);
    EXPECT_EQ(e->updated_descriptors.cols(), 3);
    EXPECT_EQ(e->updated_normals.cols(), 3);
}

// 50 000 events fan out to two observers drained on two threads while the
// producer publishes: every consumer sees every event in order, and every event
// is freed exactly once.
TEST(EventSystemTest, CrossThreadFanOutFreesEveryEventOnce) {
    ASSERT_EQ(ProbeEvent::alive.load(), 0);
    constexpr int kEvents = 50'000;
    Observable observable;
    AsyncObserver observer1;
    AsyncObserver observer2;
    auto subscription1 = observable.ConnectAsync(observer1);
    auto subscription2 = observable.ConnectAsync(observer2);

    std::atomic<bool> done{false};
    auto consume = [&done](AsyncObserver& observer, uint64_t& out_sum, int& out_count) {
        auto listener = observer.GetListener();
        while (true) {
            EventPtr e = listener.GetNextEvent();
            if (!e) {
                if (!done.load(std::memory_order_acquire)) {
                    std::this_thread::yield();
                    continue;
                }
                e = listener.GetNextEvent();  // anything pushed before `done`
                if (!e) break;
            }
            out_sum += static_cast<uint64_t>(static_cast<const ProbeEvent&>(*e).value);
            ++out_count;
        }
    };
    uint64_t sum1 = 0;
    uint64_t sum2 = 0;
    int count1 = 0;
    int count2 = 0;
    std::thread consumer1(consume, std::ref(observer1), std::ref(sum1), std::ref(count1));
    std::thread consumer2(consume, std::ref(observer2), std::ref(sum2), std::ref(count2));

    uint64_t expected = 0;
    for (int i = 0; i < kEvents; ++i) {
        expected += static_cast<uint64_t>(i);
        observable.Notify(MakeProbe(i));
    }
    done.store(true, std::memory_order_release);
    consumer1.join();
    consumer2.join();

    EXPECT_EQ(count1, kEvents);
    EXPECT_EQ(count2, kEvents);
    EXPECT_EQ(sum1, expected);
    EXPECT_EQ(sum2, expected);
    EXPECT_EQ(ProbeEvent::alive.load(), 0);
}

TEST(EventSystemTest, ObservableSyncAndAsync) {
    Observable observable;
    int sync_count = 0;
    AsyncObserver async_obs;

    auto sync_subscription = observable.ConnectSync(
        [](void* ctx, const Event&) { ++*static_cast<int*>(ctx); },
        &sync_count);
    auto async_subscription = observable.ConnectAsync(async_obs);

    observable.Notify(std::make_shared<OrbMapTransaction>());

    EXPECT_EQ(sync_count, 1);

    auto listener = async_obs.GetListener();
    const EventPtr got = listener.GetNextEvent();
    ASSERT_NE(got, nullptr);
    EXPECT_EQ(got->type, EventType::MapTransaction);
}

// An event does not pin the measurement it carries longer than its own life: the
// last EventPtr dropped releases it.
TEST(EventSystemTest, LastDropReleasesTheMeasurement) {
    using TMeasurement = features::TMeasurement<features::OrbFeatureExtractor>;
    const auto measurement = std::make_shared<const TMeasurement>(
        features::TMonocularMeasurement<features::OrbFeatureExtractor>{});

    auto tracked = std::make_shared<OrbFrameTracked>();
    tracked->measurement = measurement;
    auto transaction = std::make_shared<OrbMapTransaction>();
    transaction->added_frames.emplace_back().measurement = measurement;
    EXPECT_EQ(measurement.use_count(), 3);

    Observable observable;
    AsyncObserver observer;
    auto subscription = observable.ConnectAsync(observer);
    observable.Notify(std::move(tracked));
    observable.Notify(std::move(transaction));
    EXPECT_EQ(measurement.use_count(), 3);  // held by the two queued events

    auto listener = observer.GetListener();
    EventPtr first = listener.GetNextEvent();
    EventPtr second = listener.GetNextEvent();
    ASSERT_NE(first, nullptr);
    ASSERT_NE(second, nullptr);
    EXPECT_EQ(measurement.use_count(), 3);
    first.reset();
    EXPECT_EQ(measurement.use_count(), 2);
    second.reset();
    EXPECT_EQ(measurement.use_count(), 1);
}

TEST(EventSystemTest, LoopDetectedTypeTag) {
    const auto event = std::make_shared<LoopDetected>();
    EXPECT_EQ(event->type, EventType::LoopDetected);
}

TEST(EventSystemTest, FrameTrackedTypeTag) {
    const auto event = std::make_shared<OrbFrameTracked>();
    EXPECT_EQ(event->type, EventType::FrameTracked);
}

TEST(EventSystemTest, NewEventTypeTags) {
    const auto created = std::make_shared<OrbMapTransaction>();
    const auto lost = std::make_shared<TrackingLost>();
    const auto init = std::make_shared<OrbInitializationMeasurement>();
    const auto finished = std::make_shared<GlobalBAFinished>();

    EXPECT_EQ(created->type, EventType::MapTransaction);
    EXPECT_EQ(lost->type, EventType::TrackingLost);
    EXPECT_EQ(init->type, EventType::InitializationMeasurement);
    EXPECT_EQ(finished->type, EventType::GlobalBAFinished);
    EXPECT_EQ(created->map_id, map::MapId{0});
    EXPECT_EQ(lost->map_id, map::MapId{0});
    EXPECT_EQ(init->map_id, map::MapId{0});
    EXPECT_FALSE(created->priority);
    EXPECT_TRUE(lost->final);  // a TrackingLost ends the map unless it says otherwise
}

TEST(EventSystemTest, NewFrameHoldsMonoOrStereo) {
    const auto event = std::make_shared<OrbMapTransaction>();
    using TMeasurement = features::TMeasurement<features::OrbFeatureExtractor>;
    event->added_frames.emplace_back().measurement = std::make_shared<const TMeasurement>(
        features::TMonocularMeasurement<features::OrbFeatureExtractor>{});
    EXPECT_TRUE(std::holds_alternative<features::TMonocularMeasurement<features::OrbFeatureExtractor>>(
        *event->added_frames[0].measurement));
    event->added_frames.emplace_back().measurement = std::make_shared<const TMeasurement>(
        features::TBinocularMeasurement<features::OrbFeatureExtractor>{});
    EXPECT_TRUE(std::holds_alternative<features::TBinocularMeasurement<features::OrbFeatureExtractor>>(
        *event->added_frames[1].measurement));
}

} // namespace
} // namespace voyahslam::events

namespace voyahslam::events {
namespace {

// The priority count goes up when a priority event is pushed (before the
// enqueue) and down when it is popped, whichever way the consumer pops.
TEST(AsyncObserverTest, PriorityCountTracksPushPop) {
    Observable observable;
    AsyncObserver observer;
    auto subscription = observable.ConnectAsync(observer);
    auto listener = observer.GetListener();

    auto a = MakeProbe(1, true);
    auto b = MakeProbe(2, false);
    auto c = MakeProbe(3, true);
    const EventPtr watch_a = a;
    const EventPtr watch_b = b;
    const EventPtr watch_c = c;

    EXPECT_EQ(observer.pending_priority(), 0u);
    observable.Notify(std::move(a));
    EXPECT_EQ(observer.pending_priority(), 1u);
    observable.Notify(std::move(b));
    EXPECT_EQ(observer.pending_priority(), 1u);
    observable.Notify(std::move(c));
    EXPECT_EQ(observer.pending_priority(), 2u);
    EXPECT_EQ(observer.size_approx(), 3u);

    EXPECT_EQ(listener.GetNextEvent(), watch_a);
    EXPECT_EQ(observer.pending_priority(), 1u);
    EXPECT_EQ(listener.GetNextEvent(), watch_b);
    EXPECT_EQ(observer.pending_priority(), 1u);
    EventPtr popped;
    ASSERT_TRUE(observer.TryPop(popped));
    EXPECT_EQ(popped, watch_c);
    EXPECT_EQ(observer.pending_priority(), 0u);
    EXPECT_EQ(listener.GetNextEvent(), nullptr);
    EXPECT_FALSE(observer.TryPop(popped));
    EXPECT_EQ(observer.pending_priority(), 0u);
}

}  // namespace
}  // namespace voyahslam::events
