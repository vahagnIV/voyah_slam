#include "probe_event.h"

#include <voyahslam/events/async_observer.h>
#include <voyahslam/events/observable.h>
#include <voyahslam/events/subscription.h>

#include <gtest/gtest.h>

#include <concepts>
#include <memory>
#include <thread>
#include <utility>
#include <vector>

namespace voyahslam::events {
namespace {

// A received event is immutable by type: assigning through an EventPtr does not
// compile. (Concepts, because an invalid requirement outside a template is a
// hard error, not a false.)
template <typename TPtr>
concept CanAssignPriority = requires(TPtr p) { p->priority = true; };
template <typename TPtr>
concept CanAssignMapId = requires(TPtr p) { p->map_id = map::MapId{1}; };
static_assert(!CanAssignPriority<EventPtr>);
static_assert(!CanAssignMapId<EventPtr>);
static_assert(std::is_same_v<EventPtr, std::shared_ptr<const Event>>);

// Notify takes an rvalue shared_ptr only: a producer cannot keep a mutable alias
// to a published event.
template <typename TObservable, typename TPtr>
concept CanNotify = requires(TObservable& o, TPtr&& p) { o.Notify(std::forward<TPtr>(p)); };
static_assert(CanNotify<Observable, std::shared_ptr<ProbeEvent>>);
static_assert(!CanNotify<Observable, std::shared_ptr<ProbeEvent>&>);
static_assert(!CanNotify<Observable, const std::shared_ptr<ProbeEvent>&>);
static_assert(!CanNotify<Observable, ProbeEvent*>);

// Events are not copyable-by-accident value types and cannot be destroyed through
// a base pointer.
static_assert(!std::is_destructible_v<Event>);

TEST(EventLifetimeTest, OutlivesProducerSubscriptionObserverAndListener) {
    ASSERT_EQ(ProbeEvent::alive.load(), 0);
    EventPtr held;
    {
        auto producer = std::make_unique<TestNode>();
        auto observer = std::make_unique<AsyncObserver>();
        auto listener = std::make_unique<Listener>(observer->GetListener());
        auto subscription = std::make_unique<Subscription>(producer->AddObserver(*observer));

        auto event = MakeProbe(7);
        event->map_id = map::MapId{3};
        producer->Publish(std::move(event));
        EXPECT_EQ(event, nullptr);  // Notify took the producer's handle
        held = listener->GetNextEvent();
        ASSERT_NE(held, nullptr);

        producer.reset();
        subscription.reset();
        observer.reset();
        listener.reset();
    }
    ASSERT_NE(held, nullptr);
    EXPECT_EQ(held->type, EventType::TrackingLost);
    EXPECT_EQ(held->map_id, map::MapId{3});
    EXPECT_EQ(static_cast<const ProbeEvent&>(*held).value, 7);
    EXPECT_EQ(ProbeEvent::alive.load(), 1);
    held.reset();
    EXPECT_EQ(ProbeEvent::alive.load(), 0);
}

TEST(EventLifetimeTest, LastDropFreesOnTheConsumerThread) {
    ASSERT_EQ(ProbeEvent::alive.load(), 0);
    std::thread::id destroyed_on;
    auto event = MakeProbe();
    event->destroyed_on = &destroyed_on;
    EventPtr published = std::move(event);

    std::thread::id consumer_id;
    std::thread consumer([&consumer_id, e = std::move(published)]() mutable {
        consumer_id = std::this_thread::get_id();
        e.reset();
    });
    consumer.join();
    EXPECT_EQ(destroyed_on, consumer_id);
    EXPECT_NE(destroyed_on, std::this_thread::get_id());
    EXPECT_EQ(ProbeEvent::alive.load(), 0);
}

TEST(EventLifetimeTest, NotifyWithoutObserversFreesAtOnce) {
    ASSERT_EQ(ProbeEvent::alive.load(), 0);
    Observable observable;
    auto event = MakeProbe();
    EXPECT_EQ(ProbeEvent::alive.load(), 1);
    observable.Notify(std::move(event));
    EXPECT_EQ(ProbeEvent::alive.load(), 0);
}

TEST(EventLifetimeTest, ObserverDestroyedWithQueuedEventsReleasesThem) {
    ASSERT_EQ(ProbeEvent::alive.load(), 0);
    TestNode producer;
    {
        AsyncObserver observer;
        auto subscription = producer.AddObserver(observer);
        for (int i = 0; i < 100; ++i) {
            producer.Publish(MakeProbe(i, i % 10 == 0));
        }
        EXPECT_EQ(ProbeEvent::alive.load(), 100);
        EXPECT_EQ(observer.size_approx(), 100u);
        EXPECT_EQ(observer.pending_priority(), 10u);
    }
    EXPECT_EQ(ProbeEvent::alive.load(), 0);
}

}  // namespace
}  // namespace voyahslam::events
