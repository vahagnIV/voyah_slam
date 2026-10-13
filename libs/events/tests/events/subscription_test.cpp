#include "probe_event.h"

#include <voyahslam/errors.h>
#include <voyahslam/events/async_observer.h>
#include <voyahslam/events/observable.h>
#include <voyahslam/events/subscription.h>
#include <voyahslam/logging/logger.h>

#include <spdlog/sinks/ringbuffer_sink.h>

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <memory>
#include <numeric>
#include <stdexcept>
#include <string>
#include <thread>
#include <type_traits>
#include <vector>

namespace voyahslam::events {
namespace {

using Order = std::array<int, 4>;

enum Part : int { kProducer = 0, kSubscription = 1, kObserver = 2, kListener = 3 };

std::vector<Order> AllPermutations() {
    Order order{0, 1, 2, 3};
    std::vector<Order> all;
    do {
        all.push_back(order);
    } while (std::next_permutation(order.begin(), order.end()));
    return all;
}

std::string OrderName(const ::testing::TestParamInfo<Order>& info) {
    static constexpr const char* kNames[] = {"Producer", "Subscription", "Observer", "Listener"};
    std::string name;
    for (size_t i = 0; i < info.param.size(); ++i) {
        name += (i == 0 ? "" : "_");
        name += kNames[info.param[i]];
    }
    return name;
}

// Every order in which {producer, Subscription, AsyncObserver, Listener} can be
// destroyed is memory-safe, and every event is freed exactly once (D-22). Run
// under ASan and TSan: a use-after-free or a double free is the failure.
class DestructionOrderTest : public ::testing::TestWithParam<Order> {};

TEST_P(DestructionOrderTest, AnyOrderIsSafe) {
    ASSERT_EQ(ProbeEvent::alive.load(), 0);
    auto producer = std::make_unique<TestNode>();  // never started: not a death-test case
    auto observer = std::make_unique<AsyncObserver>("DestructionOrder");
    auto listener = std::make_unique<Listener>(observer->GetListener());
    auto subscription = std::make_unique<Subscription>(producer->AddObserver(*observer));

    producer->Publish(MakeProbe(1));
    EventPtr held = listener->GetNextEvent();  // the consumer keeps this one past everything
    ASSERT_NE(held, nullptr);
    producer->Publish(MakeProbe(2));  // still queued in the observer

    for (int part : GetParam()) {
        switch (part) {
            case kProducer: producer.reset(); break;
            case kSubscription: subscription.reset(); break;
            case kObserver: observer.reset(); break;
            case kListener: listener.reset(); break;
        }
        // Whatever is still alive keeps working: a later event is delivered or
        // dropped, never dereferences a dead object.
        if (producer) {
            producer->Publish(MakeProbe(3));
        }
        if (listener && !observer) {
            EXPECT_EQ(listener->GetNextEvent(), nullptr);  // the closed inbox is empty
        }
    }

    ASSERT_NE(held, nullptr);
    EXPECT_EQ(held->type, EventType::TrackingLost);
    EXPECT_EQ(static_cast<const ProbeEvent&>(*held).value, 1);
    EXPECT_GE(ProbeEvent::alive.load(), 1);
    held.reset();
    EXPECT_EQ(ProbeEvent::alive.load(), 0);
}

INSTANTIATE_TEST_SUITE_P(AllOrders, DestructionOrderTest, ::testing::ValuesIn(AllPermutations()),
                         OrderName);

TEST(SubscriptionTest, ObserverDestroyedWhileProducerRunsIsSafe) {
    ASSERT_EQ(ProbeEvent::alive.load(), 0);
    TestNode node;
    auto observer = std::make_unique<AsyncObserver>();
    auto listener = observer->GetListener();
    auto subscription = node.AddObserver(*observer);
    node.Start();
    node.Publish(MakeProbe(1));
    EXPECT_EQ(ProbeEvent::alive.load(), 1);

    observer.reset();  // the consumer goes first: queued events are released
    EXPECT_EQ(ProbeEvent::alive.load(), 0);
    for (int i = 0; i < 3; ++i) {
        node.Publish(MakeProbe(i));
        EXPECT_EQ(ProbeEvent::alive.load(), 0);  // dropped at once by the closed inbox
    }
    EXPECT_EQ(listener.GetNextEvent(), nullptr);

    node.Stop();
    EXPECT_NO_THROW(subscription.reset());  // after Stop: disconnects, no terminate
    EXPECT_EQ(node.async_observer_count(), 0u);
}

TEST(SubscriptionTest, AddObserverWhileRunningThrows) {
    TestNode node;
    AsyncObserver observer;
    node.Start();
    EXPECT_THROW(static_cast<void>(node.AddObserver(observer)), UsageError);
    EXPECT_EQ(node.async_observer_count(), 0u);
    node.Stop();
    auto subscription = node.AddObserver(observer);  // allowed again
    EXPECT_TRUE(subscription.connected());
    EXPECT_EQ(node.async_observer_count(), 1u);
}

TEST(SubscriptionTest, AddSyncObserverWhileRunningThrows) {
    TestNode node;
    int calls = 0;
    const auto handler = [](void* ctx, const Event&) { ++*static_cast<int*>(ctx); };
    node.Start();
    EXPECT_THROW(static_cast<void>(node.AddSyncObserver(handler, &calls)), UsageError);
    node.Stop();
    auto subscription = node.AddSyncObserver(handler, &calls);
    node.Publish(MakeProbe());
    EXPECT_EQ(calls, 1);
}

TEST(SubscriptionTest, ResetWhileRunningThrows) {
    TestNode node;
    AsyncObserver observer;
    auto subscription = node.AddObserver(observer);
    node.Start();
    EXPECT_THROW(subscription.reset(), UsageError);
    EXPECT_TRUE(subscription.connected());  // the failed reset changed nothing
    EXPECT_EQ(node.async_observer_count(), 1u);

    node.Stop();
    EXPECT_NO_THROW(subscription.reset());
    EXPECT_FALSE(subscription.connected());
    EXPECT_EQ(node.async_observer_count(), 0u);
}

TEST(SubscriptionTest, MoveAssignWhileRunningThrows) {
    TestNode node;
    AsyncObserver observer1;
    AsyncObserver observer2;
    auto first = node.AddObserver(observer1);
    auto second = node.AddObserver(observer2);
    node.Start();
    EXPECT_THROW(first = std::move(second), UsageError);
    node.Stop();
    first = std::move(second);  // stopped: disconnects the first connection
    EXPECT_EQ(node.async_observer_count(), 1u);
}

TEST(SubscriptionTest, OutlivesProducer) {
    Subscription subscription;
    {
        TestNode node;
        AsyncObserver observer;
        subscription = node.AddObserver(observer);
        EXPECT_TRUE(subscription.connected());
    }
    EXPECT_TRUE(subscription.connected());  // the wiring block lives on in the subscription
    EXPECT_NO_THROW(subscription.reset());
    EXPECT_FALSE(subscription.connected());
}

TEST(SubscriptionTest, OutlivesProducerThatWasStartedAndStopped) {
    auto node = std::make_unique<TestNode>();
    AsyncObserver observer;
    auto subscription = node->AddObserver(observer);
    node->Start();
    node->Stop();
    node.reset();
    // Destroyed here, at the end of the scope, with the producer gone: no terminate.
}

TEST(SubscriptionTest, MovedFromIsInert) {
    TestNode node;
    AsyncObserver observer;
    auto first = node.AddObserver(observer);
    Subscription second = std::move(first);
    EXPECT_FALSE(first.connected());
    EXPECT_TRUE(second.connected());
    EXPECT_NO_THROW(first.reset());  // no-op: does not disconnect `second`
    EXPECT_EQ(node.async_observer_count(), 1u);

    Subscription third;
    third = std::move(second);
    EXPECT_FALSE(second.connected());
    EXPECT_TRUE(third.connected());
    EXPECT_EQ(node.async_observer_count(), 1u);
}

TEST(SubscriptionTest, MoveAssignDisconnectsTheCurrentConnection) {
    TestNode node;
    AsyncObserver observer1;
    AsyncObserver observer2;
    auto first = node.AddObserver(observer1);
    auto second = node.AddObserver(observer2);
    EXPECT_EQ(node.async_observer_count(), 2u);
    first = std::move(second);
    EXPECT_EQ(node.async_observer_count(), 1u);

    node.Publish(MakeProbe());
    EXPECT_EQ(observer1.size_approx(), 0u);  // the first connection is gone
    EXPECT_EQ(observer2.size_approx(), 1u);
}

TEST(SubscriptionTest, ResetTwiceIsNoOp) {
    TestNode node;
    AsyncObserver observer;
    auto subscription = node.AddObserver(observer);
    subscription.reset();
    EXPECT_NO_THROW(subscription.reset());
    EXPECT_EQ(node.async_observer_count(), 0u);
    Subscription never_connected;
    EXPECT_NO_THROW(never_connected.reset());
}

TEST(SubscriptionTest, DestroyedSubscriptionDisconnects) {
    TestNode node;
    AsyncObserver observer;
    {
        auto subscription = node.AddObserver(observer);
        EXPECT_EQ(node.async_observer_count(), 1u);
    }
    EXPECT_EQ(node.async_observer_count(), 0u);
    node.Publish(MakeProbe());
    EXPECT_EQ(observer.size_approx(), 0u);
}

TEST(SubscriptionTest, UsageErrorIsLogicError) {
    static_assert(std::is_base_of_v<std::logic_error, UsageError>);
    try {
        throw UsageError("misuse");
    } catch (const std::logic_error& error) {
        EXPECT_STREQ(error.what(), "misuse");
    }
}

// An inbox can be fed by several producers (the viewer gets Tracker and Mapper
// events) and queued events carry no producer tag, so disconnecting one producer
// never drains the inbox (D-08).
TEST(SubscriptionTest, DisconnectKeepsOtherProducersQueuedEvents) {
    ASSERT_EQ(ProbeEvent::alive.load(), 0);
    TestNode tracker;
    TestNode mapper;
    AsyncObserver inbox("Viewer");
    auto from_tracker = tracker.AddObserver(inbox);
    auto from_mapper = mapper.AddObserver(inbox);

    tracker.Publish(MakeProbe(10));
    mapper.Publish(MakeProbe(20));
    from_tracker.reset();

    EXPECT_EQ(inbox.size_approx(), 2u);
    int sum = 0;
    EventPtr event;
    while (inbox.TryPop(event)) {
        sum += static_cast<const ProbeEvent&>(*event).value;
    }
    EXPECT_EQ(sum, 30);
    event.reset();
    EXPECT_EQ(ProbeEvent::alive.load(), 0);

    tracker.Publish(MakeProbe(1));  // disconnected: not delivered
    mapper.Publish(MakeProbe(2));   // still connected
    EXPECT_EQ(inbox.size_approx(), 1u);
}

TEST(AsyncObserverTest, DestructorReleasesQueuedEvents) {
    ASSERT_EQ(ProbeEvent::alive.load(), 0);
    TestNode node;
    auto observer = std::make_unique<AsyncObserver>();
    auto listener = observer->GetListener();
    auto subscription = node.AddObserver(*observer);
    node.Publish(MakeProbe(1, true));
    node.Publish(MakeProbe(2, false));
    node.Publish(MakeProbe(3, true));
    EXPECT_EQ(ProbeEvent::alive.load(), 3);
    EXPECT_EQ(observer->pending_priority(), 2u);

    observer.reset();
    EXPECT_EQ(ProbeEvent::alive.load(), 0);
    EXPECT_EQ(listener.GetNextEvent(), nullptr);  // a surviving Listener sees an empty inbox
    node.Publish(MakeProbe(4));                   // and a later push is dropped at once
    EXPECT_EQ(ProbeEvent::alive.load(), 0);
}

// Without pools a stalled observer grows memory silently, so the inbox logs one
// "consumer is behind" warning when its queue reaches 1024 events and again at
// each doubling, never once per event (D-24).
TEST(AsyncObserverTest, BacklogWarningOncePerDoubling) {
    ASSERT_EQ(ProbeEvent::alive.load(), 0);
    auto sink = std::make_shared<spdlog::sinks::ringbuffer_sink_mt>(64);
    auto logger = Logger::CreateLogger("AsyncObserver");
    logger->sinks().push_back(sink);
    {
        Observable observable;
        AsyncObserver observer("StalledViewer");
        auto subscription = observable.ConnectAsync(observer);
        for (int i = 0; i < 1023; ++i) {
            observable.Notify(MakeProbe(i));
        }
        EXPECT_TRUE(sink->last_formatted(64).empty());  // below the floor
        for (int i = 1023; i < 5000; ++i) {
            observable.Notify(MakeProbe(i));
        }
        auto lines = sink->last_raw(64);
        // 1024 -> 2048 -> 4096: three warnings for 5000 events.
        ASSERT_EQ(lines.size(), 3u);
        for (const auto& message : lines) {
            EXPECT_EQ(message.level, spdlog::level::warn);
            EXPECT_NE(std::string(message.payload.begin(), message.payload.end()).find("StalledViewer"),
                      std::string::npos);
        }
    }
    logger->sinks().pop_back();
    EXPECT_EQ(ProbeEvent::alive.load(), 0);
}

// A queue that stays below the floor never warns, however many events pass through.
TEST(AsyncObserverTest, ADrainedInboxNeverWarns) {
    auto sink = std::make_shared<spdlog::sinks::ringbuffer_sink_mt>(64);
    auto logger = Logger::CreateLogger("AsyncObserver");
    logger->sinks().push_back(sink);
    {
        Observable observable;
        AsyncObserver observer("BusyButKeepingUp");
        auto subscription = observable.ConnectAsync(observer);
        EventPtr event;
        for (int i = 0; i < 5000; ++i) {
            observable.Notify(MakeProbe(i));
            ASSERT_TRUE(observer.TryPop(event));
        }
        EXPECT_TRUE(sink->last_formatted(64).empty());
    }
    logger->sinks().pop_back();
}

// A Subscription destroyed while its producer runs would leave a connection a
// running thread may still read: it terminates in every build, with the reason
// on stderr (D-07).
TEST(SubscriptionDeathTest, DestroyedWhileProducerRunsTerminates) {
    GTEST_FLAG_SET(death_test_style, "threadsafe");
    EXPECT_DEATH(
        {
            TestNode node;
            AsyncObserver observer;
            auto subscription = std::make_unique<Subscription>(node.AddObserver(observer));
            node.Start();
            subscription.reset();
        },
        "Subscription destroyed while its producer runs");
}

}  // namespace
}  // namespace voyahslam::events
