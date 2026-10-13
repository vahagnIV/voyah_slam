#include "probe_event.h"

#include <voyahslam/events/async_observer.h>
#include <voyahslam/events/observable.h>
#include <voyahslam/events/subscription.h>
#include <voyahslam/events/wake_signal.h>

#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <thread>

namespace voyahslam::events {
namespace {

TEST(WakeSignalTest, WaitReturnsImmediatelyWhenValueChanged) {
    WakeSignal s;
    uint32_t seen = s.Snapshot();
    s.Notify();
    s.Wait(seen);
    EXPECT_NE(s.Snapshot(), seen);
}

TEST(WakeSignalTest, WaitBlocksUntilNotify) {
    WakeSignal s;
    std::atomic<bool> woke{false};
    uint32_t seen = s.Snapshot();
    std::thread t([&] {
        s.Wait(seen);
        woke = true;
    });
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    EXPECT_FALSE(woke.load());
    s.Notify();
    t.join();
    EXPECT_TRUE(woke.load());
}

// The inbox owns the signal: every push to it notifies observer.wake_signal().
TEST(AsyncObserverTest, PushNotifiesTheObserversSignal) {
    AsyncObserver observer;
    Observable observable;
    auto subscription = observable.ConnectAsync(observer);

    uint32_t before = observer.wake_signal().Snapshot();
    observable.Notify(MakeProbe());
    EXPECT_EQ(observer.wake_signal().Snapshot(), before + 1);

    Listener listener = observer.GetListener();
    EXPECT_NE(listener.GetNextEvent(), nullptr);
    EXPECT_EQ(listener.GetNextEvent(), nullptr);
}

TEST(AsyncObserverTest, PushNobodyWaitsOnStillEnqueues) {
    AsyncObserver observer;
    Observable observable;
    auto subscription = observable.ConnectAsync(observer);

    auto event = MakeProbe(5);
    const EventPtr watch = event;
    observable.Notify(std::move(event));

    Listener listener = observer.GetListener();
    EXPECT_EQ(listener.GetNextEvent(), watch);
    EXPECT_EQ(listener.GetNextEvent(), nullptr);
}

TEST(AsyncObserverTest, ObservableDeliversThroughPush) {
    AsyncObserver observer;
    Observable observable;
    auto subscription = observable.ConnectAsync(observer);

    auto event = MakeProbe();
    const EventPtr watch = event;
    uint32_t before = observer.wake_signal().Snapshot();
    observable.Notify(std::move(event));
    EXPECT_EQ(observer.wake_signal().Snapshot(), before + 1);

    Listener listener = observer.GetListener();
    EXPECT_EQ(listener.GetNextEvent(), watch);
}

} // namespace
} // namespace voyahslam::events
