#include <voyahslam/events/async_observer.h>
#include <voyahslam/events/events.h>
#include <voyahslam/events/observable.h>
#include <voyahslam/events/wake_signal.h>
#include <voyahslam/features/monocular_measurement.h>
#include <voyahslam/features/orb_feature_extractor.h>
#include <voyahslam/memory/object_pool.h>

#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <thread>

namespace voyahslam::events {
namespace {

using OrbMapTransaction = MapTransaction<features::OrbFeatureExtractor>;

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

TEST(AsyncObserverTest, PushNotifiesAttachedSignal) {
    memory::ObjectPool<OrbMapTransaction, memory::ThreadSafePolicy> pool(2);
    WakeSignal s;
    AsyncObserver observer;
    observer.SetWakeSignal(&s);

    auto* event = pool.Create();
    uint32_t before = s.Snapshot();
    observer.Push(event);
    EXPECT_EQ(s.Snapshot(), before + 1);

    Listener listener = observer.GetListener();
    EXPECT_EQ(listener.GetNextEvent(), event);
    EXPECT_EQ(listener.GetNextEvent(), nullptr);
}

TEST(AsyncObserverTest, PushWithoutSignalStillEnqueues) {
    memory::ObjectPool<OrbMapTransaction, memory::ThreadSafePolicy> pool(2);
    AsyncObserver observer;
    auto* event = pool.Create();
    observer.Push(event);
    Listener listener = observer.GetListener();
    EXPECT_EQ(listener.GetNextEvent(), event);
    EXPECT_EQ(listener.GetNextEvent(), nullptr);
}

TEST(AsyncObserverTest, ObservableDeliversThroughPush) {
    memory::ObjectPool<OrbMapTransaction, memory::ThreadSafePolicy> pool(2);
    WakeSignal s;
    AsyncObserver observer;
    observer.SetWakeSignal(&s);
    Observable observable;
    observable.ConnectAsync(observer);

    auto* event = pool.Create();
    uint32_t before = s.Snapshot();
    observable.Notify(event);
    EXPECT_EQ(s.Snapshot(), before + 1);

    Listener listener = observer.GetListener();
    EXPECT_EQ(listener.GetNextEvent(), event);
}

} // namespace
} // namespace voyahslam::events
