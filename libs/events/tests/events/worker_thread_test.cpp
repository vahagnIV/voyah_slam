#include <voyahslam/events/wake_signal.h>
#include <voyahslam/events/worker_thread.h>

#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <thread>

namespace voyahslam::events {
namespace {

using Clock = std::chrono::steady_clock;

template <typename TPredicate>
bool WaitFor(TPredicate predicate, std::chrono::milliseconds timeout = std::chrono::milliseconds(1000)) {
    const auto deadline = Clock::now() + timeout;
    while (!predicate()) {
        if (Clock::now() > deadline) return false;
        std::this_thread::yield();
    }
    return true;
}

TEST(WorkerThreadTest, RunsUntilNoWorkThenSleeps) {
    WakeSignal signal;
    WorkerThread worker(signal);
    std::atomic<int> work{5};
    std::atomic<int> calls{0};
    worker.Start([&] {
        ++calls;
        if (work.load() > 0) {
            --work;
            return true;
        }
        return false;
    });
    signal.Notify();
    ASSERT_TRUE(WaitFor([&] { return work.load() == 0; }));
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
    const int first = calls.load();
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    EXPECT_EQ(first, calls.load());
    worker.Stop();
}

TEST(WorkerThreadTest, WakesOnNotify) {
    WakeSignal signal;
    WorkerThread worker(signal);
    std::atomic<int> work{0};
    std::atomic<int> processed{0};
    worker.Start([&] {
        if (work.exchange(0) > 0) {
            ++processed;
            return true;
        }
        return false;
    });
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    work = 1;
    signal.Notify();
    EXPECT_TRUE(WaitFor([&] { return processed.load() == 1; }));
    worker.Stop();
}

TEST(WorkerThreadTest, NoLostWakeUp) {
    WakeSignal signal;
    WorkerThread worker(signal);
    std::atomic<int> work{0};
    std::atomic<int> processed{0};
    worker.Start([&] {
        if (work.exchange(0) > 0) {
            ++processed;
            return true;
        }
        return false;
    });
    for (int i = 1; i <= 1000; ++i) {
        work = 1;
        signal.Notify();
        ASSERT_TRUE(WaitFor([&] { return processed.load() == i; })) << "iteration " << i;
    }
    worker.Stop();
}

TEST(WorkerThreadTest, StopWakesSleepingWorker) {
    WakeSignal signal;
    WorkerThread worker(signal);
    worker.Start([] { return false; });
    EXPECT_TRUE(worker.running());
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    const auto begin = Clock::now();
    worker.Stop();
    EXPECT_LT(Clock::now() - begin, std::chrono::milliseconds(500));
    EXPECT_FALSE(worker.running());
}

TEST(WorkerThreadTest, StopTwiceAndWithoutStart) {
    WakeSignal signal;
    WorkerThread never_started(signal);
    never_started.Stop();
    EXPECT_FALSE(never_started.running());

    WorkerThread worker(signal);
    worker.Start([] { return false; });
    worker.Stop();
    worker.Stop();
    EXPECT_FALSE(worker.running());
}

TEST(WorkerThreadTest, RestartAfterStop) {
    WakeSignal signal;
    WorkerThread worker(signal);
    std::atomic<int> calls{0};
    auto callback = [&] {
        ++calls;
        return false;
    };
    worker.Start(callback);
    worker.Start(callback);  // no-op while running
    worker.Stop();
    const int before = calls.load();
    worker.Start(callback);
    EXPECT_TRUE(WaitFor([&] { return calls.load() > before; }));
    worker.Stop();
}

TEST(WorkerThreadTest, OnWorkerThreadIsTrueOnlyOnTheWorker) {
    WakeSignal signal;
    WorkerThread worker(signal);
    EXPECT_FALSE(worker.OnWorkerThread());  // never started
    std::atomic<int> on_worker{-1};         // -1 not yet, 0 false, 1 true
    worker.Start([&] {
        if (on_worker.load() < 0) on_worker.store(worker.OnWorkerThread() ? 1 : 0);
        return false;
    });
    ASSERT_TRUE(WaitFor([&] { return on_worker.load() >= 0; }));
    EXPECT_EQ(on_worker.load(), 1);
    EXPECT_FALSE(worker.OnWorkerThread());  // the test thread is not the worker
    std::thread other([&] { EXPECT_FALSE(worker.OnWorkerThread()); });
    other.join();
    worker.Stop();
    EXPECT_FALSE(worker.OnWorkerThread());
}

} // namespace
} // namespace voyahslam::events
