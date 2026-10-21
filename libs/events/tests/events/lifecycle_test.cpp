// The component lifecycle (D-09..D-12): Created -> Running -> Stopped through an
// atomic state machine in EventNode, safe from any thread, Stopped terminal.

#include "probe_event.h"

#include <voyahslam/errors.h>
#include <voyahslam/events/async_observer.h>
#include <voyahslam/events/event_node.h>
#include <voyahslam/events/worker_thread.h>

#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

namespace voyahslam::events {
namespace {

using Clock = std::chrono::steady_clock;

template <typename TPredicate>
bool WaitFor(TPredicate predicate, std::chrono::milliseconds timeout = std::chrono::seconds(5)) {
    const auto deadline = Clock::now() + timeout;
    while (!predicate()) {
        if (Clock::now() > deadline) return false;
        std::this_thread::yield();
    }
    return true;
}

// Runs `stop` on a helper thread and waits at most `limit` for it. A node whose Stop()
// deadlocked cannot be destroyed (its worker never returns), so when the deadline
// passes the process ends here with a DEADLOCK line instead of hanging the test run.
template <typename TStop>
void StopUnderWatchdog(TStop stop, std::chrono::seconds limit) {
    std::atomic<bool> done{false};
    std::thread stopper([&] {
        stop();
        done.store(true, std::memory_order_release);
    });
    const auto deadline = Clock::now() + limit;
    while (!done.load(std::memory_order_acquire)) {
        if (Clock::now() > deadline) {
            std::fprintf(stderr, "DEADLOCK: Stop() did not return within %lld s\n",
                         static_cast<long long>(limit.count()));
            std::fflush(stderr);
            std::_Exit(1);
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    stopper.join();
}

// A component shaped like the Tracker: Start()/Stop() through StartWith/StopWith
// around a WorkerThread that is the last member.
class LifecycleNode : public EventNode {
public:
    LifecycleNode() : EventNode("LifecycleNode") {}
    ~LifecycleNode() { Stop(); }

    void Start() {
        StartWith([this] {
            if (fail_start_) throw std::runtime_error("cannot start the worker");
            worker_.Start([this] { return ProcessPending(); });
            if (hold_start_for_worker_) {
                // Stay in Starting until the worker's refused Stop() was seen.
                caught_while_starting_.store(WaitFor([this] { return caught(); }, std::chrono::seconds(3)),
                                             std::memory_order_release);
            }
        });
    }
    void Stop() {
        StopWith(
            [this] {
                worker_.Stop();
                joined_.store(true, std::memory_order_release);
            },
            [this] { return worker_.OnWorkerThread(); });
    }

    bool ProcessPending() {
        calls_.fetch_add(1, std::memory_order_relaxed);
        if ((stop_from_worker_ || stop_when_stopping_) && !attempted_.exchange(true)) {
            if (stop_when_stopping_) {
                // Park until another thread is inside Stop(), then call Stop() ourselves.
                parked_.store(true, std::memory_order_release);
                WaitFor([this] { return run_state() == RunState::Stopping; }, std::chrono::seconds(10));
            }
            try {
                Stop();
            } catch (const UsageError& error) {
                worker_error_ = error.what();
                caught_.store(true, std::memory_order_release);
            }
        }
        return false;
    }

    [[nodiscard]] bool joined() const { return joined_.load(std::memory_order_acquire); }
    [[nodiscard]] bool caught() const { return caught_.load(std::memory_order_acquire); }
    [[nodiscard]] bool parked() const { return parked_.load(std::memory_order_acquire); }
    [[nodiscard]] bool caught_while_starting() const {
        return caught_while_starting_.load(std::memory_order_acquire);
    }
    [[nodiscard]] int calls() const { return calls_.load(std::memory_order_relaxed); }
    // Read after caught() is true.
    [[nodiscard]] const std::string& worker_error() const { return worker_error_; }

    bool fail_start_ = false;
    bool stop_from_worker_ = false;
    // The worker waits for run_state() == Stopping before it calls Stop().
    bool stop_when_stopping_ = false;
    // The start lambda stays in Starting until the worker's Stop() was refused.
    bool hold_start_for_worker_ = false;

private:
    std::atomic<bool> parked_{false};
    std::atomic<bool> caught_while_starting_{false};
    std::atomic<int> calls_{0};
    std::atomic<bool> joined_{false};
    std::atomic<bool> attempted_{false};
    std::atomic<bool> caught_{false};
    std::string worker_error_;
    WorkerThread worker_{wake_signal()};
};

// Releases waiting threads all at once.
class Gate {
public:
    void Open() { open_.store(true, std::memory_order_release); }
    void Wait() const {
        while (!open_.load(std::memory_order_acquire)) std::this_thread::yield();
    }

private:
    std::atomic<bool> open_{false};
};

TEST(LifecycleTest, StartMovesCreatedToRunning) {
    LifecycleNode node;
    EXPECT_EQ(node.run_state(), RunState::Created);
    node.Start();
    EXPECT_EQ(node.run_state(), RunState::Running);
    ASSERT_TRUE(WaitFor([&] { return node.calls() > 0; }));  // the worker runs
    node.Stop();
    EXPECT_EQ(node.run_state(), RunState::Stopped);
}

TEST(LifecycleTest, StartTwiceThrows) {
    LifecycleNode node;
    node.Start();
    EXPECT_THROW(node.Start(), UsageError);
    EXPECT_EQ(node.run_state(), RunState::Running);  // the first Start() is unaffected
}

TEST(LifecycleTest, StartAfterStopThrows) {
    LifecycleNode node;
    node.Start();
    node.Stop();
    EXPECT_THROW(node.Start(), UsageError);
    EXPECT_EQ(node.run_state(), RunState::Stopped);
}

TEST(LifecycleTest, StopIsIdempotent) {
    LifecycleNode node;
    node.Start();
    node.Stop();
    EXPECT_NO_THROW(node.Stop());
    EXPECT_NO_THROW(node.Stop());
    EXPECT_TRUE(node.joined());
    EXPECT_EQ(node.run_state(), RunState::Stopped);
}

TEST(LifecycleTest, StopBeforeStartIsTerminal) {
    LifecycleNode node;
    EXPECT_NO_THROW(node.Stop());
    EXPECT_EQ(node.run_state(), RunState::Stopped);
    EXPECT_THROW(node.Start(), UsageError);
    EXPECT_EQ(node.calls(), 0);  // the worker never ran
}

TEST(LifecycleTest, ConcurrentStartsOneWins) {
    constexpr int kThreads = 8;
    LifecycleNode node;
    Gate gate;
    std::atomic<int> started{0};
    std::atomic<int> rejected{0};
    std::vector<std::thread> threads;
    for (int i = 0; i < kThreads; ++i) {
        threads.emplace_back([&] {
            gate.Wait();
            try {
                node.Start();
                started.fetch_add(1);
            } catch (const UsageError&) {
                rejected.fetch_add(1);
            }
        });
    }
    gate.Open();
    for (auto& thread : threads) thread.join();

    EXPECT_EQ(started.load(), 1);
    EXPECT_EQ(rejected.load(), kThreads - 1);
    EXPECT_EQ(node.run_state(), RunState::Running);
}

TEST(LifecycleTest, ConcurrentStopsReturnOnlyAfterJoin) {
    constexpr int kThreads = 8;
    LifecycleNode node;
    node.Start();
    ASSERT_TRUE(WaitFor([&] { return node.calls() > 0; }));
    Gate gate;
    std::atomic<int> joined_after_stop{0};
    std::vector<std::thread> threads;
    for (int i = 0; i < kThreads; ++i) {
        threads.emplace_back([&] {
            gate.Wait();
            node.Stop();
            // Every caller, including those that lost the race, sees the worker joined.
            if (node.joined()) joined_after_stop.fetch_add(1);
        });
    }
    gate.Open();
    for (auto& thread : threads) thread.join();

    EXPECT_EQ(joined_after_stop.load(), kThreads);
    EXPECT_EQ(node.run_state(), RunState::Stopped);
}

TEST(LifecycleTest, StartAndStopFromDifferentThreads) {
    LifecycleNode node;
    std::thread starter([&] { node.Start(); });
    starter.join();
    ASSERT_EQ(node.run_state(), RunState::Running);
    std::thread stopper([&] { node.Stop(); });
    stopper.join();
    EXPECT_EQ(node.run_state(), RunState::Stopped);
    EXPECT_TRUE(node.joined());
}

TEST(LifecycleTest, StopFromWorkerThreadThrows) {
    LifecycleNode node;
    node.stop_from_worker_ = true;
    node.Start();
    ASSERT_TRUE(WaitFor([&] { return node.caught(); }));
    EXPECT_NE(node.worker_error().find("worker thread"), std::string::npos);
    // The component keeps running, and another thread can stop it.
    EXPECT_EQ(node.run_state(), RunState::Running);
    EXPECT_FALSE(node.joined());
    node.Stop();
    EXPECT_EQ(node.run_state(), RunState::Stopped);
    EXPECT_TRUE(node.joined());
}

// WR-01: the refusal must not depend on the state. Another thread is inside Stop()
// (Stopping, joining this worker) when the worker calls Stop() itself; waiting for
// the state to settle would wait for the thread that joins the caller.
TEST(LifecycleTest, StopFromWorkerThreadWhileAnotherThreadStopsThrows) {
    LifecycleNode node;
    node.stop_when_stopping_ = true;
    node.Start();
    ASSERT_TRUE(WaitFor([&] { return node.parked(); }));
    StopUnderWatchdog([&] { node.Stop(); }, std::chrono::seconds(10));
    ASSERT_TRUE(node.caught());
    EXPECT_NE(node.worker_error().find("worker thread"), std::string::npos);
    EXPECT_EQ(node.run_state(), RunState::Stopped);
    EXPECT_TRUE(node.joined());
}

// Same for Starting: the worker is already running while Start() has not returned.
TEST(LifecycleTest, StopFromWorkerThreadWhileStartingThrows) {
    LifecycleNode node;
    node.stop_from_worker_ = true;
    node.hold_start_for_worker_ = true;
    node.Start();
    EXPECT_TRUE(node.caught_while_starting());  // refused at once, not after Start() settled
    ASSERT_TRUE(WaitFor([&] { return node.caught(); }));
    EXPECT_NE(node.worker_error().find("worker thread"), std::string::npos);
    EXPECT_EQ(node.run_state(), RunState::Running);
    EXPECT_FALSE(node.joined());
    node.Stop();
    EXPECT_EQ(node.run_state(), RunState::Stopped);
    EXPECT_TRUE(node.joined());
}

TEST(LifecycleTest, FailedStartLeavesStopped) {
    LifecycleNode node;
    node.fail_start_ = true;
    EXPECT_THROW(node.Start(), std::runtime_error);
    EXPECT_EQ(node.run_state(), RunState::Stopped);
    EXPECT_THROW(node.Start(), UsageError);  // terminal: no retry
    EXPECT_NO_THROW(node.Stop());
}

TEST(LifecycleTest, AddObserverAllowedWhenCreatedAndStopped) {
    LifecycleNode node;
    AsyncObserver observer;
    {
        auto subscription = node.AddObserver(observer);  // Created
        EXPECT_TRUE(subscription.connected());
    }
    node.Start();
    EXPECT_THROW(static_cast<void>(node.AddObserver(observer)), UsageError);
    node.Stop();
    auto subscription = node.AddObserver(observer);  // Stopped
    EXPECT_TRUE(subscription.connected());
    EXPECT_EQ(node.async_observer_count(), 1u);
}

TEST(LifecycleTest, DestructorStopsARunningNode) {
    {
        LifecycleNode node;
        node.Start();
        ASSERT_TRUE(WaitFor([&] { return node.calls() > 0; }));
    }  // ~LifecycleNode() -> Stop(): the worker is joined before the members go
    SUCCEED();
}

}  // namespace
}  // namespace voyahslam::events
