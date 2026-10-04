#include <voyahviz/map_viewer.h>

#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <stdexcept>
#include <thread>
#include <vector>
#include <voyahslam/events/observable.h>
#include <voyahslam/events/position_updated.h>
#include <voyahslam/features/orb_feature_extractor.h>
#include <voyahslam/memory/object_pool.h>

namespace voyahviz {
namespace {

using namespace voyahslam;
using TViewer = MapViewer<features::OrbFeatureExtractor>;
using TPool = memory::ObjectPool<events::PositionUpdated, memory::ThreadSafePolicy>;

void Publish(events::Observable& out_producer, TPool& pool) {
    events::PositionUpdated* event = pool.Create();
    out_producer.Notify(event);
}

// True if a slot can be taken from the pool (i.e. it was returned); gives it back.
bool SlotReturned(TPool& pool) {
    events::PositionUpdated* event = pool.TryCreate();
    if (event == nullptr) return false;
    event->Recycle();
    return true;
}

// Polls for up to 2 s.
bool WaitForProcessed(const TViewer& viewer, size_t expected) {
    auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
    while (viewer.events_processed() < expected) {
        if (std::chrono::steady_clock::now() > deadline) return false;
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    return viewer.events_processed() == expected;
}

TEST(MapViewerTest, ProcessesEventPublishedBeforeStart) {
    TPool pool(1);
    TViewer viewer(MapViewerOptions{.fps = 30.0});
    events::Observable producer;
    producer.ConnectAsync(viewer.inbox());

    Publish(producer, pool);
    EXPECT_FALSE(SlotReturned(pool));  // held by the viewer's inbox
    viewer.Start();
    ASSERT_TRUE(WaitForProcessed(viewer, 1));
    // Acknowledged while the viewer is still running: the slot comes back.
    auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
    bool returned = false;
    while (!(returned = SlotReturned(pool)) && std::chrono::steady_clock::now() < deadline) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    EXPECT_TRUE(returned);
    viewer.Stop();
}

TEST(MapViewerTest, StopDrainsQueuedEvents) {
    TPool pool(1);
    TViewer viewer(MapViewerOptions{.fps = 30.0});
    events::Observable producer;
    producer.ConnectAsync(viewer.inbox());

    viewer.Start();
    viewer.Stop();
    Publish(producer, pool);
    viewer.Stop();  // idempotent; drains again
    EXPECT_EQ(viewer.events_processed(), 1u);
    EXPECT_TRUE(SlotReturned(pool));
}

TEST(MapViewerTest, StopWithoutStartDrains) {
    TPool pool(1);
    TViewer viewer(MapViewerOptions{.fps = 30.0});
    events::Observable producer;
    producer.ConnectAsync(viewer.inbox());

    Publish(producer, pool);
    viewer.Stop();
    EXPECT_EQ(viewer.events_processed(), 1u);
    EXPECT_TRUE(SlotReturned(pool));
}

TEST(MapViewerTest, DestructorAcknowledgesEventsPublishedAfterStop) {
    TPool pool(1);
    events::Observable producer;
    {
        TViewer viewer(MapViewerOptions{.fps = 30.0});
        producer.ConnectAsync(viewer.inbox());
        viewer.Start();
        viewer.Stop();
        Publish(producer, pool);
    }
    EXPECT_TRUE(SlotReturned(pool));
}

TEST(MapViewerTest, ReceivesFromTwoProducers) {
    // The viewer drains at its frame rate, so the pool must cover the whole
    // backlog published between ticks (here: all 20 events).
    TPool pool(32);
    TViewer viewer(MapViewerOptions{.fps = 30.0});
    events::Observable tracker;
    events::Observable mapper;
    tracker.ConnectAsync(viewer.inbox());
    mapper.ConnectAsync(viewer.inbox());

    viewer.Start();
    for (int i = 0; i < 10; ++i) {
        Publish(tracker, pool);
        Publish(mapper, pool);
    }
    EXPECT_TRUE(WaitForProcessed(viewer, 20));
    viewer.Stop();
    EXPECT_EQ(viewer.events_processed(), 20u);
    // Every slot came back: all 32 can be taken at once.
    std::vector<events::PositionUpdated*> taken;
    for (int i = 0; i < 32; ++i) {
        taken.push_back(pool.TryCreate());
        ASSERT_NE(taken.back(), nullptr) << "slot " << i << " not returned";
    }
    for (auto* event : taken) event->Recycle();
}

TEST(MapViewerTest, DestructorStopsRunningViewerAndAcknowledges) {
    TPool pool(1);
    events::Observable producer;
    {
        TViewer viewer(MapViewerOptions{.fps = 30.0});
        producer.ConnectAsync(viewer.inbox());
        viewer.Start();
        Publish(producer, pool);
        // Leave scope while running, without Stop().
    }
    EXPECT_TRUE(SlotReturned(pool));
}

TEST(MapViewerTest, StartTwiceAndRestart) {
    TPool pool(1);
    TViewer viewer(MapViewerOptions{.fps = 30.0});
    events::Observable producer;
    producer.ConnectAsync(viewer.inbox());

    viewer.Start();
    viewer.Start();
    viewer.Stop();
    viewer.Start();
    Publish(producer, pool);
    EXPECT_TRUE(WaitForProcessed(viewer, 1));
    viewer.Stop();
}

TEST(MapViewerTest, RejectsNonPositiveFps) {
    EXPECT_THROW(TViewer(MapViewerOptions{.fps = 0.0}), std::invalid_argument);
    EXPECT_THROW(TViewer(MapViewerOptions{.fps = -5.0}), std::invalid_argument);
}

TEST(MapViewerTest, RunOnCurrentThreadStopsOnRequestStop) {
    TPool pool(1);
    TViewer viewer(MapViewerOptions{.fps = 100.0});
    events::Observable producer;
    producer.ConnectAsync(viewer.inbox());

    std::thread runner([&viewer] { viewer.RunOnCurrentThread(); });
    Publish(producer, pool);
    EXPECT_TRUE(WaitForProcessed(viewer, 1));
    viewer.RequestStop();
    runner.join();
    EXPECT_EQ(viewer.events_processed(), 1u);
    EXPECT_TRUE(SlotReturned(pool));
}

TEST(MapViewerTest, StopEndsRunOnCurrentThread) {
    TPool pool(2);
    TViewer viewer(MapViewerOptions{.fps = 100.0});
    events::Observable producer;
    producer.ConnectAsync(viewer.inbox());

    std::thread runner([&viewer] { viewer.RunOnCurrentThread(); });
    Publish(producer, pool);
    ASSERT_TRUE(WaitForProcessed(viewer, 1));
    Publish(producer, pool);  // may still be queued when Stop() is called
    viewer.Stop();
    runner.join();
    // RunOnCurrentThread()'s own final drain acknowledged everything.
    EXPECT_EQ(viewer.events_processed(), 2u);
    std::vector<events::PositionUpdated*> taken;
    for (int i = 0; i < 2; ++i) {
        taken.push_back(pool.TryCreate());
        ASSERT_NE(taken.back(), nullptr) << "slot " << i << " not returned";
    }
    for (auto* event : taken) event->Recycle();
}

TEST(MapViewerTest, RunOnCurrentThreadWhileStartedThrows) {
    TViewer viewer(MapViewerOptions{.fps = 30.0});
    viewer.Start();
    EXPECT_THROW(viewer.RunOnCurrentThread(), std::logic_error);
    viewer.Stop();
}

TEST(MapViewerTest, StartWhileRunOnCurrentThreadIsNoOp) {
    TPool pool(1);
    TViewer viewer(MapViewerOptions{.fps = 100.0});
    events::Observable producer;
    producer.ConnectAsync(viewer.inbox());

    std::thread runner([&viewer] { viewer.RunOnCurrentThread(); });
    Publish(producer, pool);
    ASSERT_TRUE(WaitForProcessed(viewer, 1));
    viewer.Start();  // logs a warning, does nothing
    viewer.RequestStop();
    runner.join();
    viewer.Stop();  // nothing to join: no thread was started
    EXPECT_EQ(viewer.events_processed(), 1u);
}

TEST(MapViewerTest, RequestStopBeforeRunOnCurrentThreadReturns) {
    TPool pool(1);
    TViewer viewer(MapViewerOptions{.fps = 100.0});
    events::Observable producer;
    producer.ConnectAsync(viewer.inbox());

    Publish(producer, pool);
    viewer.RequestStop();  // e.g. the worker finished before the loop was entered
    std::atomic<bool> returned{false};
    std::thread runner([&] {
        viewer.RunOnCurrentThread();
        returned.store(true);
    });
    auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
    while (!returned.load() && std::chrono::steady_clock::now() < deadline) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    EXPECT_TRUE(returned.load()) << "RunOnCurrentThread() ignored the earlier RequestStop()";
    if (!returned.load()) viewer.RequestStop();  // unblock so the test can finish
    runner.join();
    EXPECT_EQ(viewer.events_processed(), 1u);
    EXPECT_TRUE(SlotReturned(pool));

    // The request was consumed: a later run is not ended by it.
    std::thread second([&viewer] { viewer.RunOnCurrentThread(); });
    Publish(producer, pool);
    EXPECT_TRUE(WaitForProcessed(viewer, 2));
    viewer.RequestStop();
    second.join();
}

TEST(MapViewerTest, RequestStopBeforeStartThenStopDrains) {
    TPool pool(1);
    TViewer viewer(MapViewerOptions{.fps = 100.0});
    events::Observable producer;
    producer.ConnectAsync(viewer.inbox());

    Publish(producer, pool);
    viewer.RequestStop();
    viewer.Start();
    viewer.Stop();
    EXPECT_EQ(viewer.events_processed(), 1u);
    EXPECT_TRUE(SlotReturned(pool));

    // A later Start() runs normally.
    viewer.Start();
    Publish(producer, pool);
    EXPECT_TRUE(WaitForProcessed(viewer, 2));
    viewer.Stop();
}

TEST(MapViewerTest, WaitForWindowCloseReturnsWhenHeadless) {
    TViewer never_started(MapViewerOptions{.fps = 30.0});
    never_started.WaitForWindowClose();
    EXPECT_FALSE(never_started.window_closed());

    TViewer started(MapViewerOptions{.fps = 30.0, .show_window = false});
    started.Start();
    started.WaitForWindowClose();
    EXPECT_FALSE(started.window_closed());
    started.Stop();
}

TEST(MapViewerTest, WaitForWindowCloseReturnsAfterLoopExit) {
    TViewer viewer(MapViewerOptions{.fps = 30.0});
    viewer.Start();
    viewer.Stop();
    viewer.WaitForWindowClose();
    EXPECT_FALSE(viewer.window_closed());
}

} // namespace
} // namespace voyahviz
