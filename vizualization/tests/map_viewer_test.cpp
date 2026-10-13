#include <voyahviz/map_viewer.h>

#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <memory>
#include <stdexcept>
#include <thread>
#include <vector>
#include <voyahslam/events/observable.h>
#include <voyahslam/events/frame_tracked.h>
#include <voyahslam/events/subscription.h>
#include <voyahslam/features/orb_feature_extractor.h>

namespace voyahviz {
namespace {

using namespace voyahslam;
using TViewer = MapViewer<features::OrbFeatureExtractor>;
using TFrameTracked = events::FrameTracked<features::OrbFeatureExtractor>;
using TWeakEvent = std::weak_ptr<TFrameTracked>;

// Publishes a new event; returns a weak handle on it, so a test can tell
// whether every consumer has released it.
TWeakEvent Publish(events::Observable& out_producer) {
    auto event = std::make_shared<TFrameTracked>();
    TWeakEvent weak = event;
    out_producer.Notify(std::move(event));
    return weak;
}

// True once nobody holds the event any more (the viewer released it).
bool Released(const TWeakEvent& event) { return event.expired(); }

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
    std::vector<events::Subscription> subscriptions;  // declared before the viewer and the producer
    TViewer viewer(MapViewerOptions{.fps = 30.0});
    events::Observable producer;
    subscriptions.push_back(producer.ConnectAsync(viewer.inbox()));

    const TWeakEvent event = Publish(producer);
    EXPECT_FALSE(Released(event));  // held by the viewer's inbox
    viewer.Start();
    ASSERT_TRUE(WaitForProcessed(viewer, 1));
    // Acknowledged while the viewer is still running: the event is released.
    auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
    bool returned = false;
    while (!(returned = Released(event)) && std::chrono::steady_clock::now() < deadline) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    EXPECT_TRUE(returned);
    viewer.Stop();
}

TEST(MapViewerTest, StopDrainsQueuedEvents) {
    std::vector<events::Subscription> subscriptions;  // declared before the viewer and the producer
    TViewer viewer(MapViewerOptions{.fps = 30.0});
    events::Observable producer;
    subscriptions.push_back(producer.ConnectAsync(viewer.inbox()));

    viewer.Start();
    viewer.Stop();
    const TWeakEvent event = Publish(producer);
    viewer.Stop();  // idempotent; drains again
    EXPECT_EQ(viewer.events_processed(), 1u);
    EXPECT_TRUE(Released(event));
}

TEST(MapViewerTest, StopWithoutStartDrains) {
    std::vector<events::Subscription> subscriptions;  // declared before the viewer and the producer
    TViewer viewer(MapViewerOptions{.fps = 30.0});
    events::Observable producer;
    subscriptions.push_back(producer.ConnectAsync(viewer.inbox()));

    const TWeakEvent event = Publish(producer);
    viewer.Stop();
    EXPECT_EQ(viewer.events_processed(), 1u);
    EXPECT_TRUE(Released(event));
}

TEST(MapViewerTest, DestructorAcknowledgesEventsPublishedAfterStop) {
    std::vector<events::Subscription> subscriptions;  // declared before the viewer and the producer
    events::Observable producer;
    TWeakEvent event;
    {
        TViewer viewer(MapViewerOptions{.fps = 30.0});
        subscriptions.push_back(producer.ConnectAsync(viewer.inbox()));
        viewer.Start();
        viewer.Stop();
        event = Publish(producer);
    }
    EXPECT_TRUE(Released(event));
}

TEST(MapViewerTest, ReceivesFromTwoProducers) {
    // The viewer drains at its frame rate; the events published between ticks
    // wait in its inbox (here: all 20 events may).
    std::vector<events::Subscription> subscriptions;  // declared before the viewer and the producer
    TViewer viewer(MapViewerOptions{.fps = 30.0});
    events::Observable tracker;
    events::Observable mapper;
    subscriptions.push_back(tracker.ConnectAsync(viewer.inbox()));
    subscriptions.push_back(mapper.ConnectAsync(viewer.inbox()));

    viewer.Start();
    std::vector<TWeakEvent> published;
    for (int i = 0; i < 10; ++i) {
        published.push_back(Publish(tracker));
        published.push_back(Publish(mapper));
    }
    EXPECT_TRUE(WaitForProcessed(viewer, 20));
    viewer.Stop();
    EXPECT_EQ(viewer.events_processed(), 20u);
    // Every event was released by the viewer.
    for (size_t i = 0; i < published.size(); ++i) {
        ASSERT_NE(Released(published[i]), false) << "event " << i << " not released";
    }
}

TEST(MapViewerTest, DestructorStopsRunningViewerAndAcknowledges) {
    std::vector<events::Subscription> subscriptions;  // declared before the viewer and the producer
    events::Observable producer;
    TWeakEvent event;
    {
        TViewer viewer(MapViewerOptions{.fps = 30.0});
        subscriptions.push_back(producer.ConnectAsync(viewer.inbox()));
        viewer.Start();
        event = Publish(producer);
        // Leave scope while running, without Stop().
    }
    EXPECT_TRUE(Released(event));
}

TEST(MapViewerTest, StartTwiceAndRestart) {
    std::vector<events::Subscription> subscriptions;  // declared before the viewer and the producer
    TViewer viewer(MapViewerOptions{.fps = 30.0});
    events::Observable producer;
    subscriptions.push_back(producer.ConnectAsync(viewer.inbox()));

    viewer.Start();
    viewer.Start();
    viewer.Stop();
    viewer.Start();
    Publish(producer);
    EXPECT_TRUE(WaitForProcessed(viewer, 1));
    viewer.Stop();
}

TEST(MapViewerTest, RejectsNonPositiveFps) {
    EXPECT_THROW(TViewer(MapViewerOptions{.fps = 0.0}), std::invalid_argument);
    EXPECT_THROW(TViewer(MapViewerOptions{.fps = -5.0}), std::invalid_argument);
}

TEST(MapViewerTest, RunOnCurrentThreadStopsOnRequestStop) {
    std::vector<events::Subscription> subscriptions;  // declared before the viewer and the producer
    TViewer viewer(MapViewerOptions{.fps = 100.0});
    events::Observable producer;
    subscriptions.push_back(producer.ConnectAsync(viewer.inbox()));

    std::thread runner([&viewer] { viewer.RunOnCurrentThread(); });
    const TWeakEvent event = Publish(producer);
    EXPECT_TRUE(WaitForProcessed(viewer, 1));
    viewer.RequestStop();
    runner.join();
    EXPECT_EQ(viewer.events_processed(), 1u);
    EXPECT_TRUE(Released(event));
}

TEST(MapViewerTest, StopEndsRunOnCurrentThread) {
    std::vector<events::Subscription> subscriptions;  // declared before the viewer and the producer
    TViewer viewer(MapViewerOptions{.fps = 100.0});
    events::Observable producer;
    subscriptions.push_back(producer.ConnectAsync(viewer.inbox()));

    std::thread runner([&viewer] { viewer.RunOnCurrentThread(); });
    std::vector<TWeakEvent> published;
    published.push_back(Publish(producer));
    ASSERT_TRUE(WaitForProcessed(viewer, 1));
    published.push_back(Publish(producer));  // may still be queued when Stop() is called
    viewer.Stop();
    runner.join();
    // RunOnCurrentThread()'s own final drain acknowledged everything.
    EXPECT_EQ(viewer.events_processed(), 2u);
    for (size_t i = 0; i < published.size(); ++i) {
        ASSERT_NE(Released(published[i]), false) << "event " << i << " not released";
    }
}

TEST(MapViewerTest, RunOnCurrentThreadWhileStartedThrows) {
    TViewer viewer(MapViewerOptions{.fps = 30.0});
    viewer.Start();
    EXPECT_THROW(viewer.RunOnCurrentThread(), std::logic_error);
    viewer.Stop();
}

TEST(MapViewerTest, StartWhileRunOnCurrentThreadIsNoOp) {
    std::vector<events::Subscription> subscriptions;  // declared before the viewer and the producer
    TViewer viewer(MapViewerOptions{.fps = 100.0});
    events::Observable producer;
    subscriptions.push_back(producer.ConnectAsync(viewer.inbox()));

    std::thread runner([&viewer] { viewer.RunOnCurrentThread(); });
    Publish(producer);
    ASSERT_TRUE(WaitForProcessed(viewer, 1));
    viewer.Start();  // logs a warning, does nothing
    viewer.RequestStop();
    runner.join();
    viewer.Stop();  // nothing to join: no thread was started
    EXPECT_EQ(viewer.events_processed(), 1u);
}

TEST(MapViewerTest, RequestStopBeforeRunOnCurrentThreadReturns) {
    std::vector<events::Subscription> subscriptions;  // declared before the viewer and the producer
    TViewer viewer(MapViewerOptions{.fps = 100.0});
    events::Observable producer;
    subscriptions.push_back(producer.ConnectAsync(viewer.inbox()));

    const TWeakEvent event = Publish(producer);
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
    EXPECT_TRUE(Released(event));

    // The request was consumed: a later run is not ended by it.
    std::thread second([&viewer] { viewer.RunOnCurrentThread(); });
    Publish(producer);
    EXPECT_TRUE(WaitForProcessed(viewer, 2));
    viewer.RequestStop();
    second.join();
}

TEST(MapViewerTest, RequestStopBeforeStartThenStopDrains) {
    std::vector<events::Subscription> subscriptions;  // declared before the viewer and the producer
    TViewer viewer(MapViewerOptions{.fps = 100.0});
    events::Observable producer;
    subscriptions.push_back(producer.ConnectAsync(viewer.inbox()));

    const TWeakEvent event = Publish(producer);
    viewer.RequestStop();
    viewer.Start();
    viewer.Stop();
    EXPECT_EQ(viewer.events_processed(), 1u);
    EXPECT_TRUE(Released(event));

    // A later Start() runs normally.
    viewer.Start();
    Publish(producer);
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

TEST(MapViewerTest, WaitForWindowCloseReturnsWhenNoLoopRunning) {
    // No loop running (never started): takes the !loop_running_ path and
    // returns at once. show_window is set, but without Start() no window opens.
    TViewer viewer(MapViewerOptions{.fps = 30.0, .show_window = true});
    viewer.WaitForWindowClose();
    EXPECT_FALSE(viewer.window_closed());
}

} // namespace
} // namespace voyahviz
