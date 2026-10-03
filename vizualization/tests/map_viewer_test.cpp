#include <voyahviz/map_viewer.h>

#include <gtest/gtest.h>

#include <chrono>
#include <stdexcept>
#include <thread>
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
    TViewer viewer;
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
    TViewer viewer;
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
    TViewer viewer;
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
        TViewer viewer;
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
    TViewer viewer;
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
}

TEST(MapViewerTest, StartTwiceAndRestart) {
    TPool pool(1);
    TViewer viewer;
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
    EXPECT_THROW(TViewer(0.0), std::invalid_argument);
    EXPECT_THROW(TViewer(-5.0), std::invalid_argument);
}

} // namespace
} // namespace voyahviz
