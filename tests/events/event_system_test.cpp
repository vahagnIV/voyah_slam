#include <voyahslam/events/map_events.h>
#include <voyahslam/events/observable.h>
#include <voyahslam/features/camera_frame.h>
#include <voyahslam/features/orb_feature_extractor.h>
#include <voyahslam/memory/object_pool.h>

#include <gtest/gtest.h>

namespace voyahslam::events {
namespace {

using OrbMapPointsAdded = MapPointsAdded<features::OrbFeatureExtractor>;
using OrbCameraFrame = features::CameraFrame<uint8_t, 32>;
using OrbKeyFrameCreated = KeyFrameCreated<features::OrbFeatureExtractor, OrbCameraFrame>;
using OrbNewKeyFrameCandidate = NewKeyFrameCandidate<features::OrbFeatureExtractor, OrbCameraFrame>;

TEST(EventSystemTest, EventTypeTag) {
    memory::ObjectPool<OrbMapPointsAdded, memory::ThreadSafePolicy> pool(4);
    auto* event = pool.Create();

    EXPECT_EQ(event->type, EventType::MapPointsAdded);
    event->Recycle();
}

TEST(EventSystemTest, ListenerAutoRecycle) {
    memory::ObjectPool<OrbMapPointsAdded, memory::ThreadSafePolicy> pool(2);

    moodycamel::ConcurrentQueue<Event*> queue;

    auto* e1 = pool.Create();
    auto* e2 = pool.Create();
    queue.enqueue(e1);
    queue.enqueue(e2);

    {
        Listener listener(queue);
        auto* got1 = listener.GetNextEvent();
        ASSERT_NE(got1, nullptr);
        auto* got2 = listener.GetNextEvent();
        ASSERT_NE(got2, nullptr);
        EXPECT_EQ(listener.GetNextEvent(), nullptr);
    }
    // Listener destructor recycles e2; GetNextEvent recycled e1 when moving to e2.

    auto* e3 = pool.Create();
    EXPECT_NE(e3, nullptr);
    auto* e4 = pool.Create();
    EXPECT_NE(e4, nullptr);
    e3->Recycle();
    e4->Recycle();
}

TEST(EventSystemTest, ListenerMoveSemantics) {
    memory::ObjectPool<OrbMapPointsAdded, memory::ThreadSafePolicy> pool(2);
    moodycamel::ConcurrentQueue<Event*> queue;

    auto* event = pool.Create();
    queue.enqueue(event);

    Listener listener1(queue);
    auto* got = listener1.GetNextEvent();
    ASSERT_NE(got, nullptr);

    Listener listener2(std::move(listener1));

    // listener2 destructor recycles; listener1 must not double-recycle
    // Scope exit: both destructors run, no crash
    auto* e2 = pool.Create();
    // If double-recycle happened, ref_count would go negative and pool.Create would fail
    EXPECT_NE(e2, nullptr);
    e2->Recycle();
}

TEST(EventSystemTest, ObservableSyncHandler) {
    Observable observable;
    int counter = 0;

    observable.ConnectSync(
        [](void* ctx, Event*) { ++*static_cast<int*>(ctx); },
        &counter);

    // Stack-allocated event for sync-only path (no pool needed)
    struct TestEvent : Event {
        TestEvent() : Event(EventType::PositionUpdated, nullptr, nullptr) {}
    } event;

    observable.Notify(&event);
    EXPECT_EQ(counter, 1);
}

TEST(EventSystemTest, ObservableAsyncFanOut) {
    memory::ObjectPool<OrbMapPointsAdded, memory::ThreadSafePolicy> pool(4);

    Observable observable;
    AsyncObserver observer1;
    AsyncObserver observer2;

    observable.ConnectAsync(observer1);
    observable.ConnectAsync(observer2);

    auto* event = pool.Create();
    observable.Notify(event);

    {
        auto listener1 = observer1.GetListener();
        auto* got1 = listener1.GetNextEvent();
        ASSERT_NE(got1, nullptr);
        EXPECT_EQ(got1->type, EventType::MapPointsAdded);
    }

    {
        auto listener2 = observer2.GetListener();
        auto* got2 = listener2.GetNextEvent();
        ASSERT_NE(got2, nullptr);
        EXPECT_EQ(got2->type, EventType::MapPointsAdded);
    }

    // Both listeners recycled → ref count hit 0 → event returned to pool
    auto* reused = pool.Create();
    EXPECT_NE(reused, nullptr);
    reused->Recycle();
}

TEST(EventSystemTest, ObservableSyncAndAsync) {
    memory::ObjectPool<OrbMapPointsAdded, memory::ThreadSafePolicy> pool(4);

    Observable observable;
    int sync_count = 0;
    AsyncObserver async_obs;

    observable.ConnectSync(
        [](void* ctx, Event*) { ++*static_cast<int*>(ctx); },
        &sync_count);
    observable.ConnectAsync(async_obs);

    auto* event = pool.Create();
    observable.Notify(event);

    EXPECT_EQ(sync_count, 1);

    auto listener = async_obs.GetListener();
    auto* got = listener.GetNextEvent();
    ASSERT_NE(got, nullptr);
    EXPECT_EQ(got->type, EventType::MapPointsAdded);
}

TEST(EventSystemTest, PositionUpdatedHasFrameId) {
    memory::ObjectPool<PositionUpdated, memory::ThreadSafePolicy> pool(4);
    auto* event = pool.Create();
    event->frame_id = map::FrameId{42};
    EXPECT_EQ(event->frame_id, map::FrameId{42});
    event->Recycle();
}

TEST(EventSystemTest, KeyFrameCreatedTypeTag) {
    memory::ObjectPool<OrbKeyFrameCreated, memory::ThreadSafePolicy> pool(4);
    auto* event = pool.Create();
    EXPECT_EQ(event->type, EventType::KeyFrameCreated);
    event->Recycle();
}

TEST(EventSystemTest, NewKeyFrameCandidateTypeTag) {
    memory::ObjectPool<OrbNewKeyFrameCandidate, memory::ThreadSafePolicy> pool(4);
    auto* event = pool.Create();
    EXPECT_EQ(event->type, EventType::NewKeyFrameCandidate);
    event->Recycle();
}

TEST(EventSystemTest, ObservableNoAsyncSkipsRefCount) {
    Observable observable;
    int counter = 0;

    observable.ConnectSync(
        [](void* ctx, Event*) { ++*static_cast<int*>(ctx); },
        &counter);

    // Stack-allocated event with no pool — sync-only path must not call set_ref_count_
    struct StackEvent : Event {
        StackEvent() : Event(EventType::PositionUpdated, nullptr, nullptr) {}
    } event;

    observable.Notify(&event);
    EXPECT_EQ(counter, 1);
    // No crash: set_ref_count_ was nullptr and was never called
}

} // namespace
} // namespace voyahslam::events
