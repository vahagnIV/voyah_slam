#include <voyahslam/events/events.h>
#include <voyahslam/events/observable.h>
#include <voyahslam/features/camera_frame.h>
#include <voyahslam/features/orb_feature_extractor.h>
#include <voyahslam/memory/object_pool.h>

#include <gtest/gtest.h>

#include <atomic>
#include <thread>
#include <vector>

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
    // Single slot: Create() after draining only succeeds if the event really
    // went back to the pool.
    memory::ObjectPool<OrbMapPointsAdded, memory::ThreadSafePolicy> pool(1);

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
    auto* reused = pool.TryCreate();
    EXPECT_EQ(reused, event);
    if (reused) reused->Recycle();
}

TEST(EventSystemTest, MapPointsAddedResizesAllColumnsTogether) {
    memory::ObjectPool<OrbMapPointsAdded, memory::ThreadSafePolicy> pool(1);
    auto* e = pool.Create();
    e->Resize(7);
    EXPECT_EQ(e->count(), 7u);
    EXPECT_EQ(e->positions.cols(), 7);
    EXPECT_EQ(e->descriptors.cols(), 7);
    EXPECT_EQ(e->normals.cols(), 7);
    EXPECT_EQ(e->descriptors.matrix().rows(), features::OrbFeatureExtractor::DescriptorSize);
    e->Recycle();
}

TEST(EventSystemTest, RecycledMapPointsAddedReusesItsBuffers) {
    memory::ObjectPool<OrbMapPointsAdded, memory::ThreadSafePolicy> pool(1);
    Observable observable;
    AsyncObserver observer;
    observable.ConnectAsync(observer);

    auto* e = pool.Create();
    e->Resize(500);
    const auto* positions = e->positions.matrix().data();
    const auto* descriptors = e->descriptors.matrix().data();
    observable.Notify(e);
    {
        auto listener = observer.GetListener();
        ASSERT_EQ(listener.GetNextEvent(), e);
    }

    auto* reused = pool.Create();
    ASSERT_EQ(reused, e);
    EXPECT_EQ(reused->count(), 0u);  // Reset() on reuse
    reused->Resize(300);
    EXPECT_EQ(reused->positions.matrix().data(), positions);
    EXPECT_EQ(reused->descriptors.matrix().data(), descriptors);
    reused->Recycle();
}

TEST(EventSystemTest, RecycledMapPointsUpdatedAndRemovedAreReset) {
    using OrbMapPointsUpdated = MapPointsUpdated<features::OrbFeatureExtractor>;
    memory::ObjectPool<OrbMapPointsUpdated, memory::ThreadSafePolicy> updated_pool(1);
    memory::ObjectPool<MapPointsRemoved, memory::ThreadSafePolicy> removed_pool(1);

    auto* updated = updated_pool.Create();
    updated->Resize(4);
    EXPECT_EQ(updated->ids.size(), 4u);
    EXPECT_EQ(updated->positions.cols(), 4);
    updated->Recycle();
    updated = updated_pool.Create();
    EXPECT_EQ(updated->count(), 0u);
    EXPECT_TRUE(updated->ids.empty());
    updated->Recycle();

    auto* removed = removed_pool.Create();
    removed->ids.push_back(map::MapPointId{3});
    removed->Recycle();
    removed = removed_pool.Create();
    EXPECT_TRUE(removed->ids.empty());
    removed->Recycle();
}

TEST(EventSystemTest, NotifyWithOnlySyncHandlersRecyclesPooledEvent) {
    memory::ObjectPool<OrbMapPointsAdded, memory::ThreadSafePolicy> pool(1);
    Observable observable;
    int sync_count = 0;
    observable.ConnectSync(
        [](void* ctx, Event*) { ++*static_cast<int*>(ctx); },
        &sync_count);

    auto* event = pool.Create();
    observable.Notify(event);  // Notify always takes ownership

    EXPECT_EQ(sync_count, 1);
    auto* reused = pool.TryCreate();
    EXPECT_EQ(reused, event);
    if (reused) reused->Recycle();
}

TEST(EventSystemTest, NotifyWithoutConnectionsRecyclesPooledEvent) {
    memory::ObjectPool<OrbMapPointsAdded, memory::ThreadSafePolicy> pool(1);
    Observable observable;

    auto* event = pool.Create();
    observable.Notify(event);

    auto* reused = pool.TryCreate();
    EXPECT_EQ(reused, event);
    if (reused) reused->Recycle();
}

TEST(EventSystemTest, CrossThreadFanOutReturnsEveryEventToPool) {
    constexpr int kEvents = 50'000;
    constexpr int kPoolSize = 16;
    memory::ObjectPool<PositionUpdated, memory::ThreadSafePolicy> pool(kPoolSize);
    Observable observable;
    AsyncObserver observer1;
    AsyncObserver observer2;
    observable.ConnectAsync(observer1);
    observable.ConnectAsync(observer2);

    std::atomic<bool> done{false};
    auto consume = [&done](AsyncObserver& observer, uint64_t& out_sum) {
        auto listener = observer.GetListener();
        while (true) {
            Event* e = listener.GetNextEvent();
            if (e) {
                out_sum += static_cast<PositionUpdated*>(e)->frame_id.value;
            } else if (done.load(std::memory_order_acquire)) {
                if (!(e = listener.GetNextEvent())) break;
                out_sum += static_cast<PositionUpdated*>(e)->frame_id.value;
            }
        }
    };
    uint64_t sum1 = 0;
    uint64_t sum2 = 0;
    std::thread consumer1(consume, std::ref(observer1), std::ref(sum1));
    std::thread consumer2(consume, std::ref(observer2), std::ref(sum2));

    uint64_t expected = 0;
    for (int i = 0; i < kEvents; ++i) {
        PositionUpdated* e = nullptr;
        while (!(e = pool.TryCreate())) std::this_thread::yield();
        e->frame_id = map::FrameId{static_cast<uint32_t>(i)};
        expected += static_cast<uint64_t>(i);
        observable.Notify(e);
    }
    done.store(true, std::memory_order_release);
    consumer1.join();
    consumer2.join();

    EXPECT_EQ(sum1, expected);
    EXPECT_EQ(sum2, expected);
    std::vector<PositionUpdated*> all;
    for (int i = 0; i < kPoolSize; ++i) {
        all.push_back(pool.TryCreate());
        ASSERT_NE(all.back(), nullptr);
    }
    for (auto* e : all) e->Recycle();
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
