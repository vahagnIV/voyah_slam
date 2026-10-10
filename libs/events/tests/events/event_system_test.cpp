#include <voyahslam/events/events.h>
#include <voyahslam/events/observable.h>
#include <voyahslam/features/binocular_measurement.h>
#include <voyahslam/features/measurement.h>
#include <voyahslam/features/monocular_measurement.h>
#include <voyahslam/features/orb_feature_extractor.h>
#include <voyahslam/memory/object_pool.h>

#include <gtest/gtest.h>

#include <atomic>
#include <memory>
#include <variant>
#include <thread>
#include <vector>

namespace voyahslam::events {
namespace {

using OrbMonocularMeasurement = features::MonocularMeasurement<uint8_t, 32>;
using OrbBinocularMeasurement = features::BinocularMeasurement<uint8_t, 32>;
using OrbMapTransaction = MapTransaction<features::OrbFeatureExtractor>;
using OrbFrameTracked = FrameTracked<features::OrbFeatureExtractor>;
using OrbInitializationMeasurement = InitializationMeasurement<features::OrbFeatureExtractor>;

TEST(EventSystemTest, EventTypeTag) {
    memory::ObjectPool<OrbMapTransaction, memory::ThreadSafePolicy> pool(4);
    auto* event = pool.Create();

    EXPECT_EQ(event->type, EventType::MapTransaction);
    event->Recycle();
}

TEST(EventSystemTest, ListenerAutoRecycle) {
    memory::ObjectPool<OrbMapTransaction, memory::ThreadSafePolicy> pool(2);

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
    memory::ObjectPool<OrbMapTransaction, memory::ThreadSafePolicy> pool(2);
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
        TestEvent() : Event(EventType::FrameTracked, nullptr, nullptr) {}
    } event;

    observable.Notify(&event);
    EXPECT_EQ(counter, 1);
}

TEST(EventSystemTest, ObservableAsyncFanOut) {
    // Single slot: Create() after draining only succeeds if the event really
    // went back to the pool.
    memory::ObjectPool<OrbMapTransaction, memory::ThreadSafePolicy> pool(1);

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
        EXPECT_EQ(got1->type, EventType::MapTransaction);
    }

    {
        auto listener2 = observer2.GetListener();
        auto* got2 = listener2.GetNextEvent();
        ASSERT_NE(got2, nullptr);
        EXPECT_EQ(got2->type, EventType::MapTransaction);
    }

    // Both listeners recycled → ref count hit 0 → event returned to pool
    auto* reused = pool.TryCreate();
    EXPECT_EQ(reused, event);
    if (reused) reused->Recycle();
}

TEST(EventSystemTest, MapTransactionResizesPointColumnsTogether) {
    memory::ObjectPool<OrbMapTransaction, memory::ThreadSafePolicy> pool(1);
    auto* e = pool.Create();
    e->ResizeNewPoints(7);
    EXPECT_EQ(e->num_new_points(), 7u);
    EXPECT_EQ(e->new_positions.cols(), 7);
    EXPECT_EQ(e->new_descriptors.cols(), 7);
    EXPECT_EQ(e->new_normals.cols(), 7);
    EXPECT_EQ(e->new_descriptors.matrix().rows(), features::OrbFeatureExtractor::DescriptorSize);
    e->ResizeUpdatedPoints(3);
    EXPECT_EQ(e->updated_ids.size(), 3u);
    EXPECT_EQ(e->updated_positions.cols(), 3);
    EXPECT_EQ(e->updated_descriptors.cols(), 3);
    EXPECT_EQ(e->updated_normals.cols(), 3);
    e->Recycle();
}

// Fills every part of `out_event`.
void FillEveryPart(OrbMapTransaction& out_event) {
    out_event.new_map = true;
    out_event.start_frame = map::FrameId{4};
    auto& frame = out_event.added_frames.emplace_back();
    frame.id = map::FrameId{4};
    frame.observed_features.push_back(4);
    frame.observed_points.push_back(map::MapPointId{5});
    frame.parent = map::FrameId{3};
    out_event.new_points_start_id = map::MapPointId{9};
    out_event.ResizeNewPoints(500);
    out_event.added_observations.push_back({map::FrameId{1}, 2, map::MapPointId{3}});
    out_event.ResizeUpdatedPoints(20);
    out_event.removed_observations.push_back({map::FrameId{1}, map::MapPointId{3}});
    out_event.replaced_points.push_back({map::MapPointId{6}, map::MapPointId{7}});
    out_event.removed_points.push_back(map::MapPointId{8});
    out_event.updated_poses.emplace_back(map::FrameId{1}, geometry::RigidTransformation::Identity());
    out_event.removed_frames.push_back(map::FrameId{2});
    out_event.parent_changes.emplace_back(map::FrameId{4}, map::FrameId{1});
    out_event.added_loop_edges.emplace_back(map::FrameId{4}, map::FrameId{1});
    out_event.map_epoch = 3;
}

TEST(EventSystemTest, RecycledMapTransactionIsResetAndReusesItsBuffers) {
    memory::ObjectPool<OrbMapTransaction, memory::ThreadSafePolicy> pool(1);
    Observable observable;
    AsyncObserver observer;
    observable.ConnectAsync(observer);

    auto* e = pool.Create();
    FillEveryPart(*e);
    const auto* positions = e->new_positions.matrix().data();
    const auto* descriptors = e->new_descriptors.matrix().data();
    observable.Notify(e);
    {
        auto listener = observer.GetListener();
        ASSERT_EQ(listener.GetNextEvent(), e);
    }

    auto* reused = pool.Create();
    ASSERT_EQ(reused, e);
    // Reset() on reuse clears every part.
    EXPECT_FALSE(reused->new_map);
    EXPECT_FALSE(reused->start_frame.has_value());
    EXPECT_TRUE(reused->added_frames.empty());
    EXPECT_EQ(reused->new_points_start_id, map::MapPointId{0});
    EXPECT_EQ(reused->num_new_points(), 0u);
    EXPECT_TRUE(reused->added_observations.empty());
    EXPECT_TRUE(reused->updated_ids.empty());
    EXPECT_EQ(reused->updated_positions.cols(), 0);
    EXPECT_TRUE(reused->removed_observations.empty());
    EXPECT_TRUE(reused->replaced_points.empty());
    EXPECT_TRUE(reused->removed_points.empty());
    EXPECT_TRUE(reused->updated_poses.empty());
    EXPECT_TRUE(reused->removed_frames.empty());
    EXPECT_TRUE(reused->parent_changes.empty());
    EXPECT_TRUE(reused->added_loop_edges.empty());
    EXPECT_EQ(reused->map_epoch, 0u);
    reused->ResizeNewPoints(300);
    EXPECT_EQ(reused->new_positions.matrix().data(), positions);
    EXPECT_EQ(reused->new_descriptors.matrix().data(), descriptors);
    reused->Recycle();
}

TEST(EventSystemTest, RecycledMapTransactionKeepsCapacity) {
    memory::ObjectPool<OrbMapTransaction, memory::ThreadSafePolicy> pool(1);
    auto* e = pool.Create();
    FillEveryPart(*e);
    e->Recycle();

    e = pool.Create();
    EXPECT_GE(e->added_frames.capacity(), 1u);
    EXPECT_GE(e->added_observations.capacity(), 1u);
    EXPECT_GE(e->updated_ids.capacity(), 20u);
    EXPECT_GE(e->updated_positions.capacity(), 20);
    EXPECT_GE(e->new_normals.capacity(), 500);
    EXPECT_GE(e->removed_observations.capacity(), 1u);
    EXPECT_GE(e->replaced_points.capacity(), 1u);
    EXPECT_GE(e->removed_points.capacity(), 1u);
    EXPECT_GE(e->updated_poses.capacity(), 1u);
    EXPECT_GE(e->removed_frames.capacity(), 1u);
    EXPECT_GE(e->parent_changes.capacity(), 1u);
    EXPECT_GE(e->added_loop_edges.capacity(), 1u);
    e->Recycle();
}

TEST(EventSystemTest, NotifyWithOnlySyncHandlersRecyclesPooledEvent) {
    memory::ObjectPool<OrbMapTransaction, memory::ThreadSafePolicy> pool(1);
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
    memory::ObjectPool<OrbMapTransaction, memory::ThreadSafePolicy> pool(1);
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
    memory::ObjectPool<OrbMapTransaction, memory::ThreadSafePolicy> pool(kPoolSize);
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
                out_sum += static_cast<OrbMapTransaction*>(e)->start_frame->value;
            } else if (done.load(std::memory_order_acquire)) {
                if (!(e = listener.GetNextEvent())) break;
                out_sum += static_cast<OrbMapTransaction*>(e)->start_frame->value;
            }
        }
    };
    uint64_t sum1 = 0;
    uint64_t sum2 = 0;
    std::thread consumer1(consume, std::ref(observer1), std::ref(sum1));
    std::thread consumer2(consume, std::ref(observer2), std::ref(sum2));

    uint64_t expected = 0;
    for (int i = 0; i < kEvents; ++i) {
        OrbMapTransaction* e = nullptr;
        while (!(e = pool.TryCreate())) std::this_thread::yield();
        e->start_frame = map::FrameId{static_cast<uint32_t>(i)};
        expected += static_cast<uint64_t>(i);
        observable.Notify(e);
    }
    done.store(true, std::memory_order_release);
    consumer1.join();
    consumer2.join();

    EXPECT_EQ(sum1, expected);
    EXPECT_EQ(sum2, expected);
    std::vector<OrbMapTransaction*> all;
    for (int i = 0; i < kPoolSize; ++i) {
        all.push_back(pool.TryCreate());
        ASSERT_NE(all.back(), nullptr);
    }
    for (auto* e : all) e->Recycle();
}

TEST(EventSystemTest, ObservableSyncAndAsync) {
    memory::ObjectPool<OrbMapTransaction, memory::ThreadSafePolicy> pool(4);

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
    EXPECT_EQ(got->type, EventType::MapTransaction);
}

TEST(EventSystemTest, RecycledFrameTrackedIsReset) {
    memory::ObjectPool<OrbFrameTracked, memory::ThreadSafePolicy> pool(1);
    auto* event = pool.Create();
    event->timestamp = 1.5;
    event->observed_features = {1, 2};
    event->observed_points = {map::MapPointId{3}, map::MapPointId{4}};
    event->visible_points = {map::MapPointId{5}};
    event->reference_frame_id = map::FrameId{42};
    event->local_frames = {map::FrameId{42}};
    event->stage = TrackingStage::ReferenceFrame;
    event->num_matches = 7;
    event->num_inliers = 2;
    event->map_epoch = 5;
    event->Recycle();

    auto* reused = pool.Create();
    ASSERT_EQ(reused, event);
    EXPECT_EQ(reused->timestamp, 0.0);
    EXPECT_TRUE(reused->observed_features.empty());
    EXPECT_TRUE(reused->observed_points.empty());
    EXPECT_TRUE(reused->visible_points.empty());
    EXPECT_FALSE(reused->reference_frame_id.has_value());
    EXPECT_TRUE(reused->local_frames.empty());
    EXPECT_EQ(reused->stage, TrackingStage::MotionModel);
    EXPECT_EQ(reused->num_matches, 0u);
    EXPECT_EQ(reused->num_inliers, 0u);
    EXPECT_EQ(reused->map_epoch, 0u);
    reused->Recycle();
}

// A pooled event keeps its slot's object alive; on reuse it lets go of the
// shared measurement it carried.
TEST(EventSystemTest, RecycledEventsReleaseTheirMeasurement) {
    using TMeasurement = features::TMeasurement<features::OrbFeatureExtractor>;
    const auto measurement = std::make_shared<const TMeasurement>(
        features::TMonocularMeasurement<features::OrbFeatureExtractor>{});

    memory::ObjectPool<OrbFrameTracked, memory::ThreadSafePolicy> tracked_pool(1);
    auto* tracked = tracked_pool.Create();
    tracked->measurement = measurement;
    tracked->Recycle();
    memory::ObjectPool<OrbMapTransaction, memory::ThreadSafePolicy> transaction_pool(1);
    auto* transaction = transaction_pool.Create();
    transaction->added_frames.emplace_back().measurement = measurement;
    transaction->Recycle();
    EXPECT_EQ(measurement.use_count(), 3);

    auto* reused_tracked = tracked_pool.Create();
    ASSERT_EQ(reused_tracked, tracked);
    EXPECT_EQ(reused_tracked->measurement, nullptr);
    auto* reused_transaction = transaction_pool.Create();
    ASSERT_EQ(reused_transaction, transaction);
    EXPECT_EQ(measurement.use_count(), 1);
    reused_tracked->Recycle();
    reused_transaction->Recycle();
}

TEST(EventSystemTest, RecycledTrackingLostIsReset) {
    memory::ObjectPool<TrackingLost, memory::ThreadSafePolicy> pool(1);
    auto* event = pool.Create();
    EXPECT_TRUE(event->final);  // a TrackingLost ends the map unless it says otherwise
    event->timestamp = 1.5;
    event->final = false;
    event->Recycle();

    auto* reused = pool.Create();
    ASSERT_EQ(reused, event);
    EXPECT_EQ(reused->timestamp, 0.0);
    EXPECT_TRUE(reused->final);
    reused->Recycle();
}

TEST(EventSystemTest, LoopDetectedTypeTag) {
    memory::ObjectPool<LoopDetected, memory::ThreadSafePolicy> pool(4);
    auto* event = pool.Create();
    EXPECT_EQ(event->type, EventType::LoopDetected);
    event->Recycle();
}

TEST(EventSystemTest, RecycledGlobalBAFinishedIsReset) {
    memory::ObjectPool<GlobalBAFinished, memory::ThreadSafePolicy> pool(1);
    auto* event = pool.Create();
    EXPECT_EQ(event->type, EventType::GlobalBAFinished);
    event->map_epoch = 3;
    event->frame_poses.emplace_back(map::FrameId{7}, geometry::RigidTransformation::Identity());
    event->point_positions.emplace_back(map::MapPointId{2}, geometry::TPoint3DF(1, 2, 3));
    event->priority = true;
    event->Recycle();

    auto* reused = pool.Create();
    ASSERT_EQ(reused, event);
    EXPECT_EQ(reused->type, EventType::GlobalBAFinished);
    EXPECT_EQ(reused->map_epoch, 0u);
    EXPECT_TRUE(reused->frame_poses.empty());
    EXPECT_TRUE(reused->point_positions.empty());
    EXPECT_FALSE(reused->priority);
    reused->Recycle();
}

TEST(EventSystemTest, FrameTrackedTypeTag) {
    memory::ObjectPool<OrbFrameTracked, memory::ThreadSafePolicy> pool(4);
    auto* event = pool.Create();
    EXPECT_EQ(event->type, EventType::FrameTracked);
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
        StackEvent() : Event(EventType::FrameTracked, nullptr, nullptr) {}
    } event;

    observable.Notify(&event);
    EXPECT_EQ(counter, 1);
    // No crash: set_ref_count_ was nullptr and was never called
}

TEST(EventSystemTest, NewEventTypeTags) {
    memory::ObjectPool<OrbMapTransaction, memory::ThreadSafePolicy> created(1);
    memory::ObjectPool<TrackingLost, memory::ThreadSafePolicy> lost(1);
    memory::ObjectPool<OrbInitializationMeasurement, memory::ThreadSafePolicy> init(1);

    auto* c = created.Create();
    auto* l = lost.Create();
    auto* i = init.Create();
    EXPECT_EQ(c->type, EventType::MapTransaction);
    EXPECT_EQ(l->type, EventType::TrackingLost);
    EXPECT_EQ(i->type, EventType::InitializationMeasurement);
    EXPECT_EQ(c->map_id, map::MapId{0});
    EXPECT_EQ(l->map_id, map::MapId{0});
    EXPECT_EQ(i->map_id, map::MapId{0});
    c->Recycle();
    l->Recycle();
    i->Recycle();
}

TEST(EventSystemTest, NewFrameHoldsMonoOrStereo) {
    memory::ObjectPool<OrbMapTransaction, memory::ThreadSafePolicy> pool(1);

    auto* event = pool.Create();
    using TMeasurement = features::TMeasurement<features::OrbFeatureExtractor>;
    event->added_frames.emplace_back().measurement = std::make_shared<const TMeasurement>(
        features::TMonocularMeasurement<features::OrbFeatureExtractor>{});
    EXPECT_TRUE(std::holds_alternative<features::TMonocularMeasurement<features::OrbFeatureExtractor>>(
        *event->added_frames[0].measurement));
    event->added_frames.emplace_back().measurement = std::make_shared<const TMeasurement>(
        features::TBinocularMeasurement<features::OrbFeatureExtractor>{});
    EXPECT_TRUE(std::holds_alternative<features::TBinocularMeasurement<features::OrbFeatureExtractor>>(
        *event->added_frames[1].measurement));
    event->Recycle();
}

} // namespace
} // namespace voyahslam::events

namespace voyahslam::events {
namespace {

TEST(AsyncObserverTest, PriorityCountTracksPushPop) {
    memory::ObjectPool<OrbFrameTracked, memory::ThreadSafePolicy> pool(3);
    AsyncObserver observer;
    auto listener = observer.GetListener();
    auto* a = pool.Create();
    auto* b = pool.Create();
    auto* c = pool.Create();
    a->priority = true;
    b->priority = false;
    c->priority = true;
    for (Event* e : {static_cast<Event*>(a), static_cast<Event*>(b), static_cast<Event*>(c)}) {
        e->set_ref_count_(e, 1);
    }
    EXPECT_EQ(observer.pending_priority(), 0u);
    observer.Push(a);
    EXPECT_EQ(observer.pending_priority(), 1u);
    observer.Push(b);
    EXPECT_EQ(observer.pending_priority(), 1u);
    observer.Push(c);
    EXPECT_EQ(observer.pending_priority(), 2u);
    EXPECT_EQ(&observer.pending_priority_counter(), &observer.pending_priority_counter());

    EXPECT_EQ(listener.GetNextEvent(), a);
    EXPECT_EQ(observer.pending_priority(), 1u);
    EXPECT_EQ(listener.GetNextEvent(), b);
    EXPECT_EQ(observer.pending_priority(), 1u);
    EXPECT_EQ(listener.GetNextEvent(), c);
    EXPECT_EQ(observer.pending_priority(), 0u);
    EXPECT_EQ(listener.GetNextEvent(), nullptr);
    EXPECT_EQ(observer.pending_priority(), 0u);
}

TEST(AsyncObserverTest, TryPopDecrementsAndReusedEventIsNotCounted) {
    memory::ObjectPool<OrbFrameTracked, memory::ThreadSafePolicy> pool(1);
    AsyncObserver observer;
    auto* event = pool.Create();
    event->priority = true;
    event->set_ref_count_(event, 1);
    observer.Push(event);
    ASSERT_EQ(observer.pending_priority(), 1u);
    Event* popped = nullptr;
    ASSERT_TRUE(observer.TryPop(popped));
    EXPECT_EQ(observer.pending_priority(), 0u);
    popped->recycle_(popped);

    auto* reused = pool.Create();
    ASSERT_EQ(reused, event);
    reused->priority = false;  // the producer resets it on every publish
    reused->set_ref_count_(reused, 1);
    observer.Push(reused);
    EXPECT_EQ(observer.pending_priority(), 0u);
    ASSERT_TRUE(observer.TryPop(popped));
    EXPECT_EQ(observer.pending_priority(), 0u);
    popped->recycle_(popped);
    EXPECT_FALSE(observer.TryPop(popped));
}

}  // namespace
}  // namespace voyahslam::events
