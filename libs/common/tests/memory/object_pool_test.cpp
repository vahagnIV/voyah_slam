#include <voyahslam/memory/object_pool.h>
#include <voyahslam/memory/recyclable.h>

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <thread>
#include <vector>

namespace voyahslam::memory {
namespace {

struct TestObject : Recyclable<TestObject, NoSyncPolicy> {
    int value = 0;
};

struct ThreadSafeTestObject : Recyclable<ThreadSafeTestObject, ThreadSafePolicy> {
    int value = 0;
};

struct BufferObject : Recyclable<BufferObject, NoSyncPolicy> {
    std::vector<int> buffer;
    int reset_calls = 0;
    void Reset() {
        buffer.clear();
        ++reset_calls;
    }
};

int g_destroyed = 0;
struct CountingObject : Recyclable<CountingObject, NoSyncPolicy> {
    ~CountingObject() { ++g_destroyed; }
};

// Create a pooled object and set its value (objects are default-constructed).
template <typename TPool>
auto* Make(TPool& pool, int value) {
    auto* obj = pool.Create();
    obj->value = value;
    return obj;
}

TEST(ObjectPoolTest, CreateAndRecycle) {
    ObjectPool<TestObject, NoSyncPolicy> pool(4);

    auto* obj = Make(pool, 42);
    ASSERT_NE(obj, nullptr);
    EXPECT_EQ(obj->value, 42);

    auto* addr = obj;
    obj->Recycle();

    auto* obj2 = Make(pool, 99);
    EXPECT_EQ(obj2, addr);
    EXPECT_EQ(obj2->value, 99);
    obj2->Recycle();
}

TEST(ObjectPoolTest, RefCounting) {
    ObjectPool<TestObject, NoSyncPolicy> pool(2);

    auto* obj = Make(pool, 10);
    obj->SetRefCount(3);

    obj->Recycle();
    // ref_count is now 2 — pool should NOT have reclaimed
    auto* other = Make(pool, 20);
    EXPECT_NE(other, obj);  // obj still held, got a different slot
    other->Recycle();

    obj->Recycle();
    // ref_count is now 1 — still held

    obj->Recycle();
    // ref_count hit 0 → returned to pool

    auto* reused = Make(pool, 30);
    EXPECT_EQ(reused->value, 30);
    reused->Recycle();
}

TEST(ObjectPoolTest, SetRefCountAndAddRef) {
    ObjectPool<TestObject, NoSyncPolicy> pool(2);

    auto* obj = Make(pool, 10);
    obj->SetRefCount(2);
    obj->AddRef();  // now 3

    obj->Recycle();  // 2
    obj->Recycle();  // 1
    obj->Recycle();  // 0 → returned to pool

    auto* reused = Make(pool, 77);
    EXPECT_EQ(reused->value, 77);
    reused->Recycle();
}

TEST(ObjectPoolTest, ThreadSafety) {
    ObjectPool<ThreadSafeTestObject, ThreadSafePolicy> pool(1000);
    std::atomic<int> total_created{0};

    auto worker = [&pool, &total_created]() {
        for (int i = 0; i < 100; ++i) {
            auto* obj = Make(pool, i);
            ASSERT_NE(obj, nullptr);
            total_created.fetch_add(1, std::memory_order_relaxed);
            obj->Recycle();
        }
    };

    std::vector<std::thread> threads;
    for (int t = 0; t < 4; ++t) {
        threads.emplace_back(worker);
    }
    for (auto& t : threads) {
        t.join();
    }

    EXPECT_EQ(total_created.load(), 400);
}

TEST(ObjectPoolTest, ConcurrentCreateRecycleNeverHandsOutSameSlotTwice) {
    constexpr int kThreads = 8;
    constexpr int kIterations = 200'000;
    // Fewer slots than threads: maximal contention on the free-list head.
    constexpr int kSlots = kThreads - 2;
    ObjectPool<ThreadSafeTestObject, ThreadSafePolicy> pool(kSlots);

    std::array<ThreadSafeTestObject*, kSlots> slots{};
    for (auto& slot : slots) slot = pool.Create();
    for (auto* slot : slots) slot->Recycle();
    std::array<std::atomic<int>, kSlots> in_use{};
    auto slot_index = [&slots](const ThreadSafeTestObject* obj) {
        return static_cast<int>(std::find(slots.begin(), slots.end(), obj) - slots.begin());
    };

    std::atomic<int> collisions{0};
    auto worker = [&](int thread_idx) {
        for (int i = 0; i < kIterations; ++i) {
            ThreadSafeTestObject* obj = nullptr;
            while (!(obj = pool.TryCreate())) {
                std::this_thread::yield();
            }
            obj->value = thread_idx;
            int idx = slot_index(obj);
            if (idx == kSlots || in_use[idx].exchange(1, std::memory_order_acq_rel) != 0) {
                collisions.fetch_add(1, std::memory_order_relaxed);
            }
            if (obj->value != thread_idx) collisions.fetch_add(1, std::memory_order_relaxed);
            if (idx != kSlots) in_use[idx].store(0, std::memory_order_release);
            obj->Recycle();
        }
    };

    std::vector<std::thread> threads;
    for (int t = 0; t < kThreads; ++t) {
        threads.emplace_back(worker, t);
    }
    for (auto& t : threads) {
        t.join();
    }

    EXPECT_EQ(collisions.load(), 0);
    // Every slot must be back in the pool.
    std::vector<ThreadSafeTestObject*> all;
    for (int i = 0; i < kSlots; ++i) {
        all.push_back(pool.TryCreate());
        ASSERT_NE(all.back(), nullptr);
    }
    EXPECT_EQ(pool.TryCreate(), nullptr);
    for (auto* obj : all) obj->Recycle();
}

TEST(ObjectPoolTest, TryCreateReturnsNullWhenExhausted) {
    ObjectPool<TestObject, NoSyncPolicy> pool(1);
    auto* obj = pool.TryCreate();
    ASSERT_NE(obj, nullptr);
    EXPECT_EQ(pool.TryCreate(), nullptr);
    obj->Recycle();
    auto* again = pool.TryCreate();
    ASSERT_NE(again, nullptr);
    again->Recycle();
}

TEST(ObjectPoolTest, RecycledObjectKeepsItsAllocationsAndIsReset) {
    ObjectPool<BufferObject, NoSyncPolicy> pool(1);

    auto* obj = pool.Create();
    obj->buffer.resize(1000, 42);
    const int* data = obj->buffer.data();
    obj->Recycle();

    auto* reused = pool.Create();
    ASSERT_EQ(reused, obj);
    EXPECT_EQ(reused->reset_calls, 1);  // not called on first construction
    EXPECT_TRUE(reused->buffer.empty());
    EXPECT_GE(reused->buffer.capacity(), 1000u);
    reused->buffer.resize(1000);
    EXPECT_EQ(reused->buffer.data(), data);
    reused->Recycle();
}

TEST(ObjectPoolTest, RefCountIsResetOnReuse) {
    ObjectPool<TestObject, NoSyncPolicy> pool(1);
    auto* obj = pool.Create();
    obj->SetRefCount(2);
    obj->Recycle();
    obj->Recycle();  // 0 → back to pool

    auto* reused = pool.Create();
    ASSERT_EQ(reused, obj);
    reused->Recycle();  // single Recycle must suffice
    EXPECT_EQ(pool.TryCreate(), obj);
}

TEST(ObjectPoolTest, DestructorDestroysEveryConstructedObjectOnce) {
    g_destroyed = 0;
    {
        ObjectPool<CountingObject, NoSyncPolicy> pool(8);
        auto* a = pool.Create();
        auto* b = pool.Create();
        auto* c = pool.Create();
        a->Recycle();
        b->Recycle();
        (void)pool.Create();  // reuses b's slot, no new construction
        (void)c;
        EXPECT_EQ(g_destroyed, 0);  // Recycle no longer destroys
    }
    EXPECT_EQ(g_destroyed, 3);
}

TEST(ObjectPoolDeathTest, PoolExhaustion) {
    ObjectPool<TestObject, NoSyncPolicy> pool(1);
    auto* obj = Make(pool, 1);
    ASSERT_NE(obj, nullptr);

    EXPECT_DEATH(pool.Create(), "ObjectPool exhausted");

    obj->Recycle();
}

} // namespace
} // namespace voyahslam::memory
