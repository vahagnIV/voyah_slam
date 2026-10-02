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
    explicit TestObject(int v = 0) : value(v) {}
};

struct ThreadSafeTestObject : Recyclable<ThreadSafeTestObject, ThreadSafePolicy> {
    int value = 0;
    explicit ThreadSafeTestObject(int v = 0) : value(v) {}
};

TEST(ObjectPoolTest, CreateAndRecycle) {
    ObjectPool<TestObject, NoSyncPolicy> pool(4);

    auto* obj = pool.Create(42);
    ASSERT_NE(obj, nullptr);
    EXPECT_EQ(obj->value, 42);

    auto* addr = obj;
    obj->Recycle();

    auto* obj2 = pool.Create(99);
    EXPECT_EQ(obj2, addr);
    EXPECT_EQ(obj2->value, 99);
    obj2->Recycle();
}

TEST(ObjectPoolTest, RefCounting) {
    ObjectPool<TestObject, NoSyncPolicy> pool(2);

    auto* obj = pool.Create(10);
    obj->SetRefCount(3);

    obj->Recycle();
    // ref_count is now 2 — pool should NOT have reclaimed
    auto* other = pool.Create(20);
    EXPECT_NE(other, obj);  // obj still held, got a different slot
    other->Recycle();

    obj->Recycle();
    // ref_count is now 1 — still held

    obj->Recycle();
    // ref_count hit 0 → returned to pool

    auto* reused = pool.Create(30);
    EXPECT_EQ(reused->value, 30);
    reused->Recycle();
}

TEST(ObjectPoolTest, SetRefCountAndAddRef) {
    ObjectPool<TestObject, NoSyncPolicy> pool(2);

    auto* obj = pool.Create(10);
    obj->SetRefCount(2);
    obj->AddRef();  // now 3

    obj->Recycle();  // 2
    obj->Recycle();  // 1
    obj->Recycle();  // 0 → returned to pool

    auto* reused = pool.Create(77);
    EXPECT_EQ(reused->value, 77);
    reused->Recycle();
}

TEST(ObjectPoolTest, ThreadSafety) {
    ObjectPool<ThreadSafeTestObject, ThreadSafePolicy> pool(1000);
    std::atomic<int> total_created{0};

    auto worker = [&pool, &total_created]() {
        for (int i = 0; i < 100; ++i) {
            auto* obj = pool.Create(i);
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
            while (!(obj = pool.TryCreate(thread_idx))) {
                std::this_thread::yield();
            }
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
        all.push_back(pool.TryCreate(i));
        ASSERT_NE(all.back(), nullptr);
    }
    EXPECT_EQ(pool.TryCreate(0), nullptr);
    for (auto* obj : all) obj->Recycle();
}

TEST(ObjectPoolTest, TryCreateReturnsNullWhenExhausted) {
    ObjectPool<TestObject, NoSyncPolicy> pool(1);
    auto* obj = pool.TryCreate(1);
    ASSERT_NE(obj, nullptr);
    EXPECT_EQ(pool.TryCreate(2), nullptr);
    obj->Recycle();
    auto* again = pool.TryCreate(3);
    ASSERT_NE(again, nullptr);
    again->Recycle();
}

TEST(ObjectPoolDeathTest, PoolExhaustion) {
    ObjectPool<TestObject, NoSyncPolicy> pool(1);
    auto* obj = pool.Create(1);
    ASSERT_NE(obj, nullptr);

    EXPECT_DEATH(pool.Create(2), "ObjectPool exhausted");

    obj->Recycle();
}

} // namespace
} // namespace voyahslam::memory
