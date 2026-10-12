#include <voyahslam/memory/object_pool.h>
#include <voyahslam/memory/recyclable.h>
#include <voyahslam/logging/logger.h>

#include <spdlog/sinks/ringbuffer_sink.h>

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

// An exhausted pool grows instead of aborting (was a death test until 0.3.0).
TEST(ObjectPoolTest, CreateBeyondCapacityGrowsInsteadOfAborting) {
    ObjectPool<TestObject, NoSyncPolicy> pool(2);
    EXPECT_EQ(pool.capacity(), 2u);
    EXPECT_EQ(pool.num_overflow_created(), 0u);
    std::vector<TestObject*> objs;
    for (int i = 0; i < 7; ++i) {
        objs.push_back(Make(pool, i));
        ASSERT_NE(objs.back(), nullptr);
    }
    // Every object is distinct and keeps its value.
    for (int i = 0; i < 7; ++i) {
        EXPECT_EQ(objs[i]->value, i);
        for (int j = i + 1; j < 7; ++j) EXPECT_NE(objs[i], objs[j]);
    }
    EXPECT_EQ(pool.num_overflow_created(), 5u);
    EXPECT_GE(pool.capacity(), 7u);
    for (auto* obj : objs) obj->Recycle();
}

// Overflow objects go back on the free list: the pool keeps its peak size, so a
// second spike of the same height constructs nothing new.
TEST(ObjectPoolTest, OverflowObjectsAreReusedAfterRecycle) {
    g_destroyed = 0;
    {
        ObjectPool<CountingObject, NoSyncPolicy> pool(2);
        std::vector<CountingObject*> first;
        for (int i = 0; i < 6; ++i) first.push_back(pool.Create());
        EXPECT_EQ(pool.num_overflow_created(), 4u);
        const size_t peak_capacity = pool.capacity();
        for (auto* obj : first) obj->Recycle();
        EXPECT_EQ(g_destroyed, 0);  // overflow objects are kept, not freed

        std::vector<CountingObject*> second;
        for (int i = 0; i < 6; ++i) second.push_back(pool.Create());
        EXPECT_EQ(pool.num_overflow_created(), 4u);  // nothing new constructed
        EXPECT_EQ(pool.capacity(), peak_capacity);
        std::sort(first.begin(), first.end());
        std::sort(second.begin(), second.end());
        EXPECT_EQ(first, second);
        for (auto* obj : second) obj->Recycle();
    }
    EXPECT_EQ(g_destroyed, 6);  // the destructor frees the overflow objects too
}

// TryCreate never grows the pool: null on exhaustion, and no overflow counted.
TEST(ObjectPoolTest, TryCreateNeverGrows) {
    ObjectPool<TestObject, NoSyncPolicy> pool(2);
    auto* a = pool.TryCreate();
    auto* b = pool.TryCreate();
    ASSERT_NE(a, nullptr);
    ASSERT_NE(b, nullptr);
    EXPECT_EQ(pool.TryCreate(), nullptr);
    EXPECT_EQ(pool.capacity(), 2u);
    EXPECT_EQ(pool.num_overflow_created(), 0u);
    a->Recycle();
    b->Recycle();
}

// After a Create() spike the grown slots are free slots, which TryCreate may
// hand out; once they are all in use it returns null again.
TEST(ObjectPoolTest, TryCreateUsesGrownSlotsButStillReturnsNullWhenAllInUse) {
    ObjectPool<TestObject, NoSyncPolicy> pool(1);
    auto* a = pool.Create();
    auto* b = pool.Create();  // grows
    b->Recycle();
    auto* c = pool.TryCreate();
    EXPECT_EQ(c, b);
    std::vector<TestObject*> rest;
    while (auto* obj = pool.TryCreate()) rest.push_back(obj);
    EXPECT_EQ(pool.TryCreate(), nullptr);
    EXPECT_EQ(rest.size() + 2, pool.capacity());
    a->Recycle();
    c->Recycle();
    for (auto* obj : rest) obj->Recycle();
}

// The growth warning is logged at the first overflow and then only when the
// pool grows again (geometrically), never once per event.
TEST(ObjectPoolTest, OverflowWarningIsRateLimited) {
    auto sink = std::make_shared<spdlog::sinks::ringbuffer_sink_mt>(64);
    auto logger = Logger::CreateLogger("ObjectPool");
    logger->sinks().push_back(sink);
    {
        ObjectPool<TestObject, NoSyncPolicy> pool(4, "WarnTestPool");
        std::vector<TestObject*> objs;
        for (int i = 0; i < 4; ++i) objs.push_back(pool.Create());
        EXPECT_TRUE(sink->last_formatted(64).empty());  // no overflow yet
        for (int i = 0; i < 1000; ++i) objs.push_back(pool.Create());
        EXPECT_EQ(pool.num_overflow_created(), 1000u);
        auto lines = sink->last_raw(64);
        // 4 -> 8 -> 16 -> ... -> 1024: 8 growths for 1000 overflow objects.
        EXPECT_EQ(lines.size(), 8u);
        for (const auto& msg : lines) {
            EXPECT_EQ(msg.level, spdlog::level::warn);
            EXPECT_NE(std::string(msg.payload.begin(), msg.payload.end()).find("WarnTestPool"),
                      std::string::npos);
        }
        for (auto* obj : objs) obj->Recycle();
    }
    logger->sinks().pop_back();
}

// A producer creates far beyond capacity while consumers on other threads
// recycle concurrently. Run under TSan and ASan (DESIGN "Object pools").
TEST(ObjectPoolTest, ConcurrentOverflowCreateAndRecycleIsSafe) {
    constexpr int kConsumers = 3;
    constexpr int kEvents = 200'000;
    constexpr size_t kSlots = 4;
    ObjectPool<ThreadSafeTestObject, ThreadSafePolicy> pool(kSlots);

    // Single-producer / single-consumer rings, one per consumer.
    constexpr size_t kRing = 4096;
    struct Ring {
        std::array<std::atomic<ThreadSafeTestObject*>, kRing> cells{};
        std::atomic<size_t> head{0};
        std::atomic<size_t> tail{0};
    };
    std::vector<Ring> rings(kConsumers);
    std::atomic<bool> done{false};
    std::atomic<int> bad{0};

    auto consumer = [&](int c) {
        Ring& ring = rings[c];
        size_t tail = 0;
        while (true) {
            size_t head = ring.head.load(std::memory_order_acquire);
            if (tail == head) {
                if (done.load(std::memory_order_acquire) &&
                    tail == ring.head.load(std::memory_order_acquire)) break;
                std::this_thread::yield();
                continue;
            }
            auto* obj = ring.cells[tail % kRing].load(std::memory_order_relaxed);
            if (obj->value < 0) bad.fetch_add(1, std::memory_order_relaxed);
            obj->value = -1;  // write to the object before recycling it
            ++tail;
            ring.tail.store(tail, std::memory_order_release);
            obj->Recycle();
        }
    };
    std::vector<std::thread> threads;
    for (int c = 0; c < kConsumers; ++c) threads.emplace_back(consumer, c);

    std::vector<size_t> heads(kConsumers, 0);
    for (int i = 0; i < kEvents; ++i) {
        int c = i % kConsumers;
        Ring& ring = rings[c];
        while (heads[c] - ring.tail.load(std::memory_order_acquire) >= kRing) {
            std::this_thread::yield();
        }
        auto* obj = pool.Create();
        obj->value = i;
        ring.cells[heads[c] % kRing].store(obj, std::memory_order_relaxed);
        ring.head.store(++heads[c], std::memory_order_release);
    }
    done.store(true, std::memory_order_release);
    for (auto& t : threads) t.join();

    EXPECT_EQ(bad.load(), 0);
    EXPECT_GT(pool.num_overflow_created(), 0u);
    // Every object is back: exactly capacity() objects can be taken without growing.
    std::vector<ThreadSafeTestObject*> all;
    while (auto* obj = pool.TryCreate()) all.push_back(obj);
    EXPECT_EQ(all.size(), pool.capacity());
    std::sort(all.begin(), all.end());
    EXPECT_EQ(std::adjacent_find(all.begin(), all.end()), all.end());
    for (auto* obj : all) obj->Recycle();
}

// Several threads Create() concurrently on an exhausted pool, so they race to grow it.
TEST(ObjectPoolTest, ConcurrentCreatorsGrowSafely) {
    constexpr int kThreads = 6;
    constexpr int kPerThread = 2000;
    ObjectPool<ThreadSafeTestObject, ThreadSafePolicy> pool(2);
    std::vector<std::vector<ThreadSafeTestObject*>> held(kThreads);
    std::vector<std::thread> threads;
    for (int t = 0; t < kThreads; ++t) {
        threads.emplace_back([&, t] {
            for (int i = 0; i < kPerThread; ++i) {
                auto* obj = pool.Create();
                obj->value = t;
                held[t].push_back(obj);
                if (i % 3 == 0) {
                    held[t].back()->Recycle();
                    held[t].pop_back();
                }
            }
        });
    }
    for (auto& t : threads) t.join();
    std::vector<ThreadSafeTestObject*> all;
    for (int t = 0; t < kThreads; ++t) {
        for (auto* obj : held[t]) {
            EXPECT_EQ(obj->value, t);
            all.push_back(obj);
        }
    }
    std::sort(all.begin(), all.end());
    EXPECT_EQ(std::adjacent_find(all.begin(), all.end()), all.end());
    EXPECT_LE(all.size(), pool.capacity());
    for (auto* obj : all) obj->Recycle();
}

} // namespace
} // namespace voyahslam::memory
