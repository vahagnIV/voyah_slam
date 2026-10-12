#ifndef VOYAHSLAM_MEMORY_OBJECT_POOL_H
#define VOYAHSLAM_MEMORY_OBJECT_POOL_H

#include <array>
#include <atomic>
#include <bit>
#include <cassert>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <limits>
#include <memory>
#include <new>
#include <string>
#include <voyahslam/memory/sync_policy.h>

namespace voyahslam::memory {

namespace detail {
// Logs (warn, logger "ObjectPool") that `pool_name` grew from `old_capacity`
// to `new_capacity` objects. Defined in object_pool.cpp to keep spdlog out of
// this header.
void LogPoolGrowth(const char* pool_name, size_t old_capacity, size_t new_capacity,
                   uint64_t overflow_created);
} // namespace detail

// Pool of reusable objects that grows on overflow. Each slot's object is
// default-constructed the first time the slot is handed out and then kept alive
// across Recycle()/Create() cycles, so whatever it allocated (vectors, Eigen
// buffers) is reused instead of freed. On reuse the pool resets the reference
// count and calls `T::Reset()` if T has one. Objects are destroyed only when the
// pool is destroyed.
//
// Slots live in chunks. Chunk 0 holds the `initial_capacity` slots given to the
// constructor; when Create() finds no free slot it adds chunk k >= 1 of
// initial_capacity << (k - 1) slots, doubling the capacity, and logs one
// warning per growth (so a spike of n overflow objects logs O(log n) lines).
// Slot indices are global and contiguous across chunks: chunk k covers
// [initial << (k - 1), initial << k). Recycled overflow objects go back on the
// free list like any other, so the pool keeps its peak size and a later spike
// of the same height allocates nothing. TryCreate() never grows.
//
// The free-list is a stack of slot indices whose links live in per-chunk
// arrays, so a slot's memory is never read while another thread may be
// constructing an object in it, and slots are not touched (or committed by the
// OS) until first use. With ThreadSafePolicy the stack is a lock-free Treiber
// stack; the head packs (tag << 32 | index) and the tag is bumped on every
// operation to prevent ABA. Growth is lock-free too: a chunk is published by a
// CAS on its (null) chunk pointer and its slots are pushed as one chain; a
// creator that loses the race simply retries. A creator whose pop failed
// before another creator finished a growth retries instead of growing again
// (Grow() takes the growth count seen before the pop); only a creator that
// fails while another is between its chunk CAS and its push grows one more
// chunk (waiting there would not be lock-free). Chunks are freed only by the
// pool's destructor, so a stale index read by a losing CAS is always valid.
//
// Limits: the slot count stays below 2^32 - 1 (indices are 32-bit), so the
// largest capacity is the largest initial << k <= 2^32 - 1, between 2^31 and
// 2^32 - 1 slots; past it Create() aborts. In practice memory runs out first:
// a std::bad_alloc from a growth propagates out of Create() on the producer's
// thread (for the Tracker and Mapper, their worker threads).
template <typename T, typename SyncPolicy = NoSyncPolicy>
class ObjectPool {
public:
    explicit ObjectPool(size_t initial_capacity = 1'000'000, const char* name = "ObjectPool");
    ObjectPool(const ObjectPool&) = delete;
    ObjectPool& operator=(const ObjectPool&) = delete;
    ~ObjectPool();

    // When no slot is free the pool grows (see the class comment); throws
    // std::bad_alloc if the growth cannot be allocated.
    T* Create();
    // Returns nullptr if no slot is free; never grows the pool.
    T* TryCreate();
    void Recycle(T* obj);

    // Slots in the pool: the initial capacity plus every grown chunk.
    size_t capacity() const noexcept { return capacity_.load(std::memory_order_relaxed); }
    // Objects constructed in grown slots (beyond the initial capacity).
    uint64_t num_overflow_created() const noexcept {
        return num_overflow_created_.load(std::memory_order_relaxed);
    }

private:
    static constexpr uint32_t kNil = std::numeric_limits<uint32_t>::max();
    static constexpr size_t kAlign = alignof(T);
    static constexpr size_t kSize = sizeof(T);  // always a multiple of alignof(T)
    static constexpr size_t kMaxChunks = 33;

    static_assert(std::atomic<uint64_t>::is_always_lock_free);

    struct Chunk {
        // `storage` is allocated last, so a std::bad_alloc leaks nothing.
        Chunk(uint32_t begin_index, size_t count) :
            next(std::make_unique<std::atomic<uint32_t>[]>(count)),
            constructed(std::make_unique<bool[]>(count)),
            storage(static_cast<std::byte*>(::operator new(count * kSize, std::align_val_t(kAlign)))),
            begin(begin_index),
            size(count) {}
        ~Chunk() {
            for (size_t i = 0; i < size; ++i) {
                if (constructed[i]) {
                    reinterpret_cast<T*>(storage + i * kSize)->~T();
                }
            }
            ::operator delete(storage, std::align_val_t(kAlign));
        }
        Chunk(const Chunk&) = delete;
        Chunk& operator=(const Chunk&) = delete;

        std::unique_ptr<std::atomic<uint32_t>[]> next;
        // Written only by the thread that currently owns the slot; read in ~Chunk.
        std::unique_ptr<bool[]> constructed;
        std::byte* storage;
        uint32_t begin;  // global index of the first slot
        size_t size;
    };

    std::array<std::atomic<Chunk*>, kMaxChunks> chunks_{};
    std::atomic<uint64_t> head_{0};
    std::atomic<size_t> capacity_{0};
    std::atomic<uint64_t> num_overflow_created_{0};
    // Completed growths (chunk pushed); Grow() skips if it changed since a pop.
    std::atomic<uint64_t> num_growths_{0};
    size_t initial_capacity_ = 0;
    std::string name_;

    static uint64_t Pack(uint32_t tag, uint32_t index) {
        return (static_cast<uint64_t>(tag) << 32) | index;
    }
    static uint32_t Tag(uint64_t head) { return static_cast<uint32_t>(head >> 32); }
    static uint32_t Index(uint64_t head) { return static_cast<uint32_t>(head); }

    size_t ChunkOf(uint32_t index) const noexcept {
        return index < initial_capacity_
                   ? 0
                   : static_cast<size_t>(std::bit_width(index / initial_capacity_));
    }
    Chunk* ChunkAt(size_t k) const noexcept { return chunks_[k].load(std::memory_order_acquire); }

    uint32_t PopSlot();
    // Pushes the chain first -> ... -> last (already linked) onto the free list.
    void PushChain(uint32_t first, uint32_t last);
    // Adds the next chunk, unless a growth completed since `seen_growths`
    // (the caller then retries its pop).
    void Grow(uint64_t seen_growths);
    std::atomic<uint32_t>& NextOf(uint32_t index) const;
    T* SlotAt(uint32_t index) const;
    bool& ConstructedAt(uint32_t index) const;
    // kNil if `p` is not the start of a slot of this pool.
    uint32_t IndexOf(const void* p) const noexcept;
};

template <typename T, typename SyncPolicy>
ObjectPool<T, SyncPolicy>::ObjectPool(size_t initial_capacity, const char* name) :
    initial_capacity_(initial_capacity),
    name_(name) {
    assert(initial_capacity > 0 && initial_capacity < kNil && "ObjectPool size out of range");
    auto* chunk = new Chunk(0, initial_capacity);
    for (size_t i = 0; i < initial_capacity; ++i) {
        chunk->next[i].store(i + 1 < initial_capacity ? static_cast<uint32_t>(i + 1) : kNil,
                             std::memory_order_relaxed);
    }
    chunks_[0].store(chunk, std::memory_order_relaxed);
    capacity_.store(initial_capacity, std::memory_order_relaxed);
    head_.store(Pack(0, 0), std::memory_order_release);
}

template <typename T, typename SyncPolicy>
ObjectPool<T, SyncPolicy>::~ObjectPool() {
    for (auto& chunk : chunks_) {
        delete chunk.load(std::memory_order_acquire);
    }
}

template <typename T, typename SyncPolicy>
T* ObjectPool<T, SyncPolicy>::Create() {
    while (true) {
        const uint64_t seen_growths = num_growths_.load(std::memory_order_acquire);
        if (T* obj = TryCreate()) {
            return obj;
        }
        Grow(seen_growths);
    }
}

template <typename T, typename SyncPolicy>
T* ObjectPool<T, SyncPolicy>::TryCreate() {
    uint32_t index = PopSlot();
    if (index == kNil) {
        return nullptr;
    }
    T* obj = SlotAt(index);
    bool& constructed = ConstructedAt(index);
    if (!constructed) {
        new (obj) T();
        obj->object_pool_ = this;
        constructed = true;
        if (index >= initial_capacity_) {
            num_overflow_created_.fetch_add(1, std::memory_order_relaxed);
        }
    } else {
        obj->ref_count_.store(1, std::memory_order_relaxed);
        if constexpr (requires { obj->Reset(); }) {
            obj->Reset();
        }
    }
    return obj;
}

template <typename T, typename SyncPolicy>
void ObjectPool<T, SyncPolicy>::Recycle(T* obj) {
    if (!obj) return;
    uint32_t index = IndexOf(obj);
    assert(index != kNil && "Recycling pointer not from this pool");
    PushChain(index, index);
}

template <typename T, typename SyncPolicy>
void ObjectPool<T, SyncPolicy>::Grow(uint64_t seen_growths) {
    if (num_growths_.load(std::memory_order_acquire) != seen_growths) {
        return;  // slots were added after the failed pop: retry it
    }
    for (size_t k = 1; k < kMaxChunks; ++k) {
        if (ChunkAt(k) != nullptr) {
            continue;
        }
        const size_t begin = initial_capacity_ << (k - 1);
        const size_t count = begin;  // chunk k doubles the capacity
        if (begin + count > kNil) {
            break;
        }
        auto chunk = std::make_unique<Chunk>(static_cast<uint32_t>(begin), count);
        for (size_t i = 0; i + 1 < count; ++i) {
            chunk->next[i].store(static_cast<uint32_t>(begin + i + 1), std::memory_order_relaxed);
        }
        Chunk* expected = nullptr;
        // Release: the chunk's contents happen-before any thread that finds the pointer.
        if (!chunks_[k].compare_exchange_strong(expected, chunk.get(), std::memory_order_acq_rel,
                                                std::memory_order_acquire)) {
            return;  // another creator grew the pool; the caller retries
        }
        chunk.release();
        const size_t old_capacity = capacity_.load(std::memory_order_relaxed);
        capacity_.fetch_add(count, std::memory_order_relaxed);
        PushChain(static_cast<uint32_t>(begin), static_cast<uint32_t>(begin + count - 1));
        num_growths_.fetch_add(1, std::memory_order_release);
        detail::LogPoolGrowth(name_.c_str(), old_capacity, old_capacity + count,
                              num_overflow_created_.load(std::memory_order_relaxed));
        return;
    }
    // The next doubling would pass 2^32 - 1 slots (indices are 32-bit): not a
    // backlog any more but a leak.
    std::fputs("ObjectPool: slot index space exhausted\n", stderr);
    std::abort();
}

template <typename T, typename SyncPolicy>
uint32_t ObjectPool<T, SyncPolicy>::PopSlot() {
    uint64_t head = head_.load(std::memory_order_acquire);
    if constexpr (SyncPolicy::kThreadSafe) {
        while (true) {
            uint32_t index = Index(head);
            if (index == kNil) return kNil;
            uint32_t next = NextOf(index).load(std::memory_order_relaxed);
            if (head_.compare_exchange_weak(head, Pack(Tag(head) + 1, next),
                                            std::memory_order_acquire,
                                            std::memory_order_acquire)) {
                return index;
            }
        }
    } else {
        uint32_t index = Index(head);
        if (index == kNil) return kNil;
        head_.store(Pack(Tag(head) + 1, NextOf(index).load(std::memory_order_relaxed)),
                    std::memory_order_relaxed);
        return index;
    }
}

template <typename T, typename SyncPolicy>
void ObjectPool<T, SyncPolicy>::PushChain(uint32_t first, uint32_t last) {
    uint64_t head = head_.load(std::memory_order_relaxed);
    std::atomic<uint32_t>& last_next = NextOf(last);
    if constexpr (SyncPolicy::kThreadSafe) {
        do {
            last_next.store(Index(head), std::memory_order_relaxed);
        } while (!head_.compare_exchange_weak(head, Pack(Tag(head) + 1, first),
                                              std::memory_order_release,
                                              std::memory_order_relaxed));
    } else {
        last_next.store(Index(head), std::memory_order_relaxed);
        head_.store(Pack(Tag(head) + 1, first), std::memory_order_relaxed);
    }
}

template <typename T, typename SyncPolicy>
std::atomic<uint32_t>& ObjectPool<T, SyncPolicy>::NextOf(uint32_t index) const {
    const Chunk* chunk = ChunkAt(ChunkOf(index));
    return chunk->next[index - chunk->begin];
}

template <typename T, typename SyncPolicy>
T* ObjectPool<T, SyncPolicy>::SlotAt(uint32_t index) const {
    const Chunk* chunk = ChunkAt(ChunkOf(index));
    return reinterpret_cast<T*>(chunk->storage + static_cast<size_t>(index - chunk->begin) * kSize);
}

template <typename T, typename SyncPolicy>
bool& ObjectPool<T, SyncPolicy>::ConstructedAt(uint32_t index) const {
    const Chunk* chunk = ChunkAt(ChunkOf(index));
    return chunk->constructed[index - chunk->begin];
}

template <typename T, typename SyncPolicy>
uint32_t ObjectPool<T, SyncPolicy>::IndexOf(const void* p) const noexcept {
    const auto addr = reinterpret_cast<std::uintptr_t>(p);
    for (size_t k = 0; k < kMaxChunks; ++k) {
        const Chunk* chunk = ChunkAt(k);
        if (!chunk) break;
        const auto base = reinterpret_cast<std::uintptr_t>(chunk->storage);
        if (addr >= base && addr < base + chunk->size * kSize) {
            if ((addr - base) % kSize != 0) return kNil;
            return chunk->begin + static_cast<uint32_t>((addr - base) / kSize);
        }
    }
    return kNil;
}

} // namespace voyahslam::memory

#endif // VOYAHSLAM_MEMORY_OBJECT_POOL_H
