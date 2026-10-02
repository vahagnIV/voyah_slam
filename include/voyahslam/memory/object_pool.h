#ifndef VOYAHSLAM_MEMORY_OBJECT_POOL_H
#define VOYAHSLAM_MEMORY_OBJECT_POOL_H

#include <atomic>
#include <cassert>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <memory>
#include <new>
#include <utility>
#include <voyahslam/memory/sync_policy.h>

namespace voyahslam::memory {

// Fixed-capacity slab allocator. The free-list is a stack of slot indices whose
// links live in a separate array, so a slot's memory is never read while another
// thread may be constructing an object in it, and slots are not touched (or
// committed by the OS) until first use. With ThreadSafePolicy the stack is a
// lock-free Treiber stack; the head packs (tag << 32 | index) and the tag is
// bumped on every operation to prevent ABA.
template <typename T, typename SyncPolicy = NoSyncPolicy>
class ObjectPool {
public:
    explicit ObjectPool(size_t max_count = 1'000'000);
    ObjectPool(const ObjectPool&) = delete;
    ObjectPool& operator=(const ObjectPool&) = delete;
    ~ObjectPool();

    // Asserts if the pool is exhausted.
    template <typename... Args>
    T* Create(Args&&... args);
    // Returns nullptr if the pool is exhausted.
    template <typename... Args>
    T* TryCreate(Args&&... args);
    void Recycle(T* obj);

private:
    static constexpr uint32_t kNil = std::numeric_limits<uint32_t>::max();
    static constexpr size_t kAlign = alignof(T);
    static constexpr size_t kSize = sizeof(T);  // always a multiple of alignof(T)

    static_assert(std::atomic<uint64_t>::is_always_lock_free);

    std::byte* storage_ = nullptr;
    std::unique_ptr<std::atomic<uint32_t>[]> next_;
    std::atomic<uint64_t> head_{0};
    size_t max_count_ = 0;

    static uint64_t Pack(uint32_t tag, uint32_t index) {
        return (static_cast<uint64_t>(tag) << 32) | index;
    }
    static uint32_t Tag(uint64_t head) { return static_cast<uint32_t>(head >> 32); }
    static uint32_t Index(uint64_t head) { return static_cast<uint32_t>(head); }

    uint32_t PopSlot();
    void PushSlot(uint32_t index);
    T* SlotAt(uint32_t index) const;
    bool BelongsToPool(const void* p) const noexcept;
};

template <typename T, typename SyncPolicy>
ObjectPool<T, SyncPolicy>::ObjectPool(size_t max_count) :
    next_(std::make_unique<std::atomic<uint32_t>[]>(max_count)),
    max_count_(max_count) {
    assert(max_count > 0 && max_count < kNil && "ObjectPool size out of range");
    storage_ = static_cast<std::byte*>(::operator new(max_count_ * kSize, std::align_val_t(kAlign)));
    for (size_t i = 0; i < max_count_; ++i) {
        next_[i].store(i + 1 < max_count_ ? static_cast<uint32_t>(i + 1) : kNil,
                       std::memory_order_relaxed);
    }
    head_.store(Pack(0, 0), std::memory_order_release);
}

template <typename T, typename SyncPolicy>
ObjectPool<T, SyncPolicy>::~ObjectPool() {
    ::operator delete(storage_, std::align_val_t(kAlign));
}

template <typename T, typename SyncPolicy>
template <typename... Args>
T* ObjectPool<T, SyncPolicy>::Create(Args&&... args) {
    T* obj = TryCreate(std::forward<Args>(args)...);
    assert(obj && "ObjectPool exhausted");
    return obj;
}

template <typename T, typename SyncPolicy>
template <typename... Args>
T* ObjectPool<T, SyncPolicy>::TryCreate(Args&&... args) {
    uint32_t index = PopSlot();
    if (index == kNil) {
        return nullptr;
    }
    T* obj = new (SlotAt(index)) T(std::forward<Args>(args)...);
    obj->object_pool_ = this;
    return obj;
}

template <typename T, typename SyncPolicy>
void ObjectPool<T, SyncPolicy>::Recycle(T* obj) {
    if (!obj) return;
    assert(BelongsToPool(obj) && "Recycling pointer not from this pool");
    obj->~T();
    PushSlot(static_cast<uint32_t>((reinterpret_cast<std::byte*>(obj) - storage_) / kSize));
}

template <typename T, typename SyncPolicy>
uint32_t ObjectPool<T, SyncPolicy>::PopSlot() {
    uint64_t head = head_.load(std::memory_order_acquire);
    if constexpr (SyncPolicy::kThreadSafe) {
        while (true) {
            uint32_t index = Index(head);
            if (index == kNil) return kNil;
            uint32_t next = next_[index].load(std::memory_order_relaxed);
            if (head_.compare_exchange_weak(head, Pack(Tag(head) + 1, next),
                                            std::memory_order_acquire,
                                            std::memory_order_acquire)) {
                return index;
            }
        }
    } else {
        uint32_t index = Index(head);
        if (index == kNil) return kNil;
        head_.store(Pack(Tag(head) + 1, next_[index].load(std::memory_order_relaxed)),
                    std::memory_order_relaxed);
        return index;
    }
}

template <typename T, typename SyncPolicy>
void ObjectPool<T, SyncPolicy>::PushSlot(uint32_t index) {
    uint64_t head = head_.load(std::memory_order_relaxed);
    if constexpr (SyncPolicy::kThreadSafe) {
        do {
            next_[index].store(Index(head), std::memory_order_relaxed);
        } while (!head_.compare_exchange_weak(head, Pack(Tag(head) + 1, index),
                                              std::memory_order_release,
                                              std::memory_order_relaxed));
    } else {
        next_[index].store(Index(head), std::memory_order_relaxed);
        head_.store(Pack(Tag(head) + 1, index), std::memory_order_relaxed);
    }
}

template <typename T, typename SyncPolicy>
T* ObjectPool<T, SyncPolicy>::SlotAt(uint32_t index) const {
    return reinterpret_cast<T*>(storage_ + static_cast<size_t>(index) * kSize);
}

template <typename T, typename SyncPolicy>
bool ObjectPool<T, SyncPolicy>::BelongsToPool(const void* p) const noexcept {
    auto addr = reinterpret_cast<std::uintptr_t>(p);
    auto base = reinterpret_cast<std::uintptr_t>(storage_);
    return addr >= base && addr < base + max_count_ * kSize && (addr - base) % kSize == 0;
}

} // namespace voyahslam::memory

#endif // VOYAHSLAM_MEMORY_OBJECT_POOL_H
