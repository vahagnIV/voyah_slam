#ifndef VOYAHSLAM_MEMORY_OBJECT_POOL_H
#define VOYAHSLAM_MEMORY_OBJECT_POOL_H

#include <cassert>
#include <cstddef>
#include <cstdint>
#include <new>
#include <type_traits>
#include <utility>
#include <voyahslam/memory/sync_policy.h>

namespace voyahslam::memory {

template <typename T, typename SyncPolicy = NoSyncPolicy>
class ObjectPool : private SyncPolicy {
public:
    explicit ObjectPool(size_t max_count = 1'000'000);
    ObjectPool(const ObjectPool&) = delete;
    ObjectPool& operator=(const ObjectPool&) = delete;
    ~ObjectPool();

    template <typename... Args>
    T* Create(Args&&... args);
    void Recycle(T* obj);

private:
    struct FreeNode { FreeNode* next; };
    static constexpr size_t kAlign = (alignof(T) > alignof(FreeNode)) ? alignof(T) : alignof(FreeNode);
    static constexpr size_t kSize  = (sizeof(T)  > sizeof(FreeNode))  ? sizeof(T)  : sizeof(FreeNode);

    std::byte* storage_ = nullptr;
    FreeNode* head_ = nullptr;
    size_t max_count_ = 0;

    void BuildFreeList() noexcept;
    bool BelongsToPool(const void* p) const noexcept;
};

template <typename T, typename SyncPolicy>
ObjectPool<T, SyncPolicy>::ObjectPool(size_t max_count)
        : max_count_(max_count) {
    static_assert(std::is_trivially_destructible_v<FreeNode>, "sanity");
    storage_ = static_cast<std::byte*>(
            ::operator new(max_count_ * kSize, std::align_val_t(kAlign))
    );
    BuildFreeList();
}

template <typename T, typename SyncPolicy>
ObjectPool<T, SyncPolicy>::~ObjectPool() {
    ::operator delete(storage_, std::align_val_t(kAlign));
}

template <typename T, typename SyncPolicy>
template <typename... Args>
T* ObjectPool<T, SyncPolicy>::Create(Args&&... args) {
    typename SyncPolicy::Lock lock(*this);

    if (!head_) {
        assert(!"ObjectPool exhausted");
        return nullptr;
    }

    FreeNode* node = head_;
    head_ = head_->next;

    T* obj = reinterpret_cast<T*>(node);
    new (obj) T(std::forward<Args>(args)...);
    obj->object_pool_ = this;
    return obj;
}

template <typename T, typename SyncPolicy>
void ObjectPool<T, SyncPolicy>::Recycle(T* obj) {
    if (!obj) return;
    assert(BelongsToPool(obj) && "Recycling pointer not from this pool");

    typename SyncPolicy::Lock lock(*this);
    obj->~T();

    auto* node = reinterpret_cast<FreeNode*>(obj);
    node->next = head_;
    head_ = node;
}

template <typename T, typename SyncPolicy>
void ObjectPool<T, SyncPolicy>::BuildFreeList() noexcept {
    head_ = nullptr;
    for (size_t i = 0; i < max_count_; ++i) {
        auto* node = reinterpret_cast<FreeNode*>(storage_ + i * kSize);
        node->next = head_;
        head_ = node;
    }
}

template <typename T, typename SyncPolicy>
bool ObjectPool<T, SyncPolicy>::BelongsToPool(const void* p) const noexcept {
    auto addr = reinterpret_cast<std::uintptr_t>(p);
    auto base = reinterpret_cast<std::uintptr_t>(storage_);
    return addr >= base && addr < base + max_count_ * kSize;
}

} // namespace voyahslam::memory

#endif // VOYAHSLAM_MEMORY_OBJECT_POOL_H
