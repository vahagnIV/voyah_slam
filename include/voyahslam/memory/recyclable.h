#ifndef VOYAHSLAM_MEMORY_RECYCLABLE_H
#define VOYAHSLAM_MEMORY_RECYCLABLE_H

#include <atomic>
#include <cassert>
#include <voyahslam/memory/sync_policy.h>

namespace voyahslam::memory {

template <typename T, typename SyncPolicy>
class ObjectPool;

template <typename T, typename SyncPolicy = NoSyncPolicy>
class Recyclable {
public:
    ~Recyclable() = default;
    void AddRef();
    void Recycle();
    void SetRefCount(int n);

protected:
    Recyclable() = default;

private:
    friend class ObjectPool<T, SyncPolicy>;
    ObjectPool<T, SyncPolicy>* object_pool_ = nullptr;
    std::atomic_int ref_count_{1};
};

template <typename T, typename SyncPolicy>
void Recyclable<T, SyncPolicy>::AddRef() {
    ref_count_.fetch_add(1, std::memory_order_relaxed);
}

template <typename T, typename SyncPolicy>
void Recyclable<T, SyncPolicy>::Recycle() {
    if (ref_count_.fetch_add(-1, std::memory_order_relaxed) == 1) {
        assert(object_pool_ && "Recycle() called on an object not owned by a pool");
        object_pool_->Recycle(static_cast<T*>(this));
    }
}

template <typename T, typename SyncPolicy>
void Recyclable<T, SyncPolicy>::SetRefCount(int n) {
    ref_count_.store(n, std::memory_order_relaxed);
}

} // namespace voyahslam::memory

#endif // VOYAHSLAM_MEMORY_RECYCLABLE_H
