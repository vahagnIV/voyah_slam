#ifndef VOYAHSLAM_MEMORY_SYNC_POLICY_H
#define VOYAHSLAM_MEMORY_SYNC_POLICY_H

namespace voyahslam::memory {

// Selects how ObjectPool synchronizes its free-list. Neither policy takes a lock.

// Create() and Recycle() may run concurrently on any threads (lock-free CAS).
struct ThreadSafePolicy {
    static constexpr bool kThreadSafe = true;
};

// Create() and Recycle() are only ever called from one thread at a time.
struct NoSyncPolicy {
    static constexpr bool kThreadSafe = false;
};

} // namespace voyahslam::memory

#endif // VOYAHSLAM_MEMORY_SYNC_POLICY_H
