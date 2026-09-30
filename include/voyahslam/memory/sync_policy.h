#ifndef VOYAHSLAM_MEMORY_SYNC_POLICY_H
#define VOYAHSLAM_MEMORY_SYNC_POLICY_H

#include <mutex>

namespace voyahslam::memory {

struct ThreadSafePolicy {
    std::mutex mtx;
    struct Lock {
        std::lock_guard<std::mutex> lg;
        explicit Lock(ThreadSafePolicy& p) : lg(p.mtx) {}
    };
};

struct NoSyncPolicy {
    struct Lock { explicit Lock(NoSyncPolicy&) {} };
};

} // namespace voyahslam::memory

#endif // VOYAHSLAM_MEMORY_SYNC_POLICY_H
