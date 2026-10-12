#include <voyahslam/memory/object_pool.h>

#include <voyahslam/logging/logger.h>

namespace voyahslam::memory::detail {

void LogPoolGrowth(const char* pool_name, size_t old_capacity, size_t new_capacity,
                   uint64_t overflow_created) {
    static const auto log = Logger::CreateLogger("ObjectPool");
    log->warn("{} pool exhausted at {} objects, grew to {} ({} overflow objects created before this growth): "
              "a consumer is falling behind; raise the pool size in the options if this "
              "backlog is expected",
              pool_name, old_capacity, new_capacity, overflow_created);
}

} // namespace voyahslam::memory::detail
