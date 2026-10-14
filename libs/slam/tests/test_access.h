#ifndef VOYAHSLAM_TESTS_TEST_ACCESS_H
#define VOYAHSLAM_TESTS_TEST_ACCESS_H

// Test access to the components' thread-affine state (DESIGN section 7, D-14,
// D-26). Each component keeps its replica (and, for the LoopCloser, its
// worker-written statistics) private, because reading it while the worker runs
// is a data race; the component befriends the struct named here, defined once
// per test binary in this header. Tests that drive ProcessPending()
// synchronously (never concurrently with a running worker) read through it.

#include <voyahslam/tracking/atlas_replica.h>
#include <voyahslam/tracking/map_replica.h>
#include <voyahslam/tracking/tracker.h>

#include <cstdint>
#include <vector>

namespace voyahslam::tracking {

struct TrackerTestAccess {
    template <typename TFeatureExtractor>
    static const tracking::AtlasReplica<TFeatureExtractor>& Atlas(const Tracker<TFeatureExtractor>& tracker) {
        return tracker.atlas();
    }
    template <typename TFeatureExtractor>
    static const tracking::MapReplica<TFeatureExtractor>& MapReplica(const Tracker<TFeatureExtractor>& tracker) {
        return tracker.map_replica();
    }
    template <typename TFeatureExtractor>
    static const std::vector<int64_t>& LastMatches(const Tracker<TFeatureExtractor>& tracker) {
        return tracker.last_matches();
    }
};

}  // namespace voyahslam::tracking

#endif  // VOYAHSLAM_TESTS_TEST_ACCESS_H
