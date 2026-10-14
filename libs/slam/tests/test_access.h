#ifndef VOYAHSLAM_TESTS_TEST_ACCESS_H
#define VOYAHSLAM_TESTS_TEST_ACCESS_H

// Test access to the components' thread-affine state (DESIGN section 7, D-14,
// D-26). Each component keeps its replica (and, for the LoopCloser, its
// worker-written statistics) private, because reading it while the worker runs
// is a data race; the component befriends the struct named here, defined once
// per test binary in this header. Tests that drive ProcessPending()
// synchronously (never concurrently with a running worker) read through it.

#include <voyahslam/global_ba/global_bundle_adjuster.h>
#include <voyahslam/loop_closing/common_region.h>
#include <voyahslam/loop_closing/loop_closer.h>
#include <voyahslam/tracking/atlas_replica.h>
#include <voyahslam/tracking/map_replica.h>
#include <voyahslam/tracking/tracker.h>

#include <cstddef>
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

namespace voyahslam::loop_closing {

// The LoopCloser's replica and worker-written statistics. `Queries` is not
// named after its return type: a member may not change the meaning of a name
// used earlier in the class.
struct LoopCloserTestAccess {
    template <typename TFeatureExtractor>
    static const tracking::AtlasReplica<TFeatureExtractor>& Atlas(const LoopCloser<TFeatureExtractor>& loop_closer) {
        return loop_closer.atlas();
    }
    template <typename TFeatureExtractor>
    static const tracking::MapReplica<TFeatureExtractor>& MapReplica(
        const LoopCloser<TFeatureExtractor>& loop_closer) {
        return loop_closer.map_replica();
    }
    template <typename TFeatureExtractor>
    static size_t NumCoincidences(const LoopCloser<TFeatureExtractor>& loop_closer) {
        return loop_closer.num_coincidences();
    }
    template <typename TFeatureExtractor>
    static size_t NumMergeCoincidences(const LoopCloser<TFeatureExtractor>& loop_closer) {
        return loop_closer.num_merge_coincidences();
    }
    template <typename TFeatureExtractor>
    static const CommonRegionStats& LastStats(const LoopCloser<TFeatureExtractor>& loop_closer) {
        return loop_closer.last_stats();
    }
    template <typename TFeatureExtractor>
    static const CommonRegionStats& LastMergeStats(const LoopCloser<TFeatureExtractor>& loop_closer) {
        return loop_closer.last_merge_stats();
    }
    template <typename TFeatureExtractor>
    static const loop_closing::QueryStats& Queries(const LoopCloser<TFeatureExtractor>& loop_closer) {
        return loop_closer.query_stats();
    }
};

}  // namespace voyahslam::loop_closing

namespace voyahslam::global_ba {

struct GlobalBundleAdjusterTestAccess {
    template <typename TFeatureExtractor>
    static const tracking::AtlasReplica<TFeatureExtractor>& Atlas(
        const GlobalBundleAdjuster<TFeatureExtractor>& global_ba) {
        return global_ba.atlas();
    }
    template <typename TFeatureExtractor>
    static const tracking::MapReplica<TFeatureExtractor>& MapReplica(
        const GlobalBundleAdjuster<TFeatureExtractor>& global_ba) {
        return global_ba.map_replica();
    }
};

}  // namespace voyahslam::global_ba

#endif  // VOYAHSLAM_TESTS_TEST_ACCESS_H
