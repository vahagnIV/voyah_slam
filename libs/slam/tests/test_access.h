#ifndef VOYAHSLAM_TESTS_TEST_ACCESS_H
#define VOYAHSLAM_TESTS_TEST_ACCESS_H

// Test access to the components' thread-affine state (DESIGN section 7, D-14,
// D-26). Each component keeps its replica (and, for the LoopCloser, its
// worker-written statistics) private, because reading it while the worker runs
// is a data race; the component befriends the struct named here, defined once
// per test binary in this header. ProcessPending() of the four components is
// private too (IN-07, 0.9.2): tests drive it synchronously (never concurrently
// with a running worker) through the ProcessPending wrappers here, and read the
// state through the other members.

#include <voyahslam/bow/bow_vector.h>
#include <voyahslam/geometry/sim3.h>
#include <voyahslam/global_ba/global_bundle_adjuster.h>
#include <voyahslam/loop_closing/common_region.h>
#include <voyahslam/loop_closing/loop_closer.h>
#include <voyahslam/mapping/mapper.h>
#include <voyahslam/tracking/atlas_replica.h>
#include <voyahslam/tracking/map_replica.h>
#include <voyahslam/tracking/tracker.h>

#include <cstddef>
#include <cstdint>
#include <optional>
#include <set>
#include <utility>
#include <vector>

namespace voyahslam::mapping {

// The Mapper's private steps for tests: publishes a transaction that adds a
// loop edge between Frames `a` and `b` of the current map; runs the loop
// correction's CorrectWindow (nothing published) and reads the map back.
struct MapperTestAccess {
    // One pass of the worker loop, driven by the test thread.
    template <typename TFeatureExtractor>
    [[nodiscard]] static bool ProcessPending(Mapper<TFeatureExtractor>& out_mapper) {
        return out_mapper.ProcessPending();
    }
    template <typename TFeatureExtractor>
    static void PublishLoopEdge(Mapper<TFeatureExtractor>& out_mapper, map::FrameId a,
                                map::FrameId b) {
        auto event = out_mapper.CreateTransaction();
        out_mapper.AddLoopEdge(a, b, *event);
        out_mapper.observable_.Notify(std::move(event));
    }
    template <typename TFeatureExtractor>
    static void CorrectWindow(Mapper<TFeatureExtractor>& out_mapper, const std::vector<map::FrameId>& window,
                              const geometry::Sim3& S_query_world) {
        out_mapper.CorrectWindow(window, S_query_world);
    }
    // Steps 1-5 of a correction of `query` with `match` that fuses only
    // `matched_points` (no loop points): the loop connections. Nothing is
    // published.
    template <typename TFeatureExtractor>
    static std::vector<std::pair<map::FrameId, map::FrameId>> LoopConnections(
        Mapper<TFeatureExtractor>& out_mapper, map::FrameId query, map::FrameId match,
        const geometry::Sim3& S_query_match,
        const std::vector<std::pair<map::MapPointId, map::MapPointId>>& matched_points) {
        std::vector<map::FrameId> window = {query};
        for (const map::FrameId frame : out_mapper.Connected(query)) window.push_back(frame);
        const geometry::RigidTransformation& T_match = out_mapper.FindKeyFrame(match)->pose();
        const auto correction = out_mapper.CorrectWindow(
            window, geometry::Compose(S_query_match, geometry::Sim3{1.0, T_match.rotation(), T_match.translation()}));
        const auto scratch = out_mapper.CreateTransaction();
        out_mapper.FuseLoopPoints(correction, matched_points, {}, *scratch);
        return out_mapper.LoopConnections(correction, match);
    }
    // Culls Frame `frame` of the current map through Frame culling's path
    // (RemoveFrame, FinishRefinement) and publishes it as one transaction.
    template <typename TFeatureExtractor>
    static void CullFrame(Mapper<TFeatureExtractor>& out_mapper, map::FrameId frame) {
        auto event = out_mapper.CreateTransaction();
        std::vector<map::MapPointId> changed;
        out_mapper.RemoveFrame(frame, *event, changed);
        out_mapper.FinishRefinement(std::move(changed), *event);
        out_mapper.num_keyframes_.fetch_sub(1, std::memory_order_acq_rel);
        out_mapper.num_map_points_.store(out_mapper.Active().map_points.size(), std::memory_order_release);
        out_mapper.observable_.Notify(std::move(event));
    }
    // Map merging step 1: the window of `frame` in map `map` (it must be live).
    template <typename TFeatureExtractor>
    static std::vector<map::FrameId> MergeWindowOf(const Mapper<TFeatureExtractor>& mapper, map::MapId map,
                                                   map::FrameId frame) {
        return mapper.MergeWindowOf(mapper.maps_.at(map), frame);
    }
    template <typename TFeatureExtractor>
    static geometry::RigidTransformation Pose(const Mapper<TFeatureExtractor>& mapper, map::FrameId frame) {
        return mapper.FindKeyFrame(frame)->pose();
    }
    template <typename TFeatureExtractor>
    static geometry::TPoint3DF Position(const Mapper<TFeatureExtractor>& mapper, map::MapPointId point) {
        return mapper.Active().map_points.Position(point);
    }
    // Map `map`'s points and their visible / found counters (it must be live).
    template <typename TFeatureExtractor>
    static size_t NumPoints(const Mapper<TFeatureExtractor>& mapper, map::MapId map) {
        return mapper.maps_.at(map).map_points.size();
    }
    template <typename TFeatureExtractor>
    static std::vector<uint32_t> NumVisible(const Mapper<TFeatureExtractor>& mapper, map::MapId map) {
        return mapper.maps_.at(map).num_visible;
    }
    template <typename TFeatureExtractor>
    static std::vector<uint32_t> NumFound(const Mapper<TFeatureExtractor>& mapper, map::MapId map) {
        return mapper.maps_.at(map).num_found;
    }

    // The Mapper-thread accessors, private on the Mapper (D-14): tests that
    // drive ProcessPending() synchronously, or read after Stop(), use these.
    template <typename TFeatureExtractor>
    static const map::CovisibilityIndex& Covisibility(const Mapper<TFeatureExtractor>& mapper) {
        return mapper.covisibility();
    }
    template <typename TFeatureExtractor>
    static std::optional<map::PointReference> Reference(const Mapper<TFeatureExtractor>& mapper,
                                                        map::MapPointId point) {
        return mapper.Reference(point);
    }
    template <typename TFeatureExtractor>
    static uint32_t NumVisible(const Mapper<TFeatureExtractor>& mapper, map::MapPointId point) {
        return mapper.NumVisible(point);
    }
    template <typename TFeatureExtractor>
    static uint32_t NumFound(const Mapper<TFeatureExtractor>& mapper, map::MapPointId point) {
        return mapper.NumFound(point);
    }
    template <typename TFeatureExtractor>
    static const LocalBaSummary& LastLocalBa(const Mapper<TFeatureExtractor>& mapper) {
        return mapper.last_local_ba();
    }
    template <typename TFeatureExtractor>
    static const bow::BowVector* BowVectorOf(const Mapper<TFeatureExtractor>& mapper, map::FrameId frame) {
        return mapper.BowVectorOf(frame);
    }
    template <typename TFeatureExtractor>
    static std::optional<map::FrameId> Parent(const Mapper<TFeatureExtractor>& mapper, map::FrameId frame) {
        return mapper.Parent(frame);
    }
    template <typename TFeatureExtractor>
    static std::set<map::FrameId> LoopEdges(const Mapper<TFeatureExtractor>& mapper, map::FrameId frame) {
        return mapper.LoopEdges(frame);
    }
    template <typename TFeatureExtractor>
    static const typename Mapper<TFeatureExtractor>::TKeyFrame* KeyFrame(const Mapper<TFeatureExtractor>& mapper,
                                                                         map::FrameId frame) {
        return mapper.KeyFrame(frame);
    }
    template <typename TFeatureExtractor>
    static std::vector<map::MapId> MapIds(const Mapper<TFeatureExtractor>& mapper) {
        return mapper.map_ids();
    }
    template <typename TFeatureExtractor>
    static std::optional<map::MapId> ActiveMap(const Mapper<TFeatureExtractor>& mapper) {
        return mapper.active_map();
    }
    template <typename TFeatureExtractor>
    static size_t NumKeyframes(const Mapper<TFeatureExtractor>& mapper, map::MapId id) {
        return mapper.num_keyframes(id);
    }
};

}  // namespace voyahslam::mapping

namespace voyahslam::tracking {

struct TrackerTestAccess {
    // One pass of the worker loop, driven by the test thread.
    template <typename TFeatureExtractor>
    [[nodiscard]] static bool ProcessPending(Tracker<TFeatureExtractor>& out_tracker) {
        return out_tracker.ProcessPending();
    }
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
    // One pass of the worker loop, driven by the test thread.
    template <typename TFeatureExtractor>
    [[nodiscard]] static bool ProcessPending(LoopCloser<TFeatureExtractor>& out_loop_closer) {
        return out_loop_closer.ProcessPending();
    }
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
    // One pass of the worker loop, driven by the test thread.
    template <typename TFeatureExtractor>
    [[nodiscard]] static bool ProcessPending(GlobalBundleAdjuster<TFeatureExtractor>& out_global_ba) {
        return out_global_ba.ProcessPending();
    }
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
