// The Mapper's map merge (map-merging spec section 3; ORB-SLAM3
// LoopClosing::MergeLocal, the visual path, with the weld of
// Optimizer::LocalBundleAdjustment(pMainKF, vpAdjustKF, vpFixedKF) and the
// merge overload of Optimizer::OptimizeEssentialGraph): one merge
// LoopDetected becomes one MapTransaction of the target map at its next
// correction epoch.
#include <algorithm>
#include <cassert>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <optional>
#include <set>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <variant>
#include <vector>
#include <voyahslam/features/instantiation.h>
#include <voyahslam/geometry/sim3.h>
#include <voyahslam/map/absorbed_point_ids.h>
#include <voyahslam/mapping/mapper.h>
#include <voyahslam/optimization/bundle_adjustment.h>
#include <voyahslam/optimization/essential_graph.h>
#include "mapping/mapper_internal.h"

namespace voyahslam::mapping {

using detail::ToPose;
using detail::ToSim3;

template <typename TFeatureExtractor>
void Mapper<TFeatureExtractor>::OnMergeDetected(const events::LoopDetected& event) {
    const auto start = std::chrono::steady_clock::now();
    const map::FrameId query = event.query_frame;
    const map::FrameId match = event.match_frame;
    const map::MapId absorbed_id = event.query_map_id;
    const map::MapId target_id = event.match_map_id;
    assert(absorbed_id != target_id);
    // The query map must be the active one: an inactive or removed map's
    // (the LoopCloser's replica was behind a loss or an earlier merge) is not.
    if (!IsActive(absorbed_id)) {
        log_->debug("Merge of Frame {} (map {}) with Frame {} (map {}): the query map is not the active map {}, "
                    "ignored",
                    query.value, absorbed_id.value, match.value, target_id.value,
                    active_.value_or(map::MapId{0}).value);
        return;
    }
    // The match map must be live and inactive.
    const auto target_it = maps_.find(target_id);
    if (target_it == maps_.end()) {
        log_->debug("Merge of Frame {} with Frame {}: map {} is not live, ignored", query.value, match.value,
                    target_id.value);
        return;
    }
    // Either Frame culled since the detection.
    const KeyFrameRecord* query_record = FindRecord(query);
    if (query_record == nullptr || FindRecordIn(target_it->second, match) == nullptr) {
        log_->debug("Merge of Frame {} with Frame {}: a Frame is no longer in its map, ignored", query.value,
                    match.value);
        return;
    }
    if (!detail::IsValidSim3(event.S_query_match)) {
        log_->debug("Merge of Frame {} with Frame {}: invalid similarity, ignored", query.value, match.value);
        return;
    }
    const bool fix_scale = detail::BinocularOf<TFeatureExtractor>(query_record->frame.measurement()) != nullptr;

    // 1. The windows, each in its map (ORB-SLAM3 builds them before moving
    // anything), and the merge window's points to fuse.
    const std::vector<map::FrameId> current_window = MergeWindowOf(Active(), query);
    const std::vector<map::FrameId> merge_window = MergeWindowOf(target_it->second, match);
    std::vector<map::MapPointId> merge_points;
    {
        std::unordered_set<uint32_t> seen;
        for (const map::FrameId frame : merge_window) {
            for (const map::MapPointId point : target_it->second.covisibility.Points(frame)) {
                if (target_it->second.map_points.IsValid(point) && seen.insert(point.value).second) {
                    merge_points.push_back(point);
                }
            }
        }
    }

    // 7 (first). Absorb: every Frame and point of the active map moves into
    // the target by the id rule, and the target becomes the active map. From
    // here on everything happens in the target.
    MapState& absorbed = Active();
    MapState& target = target_it->second;
    std::vector<map::FrameId> absorbed_frames;
    for (const std::optional<KeyFrameRecord>& record : absorbed.keyframes) {
        if (record) {
            absorbed_frames.push_back(record->frame.id());
        }
    }
    const size_t num_target_frames = num_keyframes(target_id);
    const size_t num_target_points = target.map_points.size();
    const size_t num_absorbed_points = absorbed.map_points.size();
    const std::vector<std::optional<map::MapPointId>> new_ids = AbsorbMap(absorbed, target);
    maps_.erase(absorbed_id);
    active_ = target_id;
    active_state_ = &target;
    current_map_id_.store(target_id, std::memory_order_release);

    auto merged = CreateTransaction();
    merged->map_epoch = ++Active().map_epoch;
    merged->absorbed_map = absorbed_id;
    merged->removed_maps.push_back(absorbed_id);

    // 2. The current window takes the merge similarity, its points with it
    // (clears the transaction scratch state).
    const geometry::Sim3 S_query_world = geometry::Compose(event.S_query_match, ToSim3(FindKeyFrame(match)->pose()));
    const WindowCorrection correction = CorrectWindow(current_window, S_query_world);
    // 3. One spanning tree, rooted at the target's root.
    ReRootTree(query, match, *merged);
    // 4. Seam fusion: the detected pairs (the query's points under their new
    // ids), then the merge window's points into the current window.
    std::vector<std::pair<map::MapPointId, map::MapPointId>> matched_points;
    matched_points.reserve(event.matched_points.size());
    for (const auto& [query_point, match_point] : event.matched_points) {
        if (query_point.value < new_ids.size() && new_ids[query_point.value]) {
            matched_points.emplace_back(*new_ids[query_point.value], match_point);
        }
    }
    FuseLoopPoints(correction, matched_points, merge_points, *merged);
    // 5. The weld: the current window free, the merge window fixed.
    std::vector<map::MapPointId> changed;
    const auto weld_start = std::chrono::steady_clock::now();
    WeldWindows(current_window, merge_window, *merged, changed);
    const double weld_ms = detail::MillisecondsSince(weld_start);
    // 6. The rest of the absorbed map.
    MoveRestOfMap(correction, absorbed_frames, fix_scale);
    // 7. The merge edge, kept as a loop edge (it starts a global BA of the
    // target).
    AddLoopEdge(query, match, *merged);
    // 8. One transaction: every absorbed Frame's pose; a point whose
    // reference observation the weld removed takes the replica's next one
    // (at its final pose and position); then every point.
    for (const map::FrameId frame : absorbed_frames) {
        merged->updated_poses.emplace_back(frame, FindKeyFrame(frame)->pose());
    }
    for (const map::MapPointId point : changed) {
        if (!Active().map_points.IsValid(point)) {
            continue;
        }
        if (!Active().covisibility.Contains(Active().references[point.value]->keyframe, point)) {
            const std::vector<map::PointObservation>& observations = Active().covisibility.Observations(point);
            const auto next = std::min_element(
                observations.begin(), observations.end(),
                [](const map::PointObservation& a, const map::PointObservation& b) { return a.frame < b.frame; });
            SetReference(point, *FindKeyFrame(next->frame), next->feature);
        }
        touched_.push_back(point);
    }
    FinishCorrection(*merged);
    num_keyframes_.store(num_keyframes(target_id), std::memory_order_release);
    num_map_points_.store(Active().map_points.size(), std::memory_order_release);
    num_merges_.fetch_add(1, std::memory_order_acq_rel);
    const auto timestamp_of = [this](map::FrameId frame) {
        return std::visit([](const auto& measurement) { return measurement.timestamp(); },
                          FindKeyFrame(frame)->measurement());
    };
    log_->info(
        "Map {} merged into map {}: Frame {} (t={:.3f}) with Frame {} (t={:.3f}) (scale {:.3f}, fixed scale {}), "
        "windows {} / {} Frames, {} pairs, {} points fused, {} + {} -> {} Frames, {} + {} -> {} points, epoch {}, "
        "{:.1f} ms (weld {:.1f} ms)",
        absorbed_id.value, target_id.value, query.value, timestamp_of(query), match.value, timestamp_of(match),
        event.S_query_match.scale, fix_scale, current_window.size(), merge_window.size(), matched_points.size(),
        losers_.size(), num_target_frames, absorbed_frames.size(), num_keyframes_.load(std::memory_order_relaxed),
        num_target_points, num_absorbed_points, Active().map_points.size(), Active().map_epoch,
        detail::MillisecondsSince(start), weld_ms);
    observable_.Notify(std::move(merged));
}

template <typename TFeatureExtractor>
std::vector<std::optional<map::MapPointId>> Mapper<TFeatureExtractor>::AbsorbMap(MapState& source,
                                                                                  MapState& target) {
    const std::vector<std::optional<map::MapPointId>> new_ids =
        map::AbsorbedPointIds(source.map_points.num_ids(), map::MapPointId{target.map_points.num_ids()},
                              [&](map::MapPointId point) { return source.map_points.IsValid(point); });
    // Points, by old id ascending: the target's ids are issued in that order.
    for (uint32_t p = 0; p < new_ids.size(); ++p) {
        if (!new_ids[p]) {
            continue;
        }
        const map::MapPointId old{p};
        [[maybe_unused]] const map::MapPointId id = target.map_points.Add(
            source.map_points.Position(old), source.map_points.Descriptor(old), source.map_points.Normal(old));
        assert(id == *new_ids[p] && target.references.size() == id.value);
        target.references.push_back(source.references[p]);
        target.num_visible.push_back(source.num_visible[p]);
        target.num_found.push_back(source.num_found[p]);
    }
    // Frames, slotted by id over both maps' ranges: after an earlier merge the
    // two maps' ids interleave (the source may hold ids below and above the
    // target's). The target's root stays the root.
    if (!source.keyframes.empty()) {
        assert(!target.keyframes.empty());
        const map::FrameId first = std::min(target.first_slot, source.first_slot);
        const size_t end = std::max(target.first_slot.value + target.keyframes.size(),
                                    source.first_slot.value + source.keyframes.size());
        std::vector<std::optional<KeyFrameRecord>> slots(end - first.value);
        for (MapState* state : {&target, &source}) {
            for (size_t i = 0; i < state->keyframes.size(); ++i) {
                if (state->keyframes[i]) {
                    std::optional<KeyFrameRecord>& slot = slots[state->first_slot.value + i - first.value];
                    assert(!slot);  // Frame ids are unique across maps
                    slot = std::move(state->keyframes[i]);
                }
            }
        }
        target.keyframes = std::move(slots);
        target.first_slot = first;
    }
    // Observations, point by point in the source's order (as the replicas do).
    for (uint32_t p = 0; p < new_ids.size(); ++p) {
        if (!new_ids[p]) {
            continue;
        }
        for (const map::PointObservation& observation : source.covisibility.Observations(map::MapPointId{p})) {
            target.covisibility.Add(observation.frame, observation.feature, *new_ids[p]);
        }
    }
    // The Mapper's bookkeeping: the source was the map being extended.
    target.recent_points.clear();
    for (const RecentPoint& recent : source.recent_points) {
        if (recent.id.value < new_ids.size() && new_ids[recent.id.value]) {
            target.recent_points.push_back({*new_ids[recent.id.value], recent.created, recent.stereo});
        }
    }
    target.relocalized_at = source.relocalized_at;
    return new_ids;
}

template <typename TFeatureExtractor>
std::vector<map::FrameId> Mapper<TFeatureExtractor>::MergeWindowOf(const MapState& state,
                                                                   map::FrameId frame) const {
    constexpr int kMaxRounds = 5;  // nMaxTries
    const size_t size = options_.atlas.merge_window;
    std::vector<map::FrameId> window = {frame};
    std::set<map::FrameId> in_window = {frame};
    const auto best = [&](map::FrameId of, size_t count) {
        std::vector<map::FrameId> connected = Connected(state, of);
        if (connected.size() > count) {
            connected.resize(count);
        }
        return connected;
    };
    for (const map::FrameId other : best(frame, size)) {
        if (in_window.insert(other).second) {
            window.push_back(other);
        }
    }
    for (int round = 0; round < kMaxRounds && window.size() < size; ++round) {
        // Each round adds every new Frame it finds, as ORB-SLAM3 does.
        std::vector<map::FrameId> found;
        for (const map::FrameId member : window) {
            for (const map::FrameId other : best(member, size / 2)) {
                if (!in_window.contains(other) &&
                    std::find(found.begin(), found.end(), other) == found.end()) {
                    found.push_back(other);
                }
            }
        }
        for (const map::FrameId other : found) {
            in_window.insert(other);
            window.push_back(other);
        }
    }
    return window;
}

template <typename TFeatureExtractor>
void Mapper<TFeatureExtractor>::ReRootTree(map::FrameId query, map::FrameId match,
                                           events::MapTransaction<TFeatureExtractor>& out_event) {
    std::optional<map::FrameId> new_child = FindRecord(query)->parent;
    map::FrameId new_parent = query;
    ChangeParent(query, match, out_event);
    while (new_child) {
        const std::optional<map::FrameId> old_parent = FindRecord(*new_child)->parent;
        ChangeParent(*new_child, new_parent, out_event);
        new_parent = *new_child;
        new_child = old_parent;
    }
}

template <typename TFeatureExtractor>
void Mapper<TFeatureExtractor>::WeldWindows(const std::vector<map::FrameId>& free,
                                            const std::vector<map::FrameId>& fixed,
                                            events::MapTransaction<TFeatureExtractor>& out_event,
                                            std::vector<map::MapPointId>& out_changed) {
    const MapQualityOptions& options = options_.map_quality;
    std::vector<map::FrameId> frames = free;
    frames.insert(frames.end(), fixed.begin(), fixed.end());
    std::unordered_map<uint32_t, size_t> frame_index;
    for (size_t i = 0; i < frames.size(); ++i) {
        frame_index.emplace(frames[i].value, i);
    }
    std::vector<map::MapPointId> points;
    std::unordered_set<uint32_t> seen;
    for (const map::FrameId frame : frames) {
        for (const map::MapPointId point : Active().covisibility.Points(frame)) {
            if (Active().map_points.IsValid(point) && seen.insert(point.value).second) {
                points.push_back(point);
            }
        }
    }

    optimization::BaProblem problem;
    for (size_t i = 0; i < frames.size(); ++i) {
        const TKeyFrame& keyframe = *FindKeyFrame(frames[i]);
        problem.keyframe_poses.push_back(keyframe.pose());
        problem.keyframe_fixed.push_back(i >= free.size());
        problem.keyframe_cameras.push_back(detail::LeftOf<TFeatureExtractor>(keyframe.measurement()).camera());
        problem.keyframe_stereo.push_back(detail::BaStereoOf<TFeatureExtractor>(keyframe.measurement()));
    }
    problem.points.resize(3, static_cast<Eigen::Index>(points.size()));
    std::vector<std::pair<map::FrameId, map::MapPointId>> observed;
    for (size_t p = 0; p < points.size(); ++p) {
        problem.points.col(static_cast<Eigen::Index>(p)) = Active().map_points.Position(points[p]);
        for (const map::PointObservation& observation : Active().covisibility.Observations(points[p])) {
            const auto k = frame_index.find(observation.frame.value);
            if (k == frame_index.end()) {
                continue;  // an observer outside both windows
            }
            problem.observations.push_back(detail::BaObservationOf<TFeatureExtractor>(
                FindKeyFrame(observation.frame)->measurement(), observation.feature, k->second, p,
                options_.fuse.sigma_px, options_.fuse.level_scale));
            observed.emplace_back(observation.frame, points[p]);
        }
    }
    if (points.empty() || problem.observations.empty()) {
        return;
    }
    optimization::BundleAdjustmentOptions ba_options;
    ba_options.robust_iterations = options.ba_robust_iterations;
    ba_options.iterations = options.ba_iterations;
    const optimization::BaResult result = optimization::BundleAdjust(problem, ba_options);
    if (!result.success) {
        log_->warn("Merge weld of Frame {}: BA failed ({} Frames, {} points); the windows keep their poses",
                   free.front().value, frames.size(), points.size());
        return;
    }
    for (size_t i = 0; i < free.size(); ++i) {
        FindKeyFrame(frames[i])->mutable_pose() = result.keyframe_poses[i];
    }
    for (size_t p = 0; p < points.size(); ++p) {
        Active().map_points.Position(points[p]) = result.points.col(static_cast<Eigen::Index>(p));
    }
    size_t num_outliers = 0;
    for (size_t o = 0; o < observed.size(); ++o) {
        const auto& [observer, point] = observed[o];
        if (result.inlier[o] || !Active().map_points.IsValid(point)) {
            continue;
        }
        Active().covisibility.Remove(observer, point);
        out_event.removed_observations.push_back({observer, point});
        ++num_outliers;
        if (Active().covisibility.NumObservations(point) == 0) {
            RemovePoint(point, out_event);
        } else {
            out_changed.push_back(point);
        }
    }
    log_->debug("Merge weld of Frame {}: {} free / {} fixed Frames, {} points, {} iterations, {} outliers",
                free.front().value, free.size(), fixed.size(), points.size(), result.iterations, num_outliers);
}

template <typename TFeatureExtractor>
void Mapper<TFeatureExtractor>::MoveRestOfMap(const WindowCorrection& correction,
                                              const std::vector<map::FrameId>& absorbed_frames, bool fix_scale) {
    const LoopCorrectionOptions& options = options_.loop;
    const std::set<map::FrameId> window(correction.frames.begin(), correction.frames.end());
    const double scale = correction.corrected.front().scale;
    const geometry::Sim3& S_query_world = correction.corrected.front();
    const geometry::RigidTransformation T_world_query = correction.non_corrected.front().Inverse();

    // The vertices: the absorbed Frames, the window first. Per vertex its
    // pose in the absorbed map's world (the edges' measurements) and its
    // start: the welded pose at the merge scale for the window, the merge
    // similarity for the rest (ORB-SLAM3's mono rigid move).
    optimization::EssentialGraphProblem problem;
    std::unordered_map<uint32_t, size_t> index;
    const auto add_vertex = [&](map::FrameId frame, const geometry::RigidTransformation& T_old,
                                const geometry::Sim3& start) {
        index.emplace(frame.value, problem.frames.size());
        problem.frames.push_back(frame);
        problem.T_cw.push_back(T_old);
        problem.corrected_S_cw.emplace_back(start);
    };
    for (size_t i = 0; i < correction.frames.size(); ++i) {
        const geometry::RigidTransformation& T_welded = FindKeyFrame(correction.frames[i])->pose();
        add_vertex(correction.frames[i], correction.non_corrected[i],
                   geometry::Sim3{scale, T_welded.rotation(), scale * T_welded.translation()});
        if (i > 0) {
            problem.also_fixed.push_back(i);
        }
    }
    problem.fixed = 0;
    const size_t num_window = problem.frames.size();
    for (const map::FrameId frame : absorbed_frames) {
        if (window.contains(frame)) {
            continue;
        }
        const geometry::RigidTransformation& T_iw = FindKeyFrame(frame)->pose();
        add_vertex(frame, T_iw, geometry::Compose(ToSim3(T_iw * T_world_query), S_query_world));
    }
    if (problem.frames.size() == num_window) {
        return;  // the window holds the whole absorbed map
    }

    // The rigid move: poses, and the points of the rest (those CorrectWindow
    // did not move) with their reference Frame.
    std::vector<map::MapPointId> rest_points;
    for (uint32_t p = 0; p < Active().map_points.num_ids(); ++p) {
        const map::MapPointId point{p};
        if (!Active().map_points.IsValid(point) || correction.corrected_reference.contains(p)) {
            continue;
        }
        const auto v = index.find(Active().references[p]->keyframe.value);
        if (v == index.end() || v->second < num_window) {
            continue;  // a target point (or a window point, already moved)
        }
        Active().map_points.Position(point) = geometry::Apply(
            geometry::Compose(geometry::Inverse(*problem.corrected_S_cw[v->second]), ToSim3(problem.T_cw[v->second])),
            Active().map_points.Position(point));
        rest_points.push_back(point);
    }
    for (size_t v = num_window; v < problem.frames.size(); ++v) {
        FindKeyFrame(problem.frames[v])->mutable_pose() = ToPose(*problem.corrected_S_cw[v]);
    }

    // The essential graph within the absorbed Frames (the target's have no
    // relation to them in ORB-SLAM3's merge graph): spanning tree, loop edges
    // and strong covisibility edges, each once from its newer Frame.
    for (const map::FrameId frame : problem.frames) {
        const KeyFrameRecord& record = *FindRecord(frame);
        const size_t v = index.at(frame.value);
        const auto vertex_of = [&](map::FrameId other) -> std::optional<size_t> {
            const auto it = index.find(other.value);
            return it == index.end() ? std::nullopt : std::optional<size_t>(it->second);
        };
        if (record.parent) {
            if (const auto parent = vertex_of(*record.parent)) {
                problem.edges.emplace_back(v, *parent);
            }
        }
        for (const map::FrameId partner : record.loop_edges) {
            const auto other = vertex_of(partner);
            if (other && partner < frame) {
                problem.edges.emplace_back(v, *other);
            }
        }
        for (const auto& [other, shared] :
             Active().covisibility.Covisible(frame, std::numeric_limits<size_t>::max())) {
            if (shared < options.min_essential_weight) {
                break;  // sorted by shared points
            }
            const auto u = vertex_of(other);
            if (u && other < frame && other != record.parent && !record.children.contains(other)) {
                problem.edges.emplace_back(v, *u);
            }
        }
    }
    problem.fix_scale = fix_scale;
    problem.iterations = options.essential_iterations;
    const optimization::EssentialGraphResult result = optimization::OptimizeEssentialGraph(problem);
    if (!result.success) {
        log_->warn("Merge of Frame {}: essential graph failed ({} Frames, {} edges); the rest keeps the merge "
                   "similarity",
                   correction.frames.front().value, problem.frames.size(), problem.edges.size());
        return;
    }
    for (size_t v = num_window; v < problem.frames.size(); ++v) {
        FindKeyFrame(problem.frames[v])->mutable_pose() = ToPose(result.S_cw[v]);
    }
    for (const map::MapPointId point : rest_points) {
        const size_t v = index.at(Active().references[point.value]->keyframe.value);
        Active().map_points.Position(point) =
            geometry::Apply(geometry::Inverse(result.S_cw[v]),
                            geometry::Apply(*problem.corrected_S_cw[v], Active().map_points.Position(point)));
    }
    log_->debug("Merge of Frame {}: essential graph of {} Frames ({} fixed), {} edges",
                correction.frames.front().value, problem.frames.size(), num_window, problem.edges.size());
}

// Mapper is explicitly instantiated in mapper.cpp; the members defined here
// are instantiated here.
#define INSTANTIATE_MAPPER_MERGE(FE)                                                                       \
    template void Mapper<FE>::OnMergeDetected(const events::LoopDetected&);                                \
    template std::vector<std::optional<map::MapPointId>> Mapper<FE>::AbsorbMap(MapState&, MapState&);      \
    template std::vector<map::FrameId> Mapper<FE>::MergeWindowOf(const MapState&, map::FrameId) const;     \
    template void Mapper<FE>::ReRootTree(map::FrameId, map::FrameId, events::MapTransaction<FE>&);         \
    template void Mapper<FE>::WeldWindows(const std::vector<map::FrameId>&, const std::vector<map::FrameId>&, \
                                          events::MapTransaction<FE>&, std::vector<map::MapPointId>&);     \
    template void Mapper<FE>::MoveRestOfMap(const WindowCorrection&, const std::vector<map::FrameId>&, bool);

INSTANTIATE_MAPPER_MERGE(features::OrbFeatureExtractor)

} // namespace voyahslam::mapping
