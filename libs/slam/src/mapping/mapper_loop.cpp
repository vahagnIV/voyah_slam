// The Mapper's loop correction (loop-closing spec section 3; ORB-SLAM3
// LoopClosing::CorrectLoop, SearchAndFuse and the visual
// Optimizer::OptimizeEssentialGraph): one LoopDetected becomes one
// MapTransaction at the next correction epoch.
#include <algorithm>
#include <cassert>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <optional>
#include <set>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>
#include <voyahslam/features/instantiation.h>
#include <voyahslam/geometry/sim3.h>
#include <voyahslam/map/point_statistics.h>
#include <voyahslam/mapping/mapper.h>
#include <voyahslam/optimization/essential_graph.h>
#include "mapping/mapper_internal.h"

namespace voyahslam::mapping {
namespace {

geometry::Sim3 ToSim3(const geometry::RigidTransformation& T) {
    return {1.0, T.rotation(), T.translation()};
}

// The SE3 pose of a similarity S_cw: [R t/s] (ORB-SLAM3 folds the scale into
// the translation).
geometry::RigidTransformation ToPose(const geometry::Sim3& S) {
    return {S.R, S.t / S.scale};
}

// Review Focus 4: a finite similarity with a positive scale.
bool IsValid(const geometry::Sim3& S) {
    return std::isfinite(S.scale) && S.scale > 0.0 && S.R.allFinite() && S.t.allFinite();
}

} // namespace

template <typename TFeatureExtractor>
void Mapper<TFeatureExtractor>::OnLoopDetected(const events::LoopDetected& event) {
    const auto start = std::chrono::steady_clock::now();
    const map::FrameId query = event.query_frame;
    const map::FrameId match = event.match_frame;
    // Review Focus 3: a loop of an older map (the LoopCloser's replica was
    // behind a final TrackingLost) or across maps (map merging) is not ours.
    if (state() != MapperState::Mapping || event.query_map_id != current_map_id() ||
        event.match_map_id != current_map_id()) {
        log_->debug("LoopDetected {} -> {} of maps {} / {}: not the current map {}, ignored", query.value,
                    match.value, event.query_map_id.value, event.match_map_id.value, current_map_id().value);
        return;
    }
    // Review Focus 1: the query or the match was culled since the detection.
    const KeyFrameRecord* query_record = FindRecord(query);
    if (query_record == nullptr || FindRecord(match) == nullptr || query == match) {
        log_->debug("LoopDetected {} -> {}: a Frame is no longer in the map, ignored", query.value, match.value);
        return;
    }
    if (!IsValid(event.S_query_match)) {
        log_->debug("LoopDetected {} -> {}: invalid similarity, ignored", query.value, match.value);
        return;
    }
    // Review Focus 2: the LoopCloser finds the place again until the
    // correction reaches its replica.
    if (IsClosedLoop(query, match)) {
        log_->debug("LoopDetected {} -> {}: loop already closed, ignored", query.value, match.value);
        return;
    }
    const bool fix_scale = detail::BinocularOf<TFeatureExtractor>(query_record->frame.measurement()) != nullptr;

    auto* corrected = CreateTransaction();
    corrected->map_epoch = ++map_epoch_;
    added_keys_.clear();
    replaced_.clear();
    losers_.clear();
    touched_.clear();
    const size_t num_points_before = map_points_.size();

    // 1. The window: the query and its connected Frames (CorrectLoop calls
    // GetVectorCovisibleKeyFrames after UpdateConnections).
    std::vector<map::FrameId> window = {query};
    for (const map::FrameId frame : Connected(query)) {
        window.push_back(frame);
    }
    // 2.-3. Window poses and points.
    const geometry::Sim3 S_query_world =
        geometry::Compose(event.S_query_match, ToSim3(FindKeyFrame(match)->pose()));
    const WindowCorrection correction = CorrectWindow(window, S_query_world);
    // 4. Loop fusion.
    FuseLoopPoints(correction, event.matched_points, [&] {
        std::vector<map::MapPointId> points;
        std::unordered_set<uint32_t> seen;
        for (const map::FrameId frame : MatchWindow(match)) {
            for (const map::MapPointId point : covisibility_.Points(frame)) {
                if (map_points_.IsValid(point) && seen.insert(point.value).second) {
                    points.push_back(point);
                }
            }
        }
        return points;
    }(), *corrected);
    // 5.-6. Loop connections and the essential graph.
    OptimizeGraph(correction, match, fix_scale, *corrected);
    // 7. The loop edge.
    AddLoopEdge(query, match, *corrected);
    // 8. One transaction.
    FinishCorrection(*corrected);
    num_map_points_.store(map_points_.size(), std::memory_order_release);
    log_->info(
        "Loop corrected: Frame {} with Frame {} (scale {:.3f}, fixed scale {}), window {} Frames, {} "
        "pairs, {} points merged, {} -> {} points, epoch {}, {:.1f} ms",
        query.value, match.value, event.S_query_match.scale, fix_scale, window.size(),
        event.matched_points.size(), losers_.size(), num_points_before, map_points_.size(), map_epoch_,
        detail::MillisecondsSince(start));
    observable_.Notify(corrected);
}

template <typename TFeatureExtractor>
std::vector<map::FrameId> Mapper<TFeatureExtractor>::Connected(map::FrameId frame) const {
    const auto covisible = covisibility_.Covisible(frame, std::numeric_limits<size_t>::max());
    std::vector<map::FrameId> connected;
    for (const auto& [other, shared] : covisible) {
        if (shared < options_.loop.min_connection_weight) {
            break;  // sorted by shared points
        }
        connected.push_back(other);
    }
    if (connected.empty() && !covisible.empty()) {
        connected.push_back(covisible.front().first);
    }
    return connected;
}

template <typename TFeatureExtractor>
std::vector<map::FrameId> Mapper<TFeatureExtractor>::MatchWindow(map::FrameId frame) const {
    std::vector<map::FrameId> window = {frame};
    for (const auto& [other, shared] : covisibility_.Covisible(frame, options_.loop.num_covisibles)) {
        window.push_back(other);
    }
    return window;
}

template <typename TFeatureExtractor>
bool Mapper<TFeatureExtractor>::IsClosedLoop(map::FrameId query, map::FrameId match) const {
    // A correction fuses the two sides, so a repeat finds the match's window
    // covisible with the query, or else linked to the query's window by the
    // loop edge. Both checks are a few covisibility lookups.
    std::set<map::FrameId> query_covisible = {query};
    for (const auto& [frame, shared] : covisibility_.Covisible(query, std::numeric_limits<size_t>::max())) {
        query_covisible.insert(frame);
    }
    const std::vector<map::FrameId> match_window = MatchWindow(match);
    const std::set<map::FrameId> match_side(match_window.begin(), match_window.end());
    for (const map::FrameId frame : match_window) {
        if (query_covisible.contains(frame)) {
            return true;
        }
    }
    std::vector<map::FrameId> query_window = Connected(query);
    query_window.push_back(query);
    for (const map::FrameId frame : query_window) {
        for (const map::FrameId partner : FindRecord(frame)->loop_edges) {
            if (match_side.contains(partner)) {
                return true;
            }
        }
    }
    return false;
}

template <typename TFeatureExtractor>
auto Mapper<TFeatureExtractor>::CorrectWindow(const std::vector<map::FrameId>& window,
                                              const geometry::Sim3& S_query_world) -> WindowCorrection {
    assert(!window.empty());
    WindowCorrection correction;
    correction.frames = window;
    const geometry::RigidTransformation T_world_query = FindKeyFrame(window.front())->pose().Inverse();
    for (const map::FrameId frame : window) {
        const geometry::RigidTransformation& T_iw = FindKeyFrame(frame)->pose();
        correction.non_corrected.push_back(T_iw);
        // g2oCorrectedSiw = g2oSic * mg2oLoopScw, Sic from the uncorrected poses.
        correction.corrected.push_back(geometry::Compose(ToSim3(T_iw * T_world_query), S_query_world));
        correction.previous_connected.push_back(Connected(frame));
    }
    for (size_t i = 0; i < window.size(); ++i) {
        // Project with the uncorrected pose, back with the corrected one. The
        // product S_corrected^-1 S_old is the same for every window Frame (the
        // query's), so the point moves alike whichever Frame takes it.
        const geometry::Sim3 S_correction =
            geometry::Compose(geometry::Inverse(correction.corrected[i]), ToSim3(correction.non_corrected[i]));
        for (const map::MapPointId point : covisibility_.Points(window[i])) {
            if (correction.corrected_reference.emplace(point.value, window[i]).second) {
                map_points_.Position(point) = geometry::Apply(S_correction, map_points_.Position(point));
            }
        }
        FindKeyFrame(window[i])->mutable_pose() = ToPose(correction.corrected[i]);
    }
    return correction;
}

template <typename TFeatureExtractor>
void Mapper<TFeatureExtractor>::FuseLoopPoints(
    const WindowCorrection& correction,
    const std::vector<std::pair<map::MapPointId, map::MapPointId>>& matched_points,
    const std::vector<map::MapPointId>& loop_points, events::MapTransaction<TFeatureExtractor>& out_event) {
    // CorrectLoop: each matched query feature's point is replaced by its
    // loop point. A query feature without a point is not carried by
    // LoopDetected; SearchAndFuse below gives most of them their loop point.
    const map::FrameId query = correction.frames.front();
    for (auto [query_point, match_point] : matched_points) {
        query_point = Resolve(query_point);
        match_point = Resolve(match_point);
        if (query_point == match_point || !map_points_.IsValid(query_point) ||
            !map_points_.IsValid(match_point) || !covisibility_.Contains(query, query_point)) {
            continue;
        }
        Replace(query_point, match_point, out_event);
    }
    // SearchAndFuse: the loop points, as they stand after the previous
    // Frames' replacements, into each window Frame at its corrected pose.
    for (const map::FrameId frame : correction.frames) {
        std::vector<map::MapPointId> points;
        for (const map::MapPointId point : loop_points) {
            if (map_points_.IsValid(point)) {
                points.push_back(point);
            }
        }
        FuseInto(*FindKeyFrame(frame), points, out_event, /*loop=*/true);
    }
}

template <typename TFeatureExtractor>
void Mapper<TFeatureExtractor>::OptimizeGraph(const WindowCorrection& correction, map::FrameId match,
                                              bool fix_scale,
                                              events::MapTransaction<TFeatureExtractor>& out_event) {
    const LoopCorrectionOptions& options = options_.loop;
    const map::FrameId query = correction.frames.front();

    // The vertices: every Frame, by id.
    optimization::EssentialGraphProblem problem;
    std::unordered_map<uint32_t, size_t> index;
    for (const std::optional<KeyFrameRecord>& record : keyframes_) {
        if (!record) {
            continue;
        }
        index.emplace(record->frame.id().value, problem.frames.size());
        problem.frames.push_back(record->frame.id());
        problem.T_cw.push_back(record->frame.pose());
        problem.corrected_S_cw.emplace_back();
    }
    std::unordered_set<uint32_t> in_window;
    for (size_t i = 0; i < correction.frames.size(); ++i) {
        const size_t v = index.at(correction.frames[i].value);
        problem.T_cw[v] = correction.non_corrected[i];
        problem.corrected_S_cw[v] = correction.corrected[i];
        in_window.insert(correction.frames[i].value);
    }

    // 5. The loop connections: each window Frame's connections after the
    // fusion, minus those before it and the window; strong ones only, but
    // the query's with the match.
    size_t num_loop_connections = 0;
    for (size_t i = 0; i < correction.frames.size(); ++i) {
        const map::FrameId frame = correction.frames[i];
        const std::vector<map::FrameId>& previous = correction.previous_connected[i];
        for (const auto& [other, shared] : covisibility_.Covisible(frame, std::numeric_limits<size_t>::max())) {
            const bool connected = shared >= options.min_connection_weight;
            if (!connected || in_window.contains(other.value) ||
                std::find(previous.begin(), previous.end(), other) != previous.end()) {
                continue;
            }
            if (shared < options.min_essential_weight && !(frame == query && other == match)) {
                continue;
            }
            problem.loop_connections.emplace_back(index.at(frame.value), index.at(other.value));
            ++num_loop_connections;
        }
    }

    // 6. The spanning tree, the loop edges and the strong covisibility edges
    // (not with the parent or a child), each once from its newer Frame;
    // OptimizeEssentialGraph drops a pair already among the loop connections.
    for (const std::optional<KeyFrameRecord>& record : keyframes_) {
        if (!record) {
            continue;
        }
        const map::FrameId frame = record->frame.id();
        const size_t v = index.at(frame.value);
        if (record->parent) {
            problem.edges.emplace_back(v, index.at(record->parent->value));
        }
        for (const map::FrameId partner : record->loop_edges) {
            if (partner < frame) {
                problem.edges.emplace_back(v, index.at(partner.value));
            }
        }
        for (const auto& [other, shared] : covisibility_.Covisible(frame, std::numeric_limits<size_t>::max())) {
            if (shared < options.min_essential_weight) {
                break;  // sorted by shared points
            }
            if (other < frame && other != record->parent && !record->children.contains(other)) {
                problem.edges.emplace_back(v, index.at(other.value));
            }
        }
    }
    problem.fixed = index.at(match.value);
    problem.fix_scale = fix_scale;
    problem.iterations = options.essential_iterations;

    const optimization::EssentialGraphResult result = optimization::OptimizeEssentialGraph(problem);
    if (!result.success) {
        // The window keeps its rigid correction.
        log_->warn("Loop of Frame {}: essential graph failed ({} Frames, {} edges, {} loop connections)",
                   query.value, problem.frames.size(), problem.edges.size(), num_loop_connections);
        for (const map::FrameId frame : correction.frames) {
            out_event.updated_poses.emplace_back(frame, FindKeyFrame(frame)->pose());
        }
        return;
    }
    log_->debug("Loop of Frame {}: essential graph of {} Frames, {} edges, {} loop connections", query.value,
                problem.frames.size(), problem.edges.size(), num_loop_connections);

    // Poses: [R t/s]. Points: into their reference Frame with the pose they
    // were placed with (the corrected one if CorrectWindow moved them), back
    // with its optimized similarity.
    for (size_t v = 0; v < problem.frames.size(); ++v) {
        FindKeyFrame(problem.frames[v])->mutable_pose() = ToPose(result.S_cw[v]);
        out_event.updated_poses.emplace_back(problem.frames[v], ToPose(result.S_cw[v]));
    }
    for (uint32_t p = 0; p < map_points_.num_ids(); ++p) {
        const map::MapPointId point{p};
        if (!map_points_.IsValid(point)) {
            continue;
        }
        const auto moved = correction.corrected_reference.find(p);
        const bool was_moved = moved != correction.corrected_reference.end();
        const size_t v = index.at(was_moved ? moved->second.value : references_[p]->keyframe.value);
        const geometry::Sim3 S_start = was_moved ? *problem.corrected_S_cw[v] : ToSim3(problem.T_cw[v]);
        map_points_.Position(point) = geometry::Apply(
            geometry::Inverse(result.S_cw[v]), geometry::Apply(S_start, map_points_.Position(point)));
    }
}

template <typename TFeatureExtractor>
void Mapper<TFeatureExtractor>::FinishCorrection(events::MapTransaction<TFeatureExtractor>& out_event) {
    ResolveObservationRecords(out_event);
    std::unordered_set<uint32_t> touched;
    for (const map::MapPointId point : touched_) {
        touched.insert(Resolve(point).value);
    }
    // Every point moved: ORB-SLAM3 UpdateNormalAndDepth; the descriptors of
    // the points whose observations changed (ComputeDistinctiveDescriptors).
    std::vector<map::MapPointId> points;
    points.reserve(map_points_.size());
    for (uint32_t p = 0; p < map_points_.num_ids(); ++p) {
        const map::MapPointId point{p};
        if (!map_points_.IsValid(point)) {
            continue;
        }
        if (touched.contains(p)) {
            UpdatePointStatistics(point);
        } else {
            std::vector<geometry::TPoint3DF> centres;
            for (const map::PointObservation& observation : covisibility_.Observations(point)) {
                centres.push_back(FindKeyFrame(observation.frame)->pose().Inverse().translation());
            }
            map_points_.Normal(point) = map::MeanViewingDirection(map_points_.Position(point), centres);
        }
        points.push_back(point);
    }
    WriteUpdatedPoints(points, out_event);
    out_event.new_points_start_id = map::MapPointId{map_points_.num_ids()};
}

// Mapper is explicitly instantiated in mapper.cpp; the members defined here
// are instantiated here.
#define INSTANTIATE_MAPPER_LOOP(FE)                                                                       \
    template void Mapper<FE>::OnLoopDetected(const events::LoopDetected&);                                \
    template std::vector<map::FrameId> Mapper<FE>::Connected(map::FrameId) const;                         \
    template std::vector<map::FrameId> Mapper<FE>::MatchWindow(map::FrameId) const;                       \
    template bool Mapper<FE>::IsClosedLoop(map::FrameId, map::FrameId) const;                             \
    template Mapper<FE>::WindowCorrection Mapper<FE>::CorrectWindow(const std::vector<map::FrameId>&,     \
                                                                    const geometry::Sim3&);               \
    template void Mapper<FE>::FuseLoopPoints(                                                             \
        const WindowCorrection&, const std::vector<std::pair<map::MapPointId, map::MapPointId>>&,         \
        const std::vector<map::MapPointId>&, events::MapTransaction<FE>&);                                \
    template void Mapper<FE>::OptimizeGraph(const WindowCorrection&, map::FrameId, bool,                  \
                                            events::MapTransaction<FE>&);                                 \
    template void Mapper<FE>::FinishCorrection(events::MapTransaction<FE>&);

INSTANTIATE_MAPPER_LOOP(features::SIFTFeatureExtractor)
INSTANTIATE_MAPPER_LOOP(features::OrbFeatureExtractor)

} // namespace voyahslam::mapping
