// The Mapper's merge of a global BA result (global-BA spec section 3;
// ORB-SLAM3's post-GBA block in LoopClosing::RunGlobalBundleAdjustment): one
// GlobalBAFinished becomes one MapTransaction at the next correction epoch.
#include <cassert>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <optional>
#include <unordered_map>
#include <utility>
#include <vector>
#include <voyahslam/features/instantiation.h>
#include <voyahslam/map/point_statistics.h>
#include <voyahslam/mapping/mapper.h>
#include "mapping/mapper_internal.h"

namespace voyahslam::mapping {

template <typename TFeatureExtractor>
void Mapper<TFeatureExtractor>::OnGlobalBaFinished(const events::GlobalBAFinished& event) {
    // A result of another map (or with no map, or while the map is lost), or
    // of another epoch of the map (a later correction came in between:
    // ORB-SLAM3 mnFullBAIdx), is not merged.
    if (state() != MapperState::Mapping || event.map_id != current_map_id() || event.map_epoch != map_epoch_) {
        log_->debug("GlobalBAFinished of map {} at epoch {}: the current map is {} at epoch {}, ignored",
                    event.map_id.value, event.map_epoch, current_map_id().value, map_epoch_);
        num_global_ba_rejected_.fetch_add(1, std::memory_order_acq_rel);
        return;
    }
    MergeGlobalBa(event.frame_poses, event.point_positions);
    num_global_ba_merges_.fetch_add(1, std::memory_order_acq_rel);
}

template <typename TFeatureExtractor>
void Mapper<TFeatureExtractor>::MergeGlobalBa(
    const std::vector<std::pair<map::FrameId, geometry::RigidTransformation>>& frame_poses,
    const std::vector<std::pair<map::MapPointId, geometry::TPoint3DF>>& point_positions) {
    const auto start = std::chrono::steady_clock::now();
    // One transaction; nothing here uses the scratch state, which is cleared
    // as at the start of every correction.
    ResetTransactionScratch();
    auto* merged = CreateTransaction();
    merged->map_epoch = ++map_epoch_;

    std::unordered_map<uint32_t, const geometry::RigidTransformation*> optimized_poses;
    for (const auto& [frame, T_cw] : frame_poses) {
        optimized_poses.emplace(frame.value, &T_cw);
    }
    // Frames: from the root(s), parents before children. The pose of each
    // corrected Frame before the merge (mTcwBefGBA) is kept for its children
    // and its points.
    std::unordered_map<uint32_t, geometry::RigidTransformation> before;
    std::deque<map::FrameId> queue;
    for (const std::optional<KeyFrameRecord>& record : keyframes_) {
        if (record && !record->parent) {
            queue.push_back(record->frame.id());
        }
    }
    size_t num_optimized_frames = 0;
    while (!queue.empty()) {
        const map::FrameId frame = queue.front();
        queue.pop_front();
        KeyFrameRecord& record = *FindRecord(frame);
        const geometry::RigidTransformation T_cur = record.frame.pose();
        std::optional<geometry::RigidTransformation> T_new;
        if (const auto optimized = optimized_poses.find(frame.value); optimized != optimized_poses.end()) {
            T_new = *optimized->second;
            ++num_optimized_frames;
        } else if (record.parent) {
            // A Frame created after the snapshot: T_cur T_parent,cur^-1 T_parent,new.
            if (const auto parent = before.find(record.parent->value); parent != before.end()) {
                T_new = T_cur * parent->second.Inverse() * FindKeyFrame(*record.parent)->pose();
            }
        }
        if (T_new) {
            before.emplace(frame.value, T_cur);
            record.frame.mutable_pose() = *T_new;
            merged->updated_poses.emplace_back(frame, *T_new);
        }
        for (const map::FrameId child : record.children) {
            queue.push_back(child);
        }
    }

    // Points: the optimized ones that are still alive take their result; a
    // point created after the snapshot moves with its reference Frame, if
    // that was corrected, X_new = T_ref,new^-1 T_ref,cur X.
    std::vector<map::MapPointId> moved;
    std::vector<bool> optimized_point(map_points_.num_ids(), false);
    for (const auto& [point, X] : point_positions) {
        if (map_points_.IsValid(point)) {
            map_points_.Position(point) = X;
            optimized_point[point.value] = true;
            moved.push_back(point);
        }
    }
    const size_t num_optimized_points = moved.size();
    for (uint32_t p = 0; p < map_points_.num_ids(); ++p) {
        const map::MapPointId point{p};
        if (optimized_point[p] || !map_points_.IsValid(point)) {
            continue;
        }
        const map::FrameId reference = references_[p]->keyframe;
        const auto reference_before = before.find(reference.value);
        if (reference_before == before.end()) {
            continue;
        }
        map_points_.Position(point) = FindKeyFrame(reference)->pose().Inverse().Transform(
            reference_before->second.Transform(geometry::TPoint3DF(map_points_.Position(point))));
        moved.push_back(point);
    }
    // The moved points' normals (ORB-SLAM3 UpdateNormalAndDepth), as in a
    // loop correction; their descriptors are unchanged.
    for (const map::MapPointId point : moved) {
        std::vector<geometry::TPoint3DF> centres;
        for (const map::PointObservation& observation : covisibility_.Observations(point)) {
            centres.push_back(FindKeyFrame(observation.frame)->pose().Inverse().translation());
        }
        map_points_.Normal(point) = map::MeanViewingDirection(map_points_.Position(point), centres);
    }
    WriteUpdatedPoints(moved, *merged);
    merged->new_points_start_id = map::MapPointId{map_points_.num_ids()};
    // No loop edge: the GlobalBA thread starts a run on a transaction that
    // adds one.
    assert(merged->added_loop_edges.empty());

    log_->info(
        "Global BA merged: {} of {} Frames optimized, {} propagated; {} points optimized, {} propagated; "
        "epoch {}, {:.1f} ms",
        num_optimized_frames, num_keyframes(), merged->updated_poses.size() - num_optimized_frames,
        num_optimized_points, moved.size() - num_optimized_points, map_epoch_, detail::MillisecondsSince(start));
    observable_.Notify(merged);
}

// Mapper is explicitly instantiated in mapper.cpp; the members defined here
// are instantiated here.
#define INSTANTIATE_MAPPER_GLOBAL_BA(FE)                                                              \
    template void Mapper<FE>::OnGlobalBaFinished(const events::GlobalBAFinished&);                    \
    template void Mapper<FE>::MergeGlobalBa(                                                          \
        const std::vector<std::pair<map::FrameId, geometry::RigidTransformation>>&,                   \
        const std::vector<std::pair<map::MapPointId, geometry::TPoint3DF>>&);

INSTANTIATE_MAPPER_GLOBAL_BA(features::SIFTFeatureExtractor)
INSTANTIATE_MAPPER_GLOBAL_BA(features::OrbFeatureExtractor)

} // namespace voyahslam::mapping
