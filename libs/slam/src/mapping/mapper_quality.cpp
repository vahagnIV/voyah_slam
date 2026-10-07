// The Mapper's map-quality steps around a new Frame (spec sections 3, 4.1):
// recent-point culling, fusion with the neighbours, and the final shape of
// transaction 1.
#include <algorithm>
#include <cassert>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <unordered_set>
#include <utility>
#include <vector>
#include <voyahslam/features/fuse_matcher.h>
#include <voyahslam/features/instantiation.h>
#include <voyahslam/mapping/map_quality.h>
#include <voyahslam/mapping/mapper.h>
#include "mapping/mapper_internal.h"

namespace voyahslam::mapping {

template <typename TFeatureExtractor>
void Mapper<TFeatureExtractor>::CullRecentPoints(
    events::MapTransaction<TFeatureExtractor>& out_event) {
    // ORB-SLAM3 LocalMapping::MapPointCulling: the new Frame, with its
    // tracked observations already added, is the current one.
    const map::FrameId current = keyframes_.back().id();
    std::erase_if(recent_points_, [&](const RecentPoint& recent) {
        if (!map_points_.IsValid(recent.id)) {
            return true;  // merged away
        }
        RecentPointStats stats;
        stats.frames_since_creation = current.value - recent.created.value;
        stats.visible = num_visible_[recent.id.value];
        stats.found = num_found_[recent.id.value];
        stats.weighted_obs = WeightedObservations(recent.id);
        stats.stereo = recent.stereo;
        switch (JudgeRecentPoint(stats, options_.map_quality)) {
            case RecentPointVerdict::Keep: return false;
            case RecentPointVerdict::Graduate: return true;
            case RecentPointVerdict::Remove: break;
        }
        RemovePoint(recent.id, out_event);
        return true;
    });
}

template <typename TFeatureExtractor>
std::vector<map::FrameId> Mapper<TFeatureExtractor>::FusionTargets(const TKeyFrame& keyframe) const {
    const bool stereo = detail::BinocularOf<TFeatureExtractor>(keyframe.measurement()) != nullptr;
    const MapQualityOptions& options = options_.map_quality;
    std::vector<map::FrameId> targets;
    const auto add = [&](map::FrameId frame) {
        if (frame != keyframe.id() && std::find(targets.begin(), targets.end(), frame) == targets.end()) {
            targets.push_back(frame);
        }
    };
    for (const auto& [frame, shared] : covisibility_.Covisible(
             keyframe.id(), stereo ? options.fuse_neighbours_stereo : options.fuse_neighbours_mono)) {
        add(frame);
    }
    const size_t num_first_order = targets.size();
    for (size_t i = 0; i < num_first_order; ++i) {
        for (const auto& [frame, shared] : covisibility_.Covisible(targets[i], options.fuse_second_order)) {
            add(frame);
        }
    }
    return targets;
}

template <typename TFeatureExtractor>
void Mapper<TFeatureExtractor>::Fuse(const TKeyFrame& keyframe,
                                     events::MapTransaction<TFeatureExtractor>& out_event) {
    const std::vector<map::FrameId> targets = FusionTargets(keyframe);
    // Forward: the new Frame's points, as they stand after the previous
    // targets' merges, into each target.
    for (const map::FrameId target : targets) {
        const std::vector<map::MapPointId> points = covisibility_.Points(keyframe.id());
        FuseInto(*FindKeyFrame(target), points, out_event);
    }
    // Backward: the targets' points the new Frame does not observe, once
    // each, into the new Frame.
    std::vector<map::MapPointId> points;
    std::unordered_set<uint32_t> seen;
    for (const map::FrameId target : targets) {
        for (const map::MapPointId point : covisibility_.Points(target)) {
            if (!covisibility_.Contains(keyframe.id(), point) && seen.insert(point.value).second) {
                points.push_back(point);
            }
        }
    }
    FuseInto(keyframe, points, out_event);
}

template <typename TFeatureExtractor>
void Mapper<TFeatureExtractor>::FuseInto(const TKeyFrame& frame,
                                         const std::vector<map::MapPointId>& points,
                                         events::MapTransaction<TFeatureExtractor>& out_event) {
    if (points.empty()) {
        return;
    }
    std::vector<features::FuseCandidatePoint<TFeatureExtractor>> candidates;
    candidates.reserve(points.size());
    for (const map::MapPointId point : points) {
        assert(map_points_.IsValid(point) && references_[point.value]);
        candidates.push_back({point, map_points_.Position(point), map_points_.Normal(point),
                              map_points_.Descriptor(point), *references_[point.value]});
    }
    std::optional<features::StereoView> stereo;
    if (const auto* binocular = detail::BinocularOf<TFeatureExtractor>(frame.measurement())) {
        stereo = features::StereoView{&binocular->right().distorted_points(),
                                      &binocular->right_indices(),
                                      {binocular->right().camera(), binocular->rig().T_rl}};
    }
    std::vector<std::optional<map::MapPointId>> feature_points = FeaturePoints(frame);
    const std::vector<features::FuseDecision> decisions = features::FuseMatches<TFeatureExtractor>(
        detail::LeftOf<TFeatureExtractor>(frame.measurement()), frame.pose(), stereo, feature_points,
        candidates, options_.fuse);

    for (const features::FuseDecision& decision : decisions) {
        // A point merged away or removed since the candidates were taken is
        // skipped, not redirected to its winner, which this feature's gate
        // never tested (ORB-SLAM3 Fuse skips bad points). A point an earlier
        // merge moved into this Frame is skipped too.
        const map::MapPointId point = decision.point;
        if (!map_points_.IsValid(point) || covisibility_.Contains(frame.id(), point)) {
            continue;
        }
        // The feature's point now: earlier merges may have replaced or
        // dropped it.
        std::optional<map::MapPointId>& existing = feature_points[decision.feature];
        if (existing) {
            existing = Resolve(*existing);
            if (!map_points_.IsValid(*existing) || FeatureOf(frame.id(), *existing) != decision.feature) {
                existing.reset();
            }
        }
        if (existing) {
            existing = Merge(point, *existing, out_event);
        } else {
            AddObservation(frame, decision.feature, point, out_event);
            touched_.push_back(point);
            existing = point;
        }
    }
}

template <typename TFeatureExtractor>
map::MapPointId Mapper<TFeatureExtractor>::Merge(map::MapPointId a, map::MapPointId b,
                                                 events::MapTransaction<TFeatureExtractor>& out_event) {
    // ORB-SLAM3 MapPoint::Replace, the winner chosen as in Fuse.
    const size_t weight_a = WeightedObservations(a);
    const size_t weight_b = WeightedObservations(b);
    const bool a_wins = weight_a > weight_b || (weight_a == weight_b && a < b);
    const map::MapPointId winner = a_wins ? a : b;
    const map::MapPointId loser = a_wins ? b : a;

    // A copy: the loop changes the loser's observations.
    const std::vector<map::PointObservation> observations = covisibility_.Observations(loser);
    for (const map::PointObservation& observation : observations) {
        // Consumers hold this observation unless this transaction added it.
        if (!added_keys_.contains(detail::ObservationKey(observation.frame, loser))) {
            out_event.removed_observations.push_back({observation.frame, loser});
        }
        covisibility_.Remove(observation.frame, loser);
        if (!covisibility_.Contains(observation.frame, winner)) {
            AddObservation(*FindKeyFrame(observation.frame), observation.feature, winner, out_event);
        }
    }
    num_visible_[winner.value] += num_visible_[loser.value];
    num_found_[winner.value] += num_found_[loser.value];
    RemovePoint(loser, out_event);
    replaced_[loser.value] = winner;
    losers_.push_back(loser);
    touched_.push_back(winner);
    return winner;
}

template <typename TFeatureExtractor>
map::MapPointId Mapper<TFeatureExtractor>::Resolve(map::MapPointId point) const {
    for (auto it = replaced_.find(point.value); it != replaced_.end(); it = replaced_.find(point.value)) {
        point = it->second;
    }
    return point;
}

template <typename TFeatureExtractor>
std::optional<uint32_t> Mapper<TFeatureExtractor>::FeatureOf(map::FrameId frame,
                                                             map::MapPointId point) const {
    for (const map::PointObservation& observation : covisibility_.Observations(point)) {
        if (observation.frame == frame) {
            return observation.feature;
        }
    }
    return std::nullopt;
}

template <typename TFeatureExtractor>
std::vector<std::optional<map::MapPointId>> Mapper<TFeatureExtractor>::FeaturePoints(
    const TKeyFrame& frame) const {
    std::vector<std::optional<map::MapPointId>> points(static_cast<size_t>(
        detail::LeftOf<TFeatureExtractor>(frame.measurement()).points().cols()));
    for (const map::MapPointId point : covisibility_.Points(frame.id())) {
        points[*FeatureOf(frame.id(), point)] = point;
    }
    return points;
}

template <typename TFeatureExtractor>
void Mapper<TFeatureExtractor>::FinishTransaction(
    map::MapPointId first_new, events::MapTransaction<TFeatureExtractor>& out_event) {
    // An observation record is kept, with its point resolved to the final
    // winner, iff that point is alive and observed through the record's
    // feature; once per (Frame, point).
    std::unordered_set<uint64_t> kept;
    const auto keep = [&](map::FrameId frame, uint32_t feature, map::MapPointId& point) {
        point = Resolve(point);
        return map_points_.IsValid(point) && FeatureOf(frame, point) == feature &&
               kept.insert(detail::ObservationKey(frame, point)).second;
    };
    events::NewFrame<TFeatureExtractor>& part = out_event.added_frames.back();
    size_t count = 0;
    for (size_t i = 0; i < part.observed_points.size(); ++i) {
        map::MapPointId point = part.observed_points[i];
        if (keep(part.id, part.observed_features[i], point)) {
            part.observed_features[count] = part.observed_features[i];
            part.observed_points[count] = point;
            ++count;
        }
    }
    part.observed_features.resize(count);
    part.observed_points.resize(count);
    std::erase_if(out_event.added_observations, [&](events::TransactionObservation& added) {
        return !keep(added.frame, added.feature, added.point);
    });

    for (const map::MapPointId loser : losers_) {
        out_event.replaced_points.push_back({loser, Resolve(loser)});
    }

    // The points whose observations changed, once each, alive.
    std::vector<map::MapPointId> updated;
    std::unordered_set<uint32_t> seen;
    for (map::MapPointId point : touched_) {
        point = Resolve(point);
        if (map_points_.IsValid(point) && seen.insert(point.value).second) {
            UpdatePointStatistics(point);
            if (point < first_new) {
                updated.push_back(point);
            }
        }
    }
    out_event.ResizeUpdatedPoints(static_cast<uint32_t>(updated.size()));
    for (size_t i = 0; i < updated.size(); ++i) {
        const auto col = static_cast<Eigen::Index>(i);
        out_event.updated_ids[i] = updated[i];
        out_event.updated_positions.col(col) = map_points_.Position(updated[i]);
        out_event.updated_normals.col(col) = map_points_.Normal(updated[i]);
        out_event.updated_descriptors.col(col) = map_points_.Descriptor(updated[i]);
    }

    // Every new id, merged away or not: the range is contiguous.
    const uint32_t num_new = map_points_.num_ids() - first_new.value;
    out_event.ResizeNewPoints(num_new);
    for (uint32_t i = 0; i < num_new; ++i) {
        const map::MapPointId point{first_new.value + i};
        out_event.new_positions.col(i) = map_points_.Position(point);
        out_event.new_descriptors.col(i) = map_points_.Descriptor(point);
        out_event.new_normals.col(i) = map_points_.Normal(point);
    }
}

// Mapper is explicitly instantiated in mapper.cpp; the members defined here
// are instantiated here.
#define INSTANTIATE_MAPPER_QUALITY(FE)                                                            \
    template void Mapper<FE>::CullRecentPoints(events::MapTransaction<FE>&);                      \
    template std::vector<map::FrameId> Mapper<FE>::FusionTargets(const TKeyFrame&) const;         \
    template void Mapper<FE>::Fuse(const TKeyFrame&, events::MapTransaction<FE>&);                \
    template void Mapper<FE>::FuseInto(const TKeyFrame&, const std::vector<map::MapPointId>&,     \
                                       events::MapTransaction<FE>&);                              \
    template map::MapPointId Mapper<FE>::Merge(map::MapPointId, map::MapPointId,                  \
                                               events::MapTransaction<FE>&);                      \
    template map::MapPointId Mapper<FE>::Resolve(map::MapPointId) const;                          \
    template std::optional<uint32_t> Mapper<FE>::FeatureOf(map::FrameId, map::MapPointId) const;  \
    template std::vector<std::optional<map::MapPointId>> Mapper<FE>::FeaturePoints(               \
        const TKeyFrame&) const;                                                                  \
    template void Mapper<FE>::FinishTransaction(map::MapPointId, events::MapTransaction<FE>&);

INSTANTIATE_MAPPER_QUALITY(features::SIFTFeatureExtractor)
INSTANTIATE_MAPPER_QUALITY(features::OrbFeatureExtractor)

} // namespace voyahslam::mapping
