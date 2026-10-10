// The Mapper's map-quality steps around a new Frame (spec sections 3-5):
// recent-point culling, fusion with the neighbours and the final shape of
// transaction 1; local BA and Frame culling, transaction 2.
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
#include <voyahslam/features/fuse_matcher.h>
#include <voyahslam/features/instantiation.h>
#include <voyahslam/mapping/map_quality.h>
#include <voyahslam/mapping/mapper.h>
#include <voyahslam/optimization/bundle_adjustment.h>
#include "mapping/mapper_internal.h"

namespace voyahslam::mapping {

template <typename TFeatureExtractor>
void Mapper<TFeatureExtractor>::CullRecentPoints(
    events::MapTransaction<TFeatureExtractor>& out_event) {
    // ORB-SLAM3 LocalMapping::MapPointCulling: the new Frame, with its
    // tracked observations already added, is the current one.
    const map::FrameId current = LastKeyFrame().id();
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
                                         events::MapTransaction<TFeatureExtractor>& out_event, bool loop) {
    if (points.empty()) {
        return;
    }
    // ORB-SLAM3 ORBmatcher::Fuse with a Sim3: radius th x scale factor, no
    // reprojection chi2, the left image only.
    features::FuseOptions options = options_.fuse;
    if (loop) {
        options.radius_factor = options_.loop.fuse_radius;
        options.chi2_mono = std::numeric_limits<double>::infinity();
        options.chi2_stereo = std::numeric_limits<double>::infinity();
    }
    std::vector<features::FuseCandidatePoint<TFeatureExtractor>> candidates;
    candidates.reserve(points.size());
    for (const map::MapPointId point : points) {
        assert(map_points_.IsValid(point) && references_[point.value]);
        candidates.push_back({point, map_points_.Position(point), map_points_.Normal(point),
                              map_points_.Descriptor(point), *references_[point.value]});
    }
    std::optional<features::StereoView> stereo;
    const auto* binocular = loop ? nullptr : detail::BinocularOf<TFeatureExtractor>(frame.measurement());
    if (binocular != nullptr) {
        stereo = features::StereoView{&binocular->right().distorted_points(),
                                      &binocular->right_indices(),
                                      {binocular->right().camera(), binocular->rig().T_rl}};
    }
    std::vector<std::optional<map::MapPointId>> feature_points = FeaturePoints(frame);
    const std::vector<features::FuseDecision> decisions = features::FuseMatches<TFeatureExtractor>(
        detail::LeftOf<TFeatureExtractor>(frame.measurement()), frame.pose(), stereo, feature_points,
        candidates, options);

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
        if (existing && loop) {
            Replace(*existing, point, out_event);
            existing = point;
        } else if (existing) {
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
    Replace(loser, winner, out_event);
    return winner;
}

template <typename TFeatureExtractor>
void Mapper<TFeatureExtractor>::Replace(map::MapPointId loser, map::MapPointId winner,
                                        events::MapTransaction<TFeatureExtractor>& out_event) {
    assert(loser != winner);
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
    ResolveObservationRecords(out_event);

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
    WriteUpdatedPoints(updated, out_event);

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

template <typename TFeatureExtractor>
void Mapper<TFeatureExtractor>::ResolveObservationRecords(
    events::MapTransaction<TFeatureExtractor>& out_event) {
    // An observation record is kept, with its point resolved to the final
    // winner, iff that point is alive and observed through the record's
    // feature; once per (Frame, point).
    std::unordered_set<uint64_t> kept;
    const auto keep = [&](map::FrameId frame, uint32_t feature, map::MapPointId& point) {
        point = Resolve(point);
        return map_points_.IsValid(point) && FeatureOf(frame, point) == feature &&
               kept.insert(detail::ObservationKey(frame, point)).second;
    };
    for (events::NewFrame<TFeatureExtractor>& part : out_event.added_frames) {
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
    }
    std::erase_if(out_event.added_observations, [&](events::TransactionObservation& added) {
        return !keep(added.frame, added.feature, added.point);
    });

    for (const map::MapPointId loser : losers_) {
        out_event.replaced_points.push_back({loser, Resolve(loser)});
    }
}

template <typename TFeatureExtractor>
void Mapper<TFeatureExtractor>::WriteUpdatedPoints(
    const std::vector<map::MapPointId>& points, events::MapTransaction<TFeatureExtractor>& out_event) const {
    out_event.ResizeUpdatedPoints(static_cast<uint32_t>(points.size()));
    for (size_t i = 0; i < points.size(); ++i) {
        const auto col = static_cast<Eigen::Index>(i);
        out_event.updated_ids[i] = points[i];
        out_event.updated_positions.col(col) = map_points_.Position(points[i]);
        out_event.updated_normals.col(col) = map_points_.Normal(points[i]);
        out_event.updated_descriptors.col(col) = map_points_.Descriptor(points[i]);
    }
}

template <typename TFeatureExtractor>
auto Mapper<TFeatureExtractor>::RefineAroundFrame(map::FrameId frame) -> Refinement {
    using Clock = std::chrono::steady_clock;
    auto* refined = CreateTransaction();
    Refinement refinement;
    // Points moved by BA or that lost an observation, with repeats.
    std::vector<map::MapPointId> changed;

    auto start = Clock::now();
    LocalBundleAdjust(frame, *refined, changed);
    refinement.ba_ms = detail::MillisecondsSince(start);

    start = Clock::now();
    CullFrames(frame, *refined, changed);
    refinement.cull_ms = detail::MillisecondsSince(start);
    refinement.num_culled_frames = refined->removed_frames.size();

    FinishRefinement(std::move(changed), *refined);
    refinement.num_removed_points = refined->removed_points.size();
    num_keyframes_.fetch_sub(refinement.num_culled_frames, std::memory_order_acq_rel);
    num_map_points_.store(map_points_.size(), std::memory_order_release);
    observable_.Notify(refined);
    return refinement;
}

template <typename TFeatureExtractor>
void Mapper<TFeatureExtractor>::LocalBundleAdjust(
    map::FrameId frame, events::MapTransaction<TFeatureExtractor>& out_event,
    std::vector<map::MapPointId>& out_changed) {
    const MapQualityOptions& options = options_.map_quality;
    last_local_ba_ = {};
    if (options.ba_max_optimized == 0) {
        return;
    }

    // The problem's Frames: first the optimized ones (the new Frame, then
    // its covisible Frames by shared points), then the other observers.
    std::vector<map::FrameId> frames = {frame};
    for (const auto& [covisible, shared] : covisibility_.Covisible(frame, options.ba_max_optimized - 1)) {
        if (shared >= options.ba_min_shared) {
            frames.push_back(covisible);
        }
    }
    const size_t num_optimized = frames.size();
    std::unordered_map<uint32_t, size_t> frame_index;
    for (size_t i = 0; i < frames.size(); ++i) {
        frame_index.emplace(frames[i].value, i);
    }
    std::vector<map::MapPointId> points;
    std::unordered_set<uint32_t> seen;
    for (size_t i = 0; i < num_optimized; ++i) {
        for (const map::MapPointId point : covisibility_.Points(frames[i])) {
            if (seen.insert(point.value).second) {
                points.push_back(point);
            }
        }
    }
    for (const map::MapPointId point : points) {
        for (const map::PointObservation& observation : covisibility_.Observations(point)) {
            if (frame_index.emplace(observation.frame.value, frames.size()).second) {
                frames.push_back(observation.frame);
            }
        }
    }

    optimization::BaProblem problem;
    const map::FrameId first = FirstKeyFrame().id();
    problem.keyframe_poses.reserve(frames.size());
    problem.keyframe_fixed.reserve(frames.size());
    problem.keyframe_cameras.reserve(frames.size());
    problem.keyframe_stereo.reserve(frames.size());
    for (size_t i = 0; i < frames.size(); ++i) {
        const TKeyFrame& keyframe = *FindKeyFrame(frames[i]);
        problem.keyframe_poses.push_back(keyframe.pose());
        problem.keyframe_fixed.push_back(i >= num_optimized || frames[i] == first);
        problem.keyframe_cameras.push_back(detail::LeftOf<TFeatureExtractor>(keyframe.measurement()).camera());
        problem.keyframe_stereo.push_back(detail::BaStereoOf<TFeatureExtractor>(keyframe.measurement()));
    }
    problem.points.resize(3, static_cast<Eigen::Index>(points.size()));
    // Per observation: its Frame and point, to apply the outlier verdicts.
    std::vector<std::pair<map::FrameId, map::MapPointId>> observed;
    bool has_free = false;
    for (size_t p = 0; p < points.size(); ++p) {
        problem.points.col(static_cast<Eigen::Index>(p)) = map_points_.Position(points[p]);
        for (const map::PointObservation& observation : covisibility_.Observations(points[p])) {
            const size_t k = frame_index.at(observation.frame.value);
            const TKeyFrame& keyframe = *FindKeyFrame(observation.frame);
            problem.observations.push_back(detail::BaObservationOf<TFeatureExtractor>(
                keyframe.measurement(), observation.feature, k, p, options_.fuse.sigma_px,
                options_.fuse.level_scale));
            observed.emplace_back(observation.frame, points[p]);
            has_free = has_free || !problem.keyframe_fixed[k];
        }
    }
    const bool has_fixed =
        std::find(problem.keyframe_fixed.begin(), problem.keyframe_fixed.end(), true) !=
        problem.keyframe_fixed.end();
    last_local_ba_.num_free = static_cast<size_t>(
        std::count(problem.keyframe_fixed.begin(), problem.keyframe_fixed.end(), false));
    last_local_ba_.num_fixed = frames.size() - last_local_ba_.num_free;
    last_local_ba_.num_points = points.size();
    last_local_ba_.num_observations = problem.observations.size();
    // Without a free Frame that has an observation there is nothing to
    // optimize; without a fixed one nothing anchors the map (ORB-SLAM3
    // aborts its local BA too).
    if (!has_free || !has_fixed) {
        return;
    }

    optimization::BundleAdjustmentOptions ba_options;
    ba_options.robust_iterations = options.ba_robust_iterations;
    ba_options.iterations = options.ba_iterations;
    // An urgent FrameTracked waiting in the inbox stops BA within one
    // iteration (spec section 5).
    ba_options.should_stop = [this] { return inbox_.pending_priority() > 0; };
    const optimization::BaResult result = optimization::BundleAdjust(problem, ba_options);
    if (!result.success) {
        log_->debug("Local BA of Frame {} failed; nothing applied", frame.value);
        return;
    }
    last_local_ba_.ran = true;
    last_local_ba_.interrupted = result.interrupted;
    last_local_ba_.stage_reached = result.stage_reached;
    last_local_ba_.iterations = result.iterations;

    // Poses and points always; outliers once the robust stage has finished.
    for (size_t i = 0; i < frames.size(); ++i) {
        if (!problem.keyframe_fixed[i]) {
            FindKeyFrame(frames[i])->mutable_pose() = result.keyframe_poses[i];
            out_event.updated_poses.emplace_back(frames[i], result.keyframe_poses[i]);
        }
    }
    for (size_t p = 0; p < points.size(); ++p) {
        map_points_.Position(points[p]) = result.points.col(static_cast<Eigen::Index>(p));
        out_changed.push_back(points[p]);
    }
    if (result.stage_reached < 1) {
        return;
    }
    for (size_t o = 0; o < observed.size(); ++o) {
        const auto& [observer, point] = observed[o];
        if (result.inlier[o] || !map_points_.IsValid(point)) {
            continue;
        }
        covisibility_.Remove(observer, point);
        out_event.removed_observations.push_back({observer, point});
        ++last_local_ba_.num_outliers;
        if (covisibility_.NumObservations(point) == 0) {
            RemovePoint(point, out_event);
        }
    }
}

template <typename TFeatureExtractor>
void Mapper<TFeatureExtractor>::CullFrames(map::FrameId frame,
                                           events::MapTransaction<TFeatureExtractor>& out_event,
                                           std::vector<map::MapPointId>& out_changed) {
    // ORB-SLAM3 LocalMapping::KeyFrameCulling, each candidate judged on the
    // map as the earlier ones left it. The first Frame (the tree's root) and
    // Frames with loop edges stay (ORB-SLAM3 KeyFrame::SetBadFlag).
    const map::FrameId first = FirstKeyFrame().id();
    for (const auto& [candidate, shared] :
         covisibility_.Covisible(frame, std::numeric_limits<size_t>::max())) {
        if (candidate == first || !FindRecord(candidate)->loop_edges.empty()) {
            continue;
        }
        const TKeyFrame& keyframe = *FindKeyFrame(candidate);
        const auto& levels = detail::LeftOf<TFeatureExtractor>(keyframe.measurement()).levels();
        const auto* binocular = detail::BinocularOf<TFeatureExtractor>(keyframe.measurement());
        const double close_depth =
            binocular ? options_.keyframe.close_depth_baselines * detail::RigBaseline(binocular->rig())
                      : 0.0;
        std::vector<FramePointStats> stats;
        for (const map::MapPointId point : covisibility_.Points(candidate)) {
            const std::vector<map::PointObservation>& observations = covisibility_.Observations(point);
            const auto own = std::find_if(observations.begin(), observations.end(),
                                          [&](const map::PointObservation& o) { return o.frame == candidate; });
            assert(own != observations.end());
            const auto f = static_cast<Eigen::Index>(own->feature);
            FramePointStats& point_stats = stats.emplace_back();
            point_stats.level = static_cast<int>(levels(f));
            // Stereo: only the close points count.
            if (binocular) {
                const double depth = binocular->depth(static_cast<int>(f));
                point_stats.counted =
                    binocular->right_indices()(f) >= 0 && depth > 0.0 && depth < close_depth;
            }
            for (const map::PointObservation& other : observations) {
                if (other.frame == candidate) {
                    continue;
                }
                const auto& other_levels =
                    detail::LeftOf<TFeatureExtractor>(FindKeyFrame(other.frame)->measurement()).levels();
                if (static_cast<int>(other_levels(static_cast<Eigen::Index>(other.feature))) <=
                    point_stats.level + 1) {
                    ++point_stats.other_observers_at_level;
                }
            }
        }
        if (IsRedundantFrame(stats, options_.map_quality)) {
            RemoveFrame(candidate, out_event, out_changed);
        }
    }
}

template <typename TFeatureExtractor>
void Mapper<TFeatureExtractor>::RemoveFrame(map::FrameId frame,
                                            events::MapTransaction<TFeatureExtractor>& out_event,
                                            std::vector<map::MapPointId>& out_changed) {
    const std::vector<map::MapPointId> points = covisibility_.Points(frame);
    covisibility_.RemoveFrame(frame);
    ReparentChildren(frame, out_event);
    keyframes_[frame.value - FirstKeyFrame().id().value].reset();
    out_event.removed_frames.push_back(frame);
    for (const map::MapPointId point : points) {
        if (covisibility_.NumObservations(point) < 2) {
            RemovePoint(point, out_event);
        } else {
            out_changed.push_back(point);
        }
    }
}

template <typename TFeatureExtractor>
void Mapper<TFeatureExtractor>::ReparentChildren(
    map::FrameId frame, events::MapTransaction<TFeatureExtractor>& out_event) {
    KeyFrameRecord& culled = *FindRecord(frame);
    assert(culled.parent.has_value() && culled.loop_edges.empty());
    const map::FrameId parent = *culled.parent;
    std::set<map::FrameId> children = std::move(culled.children);
    culled.children.clear();
    std::set<map::FrameId> candidates = {parent};
    // Each round attaches the (child, candidate) pair sharing the most points,
    // the lower child id and then the lower candidate id on a tie.
    while (!children.empty()) {
        std::optional<std::pair<map::FrameId, map::FrameId>> best;
        uint32_t best_shared = 0;
        for (const map::FrameId child : children) {
            // By shared points descending, then by id.
            for (const auto& [other, shared] :
                 covisibility_.Covisible(child, std::numeric_limits<size_t>::max())) {
                if (shared <= best_shared) {
                    break;
                }
                if (candidates.contains(other)) {
                    best = {child, other};
                    best_shared = shared;
                    break;
                }
            }
        }
        if (!best) {
            break;
        }
        ChangeParent(best->first, best->second, out_event);
        candidates.insert(best->first);
        children.erase(best->first);
    }
    for (const map::FrameId child : children) {
        ChangeParent(child, parent, out_event);
    }
    FindRecord(parent)->children.erase(frame);
}

template <typename TFeatureExtractor>
void Mapper<TFeatureExtractor>::FinishRefinement(
    std::vector<map::MapPointId> changed, events::MapTransaction<TFeatureExtractor>& out_event) {
    // A culled Frame's pose is not published.
    std::erase_if(out_event.updated_poses, [&](const auto& updated) {
        return FindKeyFrame(updated.first) == nullptr;
    });

    std::sort(changed.begin(), changed.end());
    changed.erase(std::unique(changed.begin(), changed.end()), changed.end());
    std::erase_if(changed, [&](map::MapPointId point) { return !map_points_.IsValid(point); });
    for (const map::MapPointId point : changed) {
        // The replica moves a reference when it removes the reference
        // observation, after this transaction's pose and point updates: the
        // remaining observation with the smallest Frame id.
        const map::PointReference& reference = *references_[point.value];
        if (!covisibility_.Contains(reference.keyframe, point)) {
            const std::vector<map::PointObservation>& observations = covisibility_.Observations(point);
            const auto next = std::min_element(
                observations.begin(), observations.end(),
                [](const map::PointObservation& a, const map::PointObservation& b) { return a.frame < b.frame; });
            SetReference(point, *FindKeyFrame(next->frame), next->feature);
        }
        UpdatePointStatistics(point);
    }
    WriteUpdatedPoints(changed, out_event);
}

// Mapper is explicitly instantiated in mapper.cpp; the members defined here
// are instantiated here.
#define INSTANTIATE_MAPPER_QUALITY(FE)                                                            \
    template void Mapper<FE>::CullRecentPoints(events::MapTransaction<FE>&);                      \
    template std::vector<map::FrameId> Mapper<FE>::FusionTargets(const TKeyFrame&) const;         \
    template void Mapper<FE>::Fuse(const TKeyFrame&, events::MapTransaction<FE>&);                \
    template void Mapper<FE>::FuseInto(const TKeyFrame&, const std::vector<map::MapPointId>&,     \
                                       events::MapTransaction<FE>&, bool);                        \
    template map::MapPointId Mapper<FE>::Merge(map::MapPointId, map::MapPointId,                  \
                                               events::MapTransaction<FE>&);                      \
    template void Mapper<FE>::Replace(map::MapPointId, map::MapPointId, events::MapTransaction<FE>&); \
    template map::MapPointId Mapper<FE>::Resolve(map::MapPointId) const;                          \
    template std::optional<uint32_t> Mapper<FE>::FeatureOf(map::FrameId, map::MapPointId) const;  \
    template std::vector<std::optional<map::MapPointId>> Mapper<FE>::FeaturePoints(               \
        const TKeyFrame&) const;                                                                  \
    template void Mapper<FE>::FinishTransaction(map::MapPointId, events::MapTransaction<FE>&);     \
    template void Mapper<FE>::ResolveObservationRecords(events::MapTransaction<FE>&);             \
    template void Mapper<FE>::WriteUpdatedPoints(const std::vector<map::MapPointId>&,             \
                                                 events::MapTransaction<FE>&) const;              \
    template Mapper<FE>::Refinement Mapper<FE>::RefineAroundFrame(map::FrameId);                  \
    template void Mapper<FE>::LocalBundleAdjust(map::FrameId, events::MapTransaction<FE>&,         \
                                                std::vector<map::MapPointId>&);                    \
    template void Mapper<FE>::CullFrames(map::FrameId, events::MapTransaction<FE>&,                \
                                         std::vector<map::MapPointId>&);                           \
    template void Mapper<FE>::RemoveFrame(map::FrameId, events::MapTransaction<FE>&,               \
                                          std::vector<map::MapPointId>&);                          \
    template void Mapper<FE>::ReparentChildren(map::FrameId, events::MapTransaction<FE>&);         \
    template void Mapper<FE>::FinishRefinement(std::vector<map::MapPointId>,                       \
                                               events::MapTransaction<FE>&);

INSTANTIATE_MAPPER_QUALITY(features::SIFTFeatureExtractor)
INSTANTIATE_MAPPER_QUALITY(features::OrbFeatureExtractor)

} // namespace voyahslam::mapping
