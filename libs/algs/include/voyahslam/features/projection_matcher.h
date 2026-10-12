#ifndef VOYAHSLAM_FEATURES_PROJECTION_MATCHER_H
#define VOYAHSLAM_FEATURES_PROJECTION_MATCHER_H

#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <numbers>
#include <optional>
#include <span>
#include <utility>
#include <vector>
#include <voyahslam/defs.h>
#include <voyahslam/features/feature_grid.h>
#include <voyahslam/features/measurement.h>
#include <voyahslam/features/rotation_histogram.h>
#include <voyahslam/geometry/camera.h>
#include <voyahslam/geometry/rigid_transformation.h>
#include <voyahslam/geometry/sim3.h>
#include <voyahslam/map/map_point_id.h>
#include <voyahslam/map/point_reference.h>

namespace voyahslam::features {

struct ProjectionMatcherOptions {
    double max_distance_last = 100.0;  // ORB-SLAM3 TH_HIGH for frame-to-frame
    double max_distance_map = 100.0;   // TH_HIGH for the local map
    double nn_ratio_map = 0.8;
    bool check_orientation = true;     // frame-to-frame only, histogram as in the init matcher
    size_t histogram_bins = 12;
    double level_scale = 1.2;
    int max_level = 7;                 // ORB 8 levels
    double max_ray_angle_deg = 85.0;   // reject projections beyond the measurement's 85° limit
    double scale_slack = 0.2;          // SearchBySim3Projection: distance in [(1 - slack) min, (1 + slack) max]
};

// The point source of both searches (TPointSource) is duck-typed on
// tracking::MapReplica's interface: num_points(), IsAlive(id), and the
// unchecked accessors Position(id), Descriptor(id), Normal(id) and
// Reference(id) (a const std::optional<map::PointReference>&), which are
// called only for alive ids. features/ does not include tracking/.
// Binocular measurements pass left(): right matches come from stereo matching.
// Projections, the bounding box, the grid and the windows are in raw
// (distorted) pixels, as ORB-SLAM3's.

namespace detail {

// Raw pixel of camera point Xc, or nullopt when Xc is behind the camera,
// beyond the ray-angle limit or outside the current raw points' bounding box.
// Refers to `camera` (no shared_ptr copy of its distortion model): every gate
// is a local of a search (here and in FuseMatches) whose measurement argument
// owns the camera, so the camera outlives it.
class ProjectionGate {
public:
    ProjectionGate(const geometry::Camera& camera, const geometry::TPoints2DF& points,
                   double max_ray_angle_deg)
        : camera_(camera), min_cos_ray_(std::cos(max_ray_angle_deg * std::numbers::pi / 180.0)) {
        if (points.cols() > 0) {
            min_ = points.rowwise().minCoeff();
            max_ = points.rowwise().maxCoeff();
        } else {
            min_.setConstant(std::numeric_limits<precision_t>::infinity());
            max_.setConstant(-std::numeric_limits<precision_t>::infinity());
        }
    }

    // A temporary camera would dangle.
    ProjectionGate(geometry::Camera&&, const geometry::TPoints2DF&, double) = delete;

    [[nodiscard]] std::optional<geometry::TPoint2DF> Project(const geometry::TPoint3DF& Xc) const {
        if (Xc.z() <= 0.0 || Xc.z() < min_cos_ray_ * Xc.norm()) {
            return std::nullopt;
        }
        const geometry::TPoint2DF uv = camera_.Project(Xc);
        if ((uv.array() < min_.array()).any() || (uv.array() > max_.array()).any()) {
            return std::nullopt;
        }
        return uv;
    }

private:
    const geometry::Camera& camera_;
    double min_cos_ray_;
    geometry::TPoint2DF min_;
    geometry::TPoint2DF max_;
};

// Pyramid level at which a point at `distance` from the camera is expected,
// from its reference observation: the reference level shifted by the log-scale
// distance ratio, clamped to [0, max_level].
[[nodiscard]] inline int PredictLevel(const map::PointReference& reference, double distance, double level_scale,
                                      int max_level) {
    const double shift = std::log(reference.distance / distance) / std::log(level_scale);
    return std::clamp(reference.level + static_cast<int>(std::lround(shift)), 0, max_level);
}

// One direction of SearchBySim3Projection: per `from` feature, the `to` feature its point
// matches through S_to_from (from-camera to to-camera coordinates), or -1.
template <typename TFeatureExtractor>
[[nodiscard]] std::vector<Eigen::Index> SearchSim3Direction(
    const TMonocularMeasurement<TFeatureExtractor>& from, const geometry::RigidTransformation& T_from_w,
    const std::vector<std::optional<map::MapPointId>>& points_from,
    const std::vector<geometry::TPoint3DF>& positions_from, const TMonocularMeasurement<TFeatureExtractor>& to,
    const geometry::Sim3& S_to_from, double th, int max_distance, const ProjectionMatcherOptions& options) {
    const Eigen::Index num_from = from.points().cols();
    assert(static_cast<Eigen::Index>(points_from.size()) == num_from);
    assert(positions_from.size() == points_from.size());
    std::vector<Eigen::Index> matches(static_cast<size_t>(num_from), -1);

    const FeatureGrid grid(to.distorted_points(), to.levels());
    const ProjectionGate gate(to.camera(), to.distorted_points(), options.max_ray_angle_deg);
    const double max_level_scale = std::pow(options.level_scale, options.max_level);
    for (Eigen::Index i = 0; i < num_from; ++i) {
        const auto index = static_cast<size_t>(i);
        if (!points_from[index]) {
            continue;
        }
        const geometry::TPoint3DF X_from = T_from_w.Transform(positions_from[index]);
        const geometry::TPoint3DF X_to = geometry::Apply(S_to_from, X_from);
        const std::optional<geometry::TPoint2DF> uv = gate.Project(X_to);
        if (!uv) {
            continue;
        }
        // The `from` observation is the scale reference (ORB-SLAM3 uses the point's).
        const map::PointReference reference{{}, i, from.levels()(i), X_from.norm()};
        const double distance = X_to.norm();
        const double max_scale_distance = reference.distance * std::pow(options.level_scale, reference.level);
        const double min_scale_distance = max_scale_distance / max_level_scale;
        if (distance < (1.0 - options.scale_slack) * min_scale_distance ||
            distance > (1.0 + options.scale_slack) * max_scale_distance) {
            continue;
        }
        const int level = PredictLevel(reference, distance, options.level_scale, options.max_level);
        const double radius = th * std::pow(options.level_scale, level);
        const typename TFeatureExtractor::TDescriptor descriptor = from.descriptors().col(i);
        double best = std::numeric_limits<double>::infinity();
        Eigen::Index best_index = -1;
        for (const Eigen::Index j : grid.FeaturesInArea(*uv, radius, level - 1, level)) {
            const typename TFeatureExtractor::TDescriptor candidate = to.descriptors().col(j);
            const double d = static_cast<double>(TFeatureExtractor::Distance(descriptor, candidate));
            if (d < best) {
                best = d;
                best_index = j;
            }
        }
        if (best_index >= 0 && best <= max_distance) {
            matches[index] = best_index;
        }
    }
    return matches;
}

}  // namespace detail

// ORB-SLAM3 ORBmatcher::SearchByProjection(Frame&, const Frame& LastFrame):
// projects the alive points matched in `last` (last_matches[i] = MapPointId
// value of last feature i, or -1) into `current` at T_cw and matches each
// within radius_px * level_scale^l over levels [l - 1, l + 1], l the last
// feature's level. Claims are one-to-one, the closer claim wins; then the
// rotation histogram drops inconsistent matches. Returns, per current
// feature, the MapPointId value or -1.
template <typename TFeatureExtractor, typename TPointSource>
[[nodiscard]] std::vector<int64_t> SearchFromLastFrame(
    const TMonocularMeasurement<TFeatureExtractor>& current, const geometry::RigidTransformation& T_cw,
    const TMonocularMeasurement<TFeatureExtractor>& last, const std::vector<int64_t>& last_matches,
    const TPointSource& points, double radius_px, const ProjectionMatcherOptions& options) {
    const Eigen::Index num_current = current.points().cols();
    assert(static_cast<Eigen::Index>(last_matches.size()) == last.points().cols());
    assert(options.histogram_bins > 0 || !options.check_orientation);
    std::vector<int64_t> matches(static_cast<size_t>(num_current), -1);
    constexpr double kInfinity = std::numeric_limits<double>::infinity();
    std::vector<double> matched_distance(static_cast<size_t>(num_current), kInfinity);
    std::vector<Eigen::Index> claimed_by(static_cast<size_t>(num_current), -1);  // last feature

    const FeatureGrid grid(current.distorted_points(), current.levels());
    const detail::ProjectionGate gate(current.camera(), current.distorted_points(), options.max_ray_angle_deg);

    for (size_t i = 0; i < last_matches.size(); ++i) {
        if (last_matches[i] < 0 || last_matches[i] > std::numeric_limits<uint32_t>::max()) {
            continue;
        }
        const map::MapPointId id{static_cast<uint32_t>(last_matches[i])};
        if (!points.IsAlive(id)) {
            continue;
        }
        const std::optional<geometry::TPoint2DF> uv =
            gate.Project(T_cw.Transform(geometry::TPoint3DF(points.Position(id))));
        if (!uv) {
            continue;
        }
        const int level = last.levels()(static_cast<Eigen::Index>(i));
        const double radius = radius_px * std::pow(options.level_scale, level);
        const std::vector<Eigen::Index> candidates = grid.FeaturesInArea(*uv, radius, level - 1, level + 1);

        const typename TFeatureExtractor::TDescriptor descriptor = points.Descriptor(id);
        double best = kInfinity;
        Eigen::Index best_index = -1;
        for (const Eigen::Index j : candidates) {
            const typename TFeatureExtractor::TDescriptor candidate = current.descriptors().col(j);
            const double distance = static_cast<double>(TFeatureExtractor::Distance(descriptor, candidate));
            if (matched_distance[static_cast<size_t>(j)] <= distance) {
                continue;  // claimed at least as closely
            }
            if (distance < best) {
                best = distance;
                best_index = j;
            }
        }
        if (best_index < 0 || best > options.max_distance_last) {
            continue;
        }
        const auto j = static_cast<size_t>(best_index);
        matches[j] = last_matches[i];
        matched_distance[j] = best;
        claimed_by[j] = static_cast<Eigen::Index>(i);
    }

    if (options.check_orientation) {
        // Built from the final claims, so a stolen claim leaves no stale entry.
        RotationHistogram histogram(options.histogram_bins);
        for (Eigen::Index j = 0; j < num_current; ++j) {
            const Eigen::Index i = claimed_by[static_cast<size_t>(j)];
            if (i >= 0) {
                histogram.Add(static_cast<double>(last.angles()(i)) - static_cast<double>(current.angles()(j)), j);
            }
        }
        for (const Eigen::Index j : histogram.Inconsistent()) {
            matches[static_cast<size_t>(j)] = -1;
        }
    }
    return matches;
}

// ORB-SLAM3 ORBmatcher::SearchByProjection(Frame&, const vector<MapPoint*>&)
// with Frame::isInFrustum: matches every alive point with a reference that is
// not yet in out_matches (per current feature, MapPointId value or -1) to an
// unmatched current feature. Visible: in front, within the ray-angle limit
// and the bounding box, and cos(view, normal) > 0.5. The level is predicted
// from the reference observation; the window is
// th * (cos > 0.998 ? 2.5 : 4.0) * level_scale^l over levels [l - 1, l].
// The best match is accepted at <= max_distance_map if the second best is on
// another level or best < nn_ratio_map * second. Returns the number added.
// If given, out_visible gets appended, in id order and once each, the alive
// points already in out_matches (as ORB-SLAM3 SearchLocalPoints, not
// re-projected) and the points with a reference that pass the visibility test
// above, matched or not. No predicted-scale (distance) bound is applied.
template <typename TFeatureExtractor, typename TPointSource>
size_t SearchFromLocalMap(const TMonocularMeasurement<TFeatureExtractor>& current,
                          const geometry::RigidTransformation& T_cw, const TPointSource& points,
                          std::vector<int64_t>& out_matches, double th,
                          const ProjectionMatcherOptions& options,
                          std::vector<map::MapPointId>* out_visible = nullptr) {
    const Eigen::Index num_current = current.points().cols();
    assert(static_cast<Eigen::Index>(out_matches.size()) == num_current);
    const size_t num_points = points.num_points();
    std::vector<bool> already_matched(num_points, false);
    for (const int64_t id : out_matches) {
        if (id >= 0 && static_cast<uint64_t>(id) < num_points) {
            already_matched[static_cast<size_t>(id)] = true;
        }
    }

    const FeatureGrid grid(current.distorted_points(), current.levels());
    const detail::ProjectionGate gate(current.camera(), current.distorted_points(), options.max_ray_angle_deg);
    const geometry::TPoint3DF centre = T_cw.Inverse().translation();
    constexpr double kInfinity = std::numeric_limits<double>::infinity();

    size_t added = 0;
    for (size_t p = 0; p < num_points; ++p) {
        const map::MapPointId id{static_cast<uint32_t>(p)};
        if (!points.IsAlive(id)) {
            continue;
        }
        if (already_matched[p]) {
            if (out_visible) {
                out_visible->push_back(id);
            }
            continue;
        }
        const auto& reference = points.Reference(id);
        if (!reference) {
            continue;  // not yet observed by a keyframe: no level to predict from
        }
        const geometry::TPoint3DF X = points.Position(id);
        const std::optional<geometry::TPoint2DF> uv = gate.Project(T_cw.Transform(X));
        if (!uv) {
            continue;
        }
        const geometry::TPoint3DF view = X - centre;
        const geometry::TPoint3DF normal = points.Normal(id);
        const double distance = view.norm();
        const double view_cos = view.dot(normal) / (distance * normal.norm());
        if (!(view_cos > 0.5)) {
            continue;
        }
        if (out_visible) {
            out_visible->push_back(id);
        }
        const int level = detail::PredictLevel(*reference, distance, options.level_scale, options.max_level);
        const double radius =
            th * (view_cos > 0.998 ? 2.5 : 4.0) * std::pow(options.level_scale, level);
        const std::vector<Eigen::Index> candidates = grid.FeaturesInArea(*uv, radius, level - 1, level);

        const typename TFeatureExtractor::TDescriptor descriptor = points.Descriptor(id);
        double best = kInfinity;
        double second = kInfinity;
        int best_level = -1;
        int second_level = -1;
        Eigen::Index best_index = -1;
        for (const Eigen::Index j : candidates) {
            if (out_matches[static_cast<size_t>(j)] >= 0) {
                continue;
            }
            const typename TFeatureExtractor::TDescriptor candidate = current.descriptors().col(j);
            const double d = static_cast<double>(TFeatureExtractor::Distance(descriptor, candidate));
            const int candidate_level = current.levels()(j);
            if (d < best) {
                second = best;
                second_level = best_level;
                best = d;
                best_level = candidate_level;
                best_index = j;
            } else if (d < second) {
                second = d;
                second_level = candidate_level;
            }
        }
        if (best_index < 0 || best > options.max_distance_map) {
            continue;
        }
        if (best_level == second_level && !(best < options.nn_ratio_map * second)) {
            continue;
        }
        out_matches[static_cast<size_t>(best_index)] = static_cast<int64_t>(p);
        ++added;
    }
    return added;
}

// ORB-SLAM3 ORBmatcher::SearchByProjection(Frame&, KeyFrame*, const
// set<MapPoint*>&, th, ORBdist), relocalization's projection passes: projects
// the alive points of one map Frame (frame_observations: (feature of
// `frame`, point)) that are not yet in out_matches (per current feature,
// MapPointId value or -1) and matches each to an unmatched current feature
// within radius th * level_scale^l over levels [l - 1, l + 1], l the level
// predicted from the point's reference. The best match is accepted at
// <= max_distance, with no ratio test; a current feature goes to the first
// point claiming it. Points behind the camera, beyond the ray-angle limit or
// outside the bounding box are skipped; there is no viewing-angle or
// scale-distance gate. Then, with check_orientation, the rotation histogram
// (the Frame feature's angle - the current one) drops inconsistent new
// matches. Returns the number added.
template <typename TFeatureExtractor, typename TPointSource>
size_t SearchFromFrame(const TMonocularMeasurement<TFeatureExtractor>& current,
                       const geometry::RigidTransformation& T_cw,
                       const TMonocularMeasurement<TFeatureExtractor>& frame,
                       std::span<const std::pair<uint32_t, map::MapPointId>> frame_observations,
                       const TPointSource& points, std::vector<int64_t>& out_matches, double th,
                       double max_distance, const ProjectionMatcherOptions& options) {
    const Eigen::Index num_current = current.points().cols();
    assert(static_cast<Eigen::Index>(out_matches.size()) == num_current);
    assert(options.histogram_bins > 0 || !options.check_orientation);
    const size_t num_points = points.num_points();
    std::vector<bool> already_matched(num_points, false);
    for (const int64_t id : out_matches) {
        if (id >= 0 && static_cast<uint64_t>(id) < num_points) {
            already_matched[static_cast<size_t>(id)] = true;
        }
    }

    const FeatureGrid grid(current.distorted_points(), current.levels());
    const detail::ProjectionGate gate(current.camera(), current.distorted_points(), options.max_ray_angle_deg);
    const geometry::TPoint3DF centre = T_cw.Inverse().translation();
    std::vector<Eigen::Index> frame_feature_of(static_cast<size_t>(num_current), -1);  // new matches only
    size_t added = 0;
    for (const auto& [frame_feature, id] : frame_observations) {
        if (id.value >= num_points || already_matched[id.value] || !points.IsAlive(id)) {
            continue;
        }
        const auto& reference = points.Reference(id);
        if (!reference) {
            continue;
        }
        const geometry::TPoint3DF X = points.Position(id);
        const std::optional<geometry::TPoint2DF> uv = gate.Project(T_cw.Transform(X));
        if (!uv) {
            continue;
        }
        const int level =
            detail::PredictLevel(*reference, (X - centre).norm(), options.level_scale, options.max_level);
        const double radius = th * std::pow(options.level_scale, level);
        const typename TFeatureExtractor::TDescriptor descriptor = points.Descriptor(id);
        double best = std::numeric_limits<double>::infinity();
        Eigen::Index best_index = -1;
        for (const Eigen::Index j : grid.FeaturesInArea(*uv, radius, level - 1, level + 1)) {
            if (out_matches[static_cast<size_t>(j)] >= 0) {
                continue;
            }
            const typename TFeatureExtractor::TDescriptor candidate = current.descriptors().col(j);
            const double d = static_cast<double>(TFeatureExtractor::Distance(descriptor, candidate));
            if (d < best) {
                best = d;
                best_index = j;
            }
        }
        if (best_index < 0 || best > max_distance) {
            continue;
        }
        out_matches[static_cast<size_t>(best_index)] = static_cast<int64_t>(id.value);
        already_matched[id.value] = true;
        frame_feature_of[static_cast<size_t>(best_index)] = static_cast<Eigen::Index>(frame_feature);
        ++added;
    }

    if (options.check_orientation) {
        RotationHistogram histogram(options.histogram_bins);
        for (Eigen::Index j = 0; j < num_current; ++j) {
            const Eigen::Index i = frame_feature_of[static_cast<size_t>(j)];
            if (i >= 0) {
                histogram.Add(static_cast<double>(frame.angles()(i)) - static_cast<double>(current.angles()(j)), j);
            }
        }
        for (const Eigen::Index j : histogram.Inconsistent()) {
            out_matches[static_cast<size_t>(j)] = -1;
            --added;
        }
    }
    return added;
}

// ORB-SLAM3 ORBmatcher::SearchBySim3 between two map Frames whose camera coordinates S_12 maps
// 2 -> 1 (x1 = S_12 x2; geometry::SolveSim3Ransac's convention). points_k / positions_k give,
// per feature of frame_k, the point it observes (nullopt: none) and its world position; T_kw
// maps world to camera k. Each point of Frame 1 is projected into Frame 2 through
// S_12⁻¹ T1w and each point of Frame 2 into Frame 1 through S_12 T2w (raw pixels: behind the
// camera, beyond the ray-angle limit or outside the raw-point bounding box is skipped). The
// observing feature is the scale reference (level l_ref at distance d_ref = its camera
// distance): the projected distance d must lie within [(1 - scale_slack) d_min,
// (1 + scale_slack) d_max], d_max = d_ref * level_scale^l_ref, d_min = d_max /
// level_scale^max_level, and predicts the level l. The point's feature descriptor is matched to
// the closest feature, observing a point or not, at levels [l - 1, l] within radius *
// level_scale^l px, accepted at <= max_distance. Returns the mutual matches (feature 1,
// feature 2), Frame 1's feature order: 1's point picks 2's feature and 2's point picks it back.
template <typename TFeatureExtractor>
[[nodiscard]] std::vector<std::pair<uint32_t, uint32_t>> SearchBySim3Projection(
    const TMonocularMeasurement<TFeatureExtractor>& frame1, const geometry::RigidTransformation& T1w,
    const std::vector<std::optional<map::MapPointId>>& points1, const std::vector<geometry::TPoint3DF>& positions1,
    const TMonocularMeasurement<TFeatureExtractor>& frame2, const geometry::RigidTransformation& T2w,
    const std::vector<std::optional<map::MapPointId>>& points2, const std::vector<geometry::TPoint3DF>& positions2,
    const geometry::Sim3& S_12, double radius, int max_distance, const ProjectionMatcherOptions& options = {}) {
    const std::vector<Eigen::Index> match12 = detail::SearchSim3Direction<TFeatureExtractor>(
        frame1, T1w, points1, positions1, frame2, geometry::Inverse(S_12), radius, max_distance, options);
    const std::vector<Eigen::Index> match21 = detail::SearchSim3Direction<TFeatureExtractor>(
        frame2, T2w, points2, positions2, frame1, S_12, radius, max_distance, options);
    std::vector<std::pair<uint32_t, uint32_t>> matches;
    for (size_t i1 = 0; i1 < match12.size(); ++i1) {
        const Eigen::Index i2 = match12[i1];
        if (i2 >= 0 && match21[static_cast<size_t>(i2)] == static_cast<Eigen::Index>(i1)) {
            matches.emplace_back(static_cast<uint32_t>(i1), static_cast<uint32_t>(i2));
        }
    }
    return matches;
}

// ORB-SLAM3 ORBmatcher::SearchByProjection(KeyFrame*, Sim3 Scw, vpPoints, vpMatched, th,
// ratioHamming), loop detection's projection: projects each alive `candidates` point with a
// reference that is not yet in out_matches (per current feature, MapPointId value or -1) into
// `current` through S_cw, as the rigid [R, t / s] (the camera in the points' units), and matches
// it to the closest unmatched current feature at levels [l - 1, l] within th * level_scale^l px,
// l the level predicted from its reference, accepted at <= max_distance. Skipped: behind the
// camera, beyond the ray-angle limit, outside the raw-point bounding box, a camera distance
// outside the reference's scale-invariance range [(1 - scale_slack) d_min, (1 + scale_slack)
// d_max] (d_max = d_ref * level_scale^l_ref, d_min = d_max / level_scale^max_level), or a
// viewing direction more than 60 deg from the point's normal. No orientation check. Returns the
// number added.
template <typename TFeatureExtractor, typename TPointSource>
size_t SearchPointsBySim3(const TMonocularMeasurement<TFeatureExtractor>& current, const geometry::Sim3& S_cw,
                          std::span<const map::MapPointId> candidates, const TPointSource& points,
                          std::vector<int64_t>& out_matches, double th, double max_distance,
                          const ProjectionMatcherOptions& options) {
    assert(static_cast<Eigen::Index>(out_matches.size()) == current.points().cols());
    const size_t num_points = points.num_points();
    std::vector<bool> already_matched(num_points, false);
    for (const int64_t id : out_matches) {
        if (id >= 0 && static_cast<uint64_t>(id) < num_points) {
            already_matched[static_cast<size_t>(id)] = true;
        }
    }
    const geometry::RigidTransformation T_cw(S_cw.R, S_cw.t / S_cw.scale);
    const geometry::TPoint3DF centre = T_cw.Inverse().translation();
    const FeatureGrid grid(current.distorted_points(), current.levels());
    const detail::ProjectionGate gate(current.camera(), current.distorted_points(), options.max_ray_angle_deg);
    const double max_level_scale = std::pow(options.level_scale, options.max_level);

    size_t added = 0;
    for (const map::MapPointId id : candidates) {
        if (id.value >= num_points || already_matched[id.value] || !points.IsAlive(id)) {
            continue;
        }
        const auto& reference = points.Reference(id);
        if (!reference) {
            continue;
        }
        const geometry::TPoint3DF X = points.Position(id);
        const std::optional<geometry::TPoint2DF> uv = gate.Project(T_cw.Transform(X));
        if (!uv) {
            continue;
        }
        const geometry::TPoint3DF view = X - centre;
        const double distance = view.norm();
        const double max_scale_distance = reference->distance * std::pow(options.level_scale, reference->level);
        const double min_scale_distance = max_scale_distance / max_level_scale;
        if (distance < (1.0 - options.scale_slack) * min_scale_distance ||
            distance > (1.0 + options.scale_slack) * max_scale_distance) {
            continue;
        }
        const geometry::TPoint3DF normal = points.Normal(id);
        if (!(view.dot(normal) >= 0.5 * distance * normal.norm())) {
            continue;
        }
        const int level = detail::PredictLevel(*reference, distance, options.level_scale, options.max_level);
        const double radius = th * std::pow(options.level_scale, level);
        const typename TFeatureExtractor::TDescriptor descriptor = points.Descriptor(id);
        double best = std::numeric_limits<double>::infinity();
        Eigen::Index best_index = -1;
        for (const Eigen::Index j : grid.FeaturesInArea(*uv, radius, level - 1, level)) {
            if (out_matches[static_cast<size_t>(j)] >= 0) {
                continue;
            }
            const typename TFeatureExtractor::TDescriptor candidate = current.descriptors().col(j);
            const double d = static_cast<double>(TFeatureExtractor::Distance(descriptor, candidate));
            if (d < best) {
                best = d;
                best_index = j;
            }
        }
        if (best_index < 0 || best > max_distance) {
            continue;
        }
        out_matches[static_cast<size_t>(best_index)] = static_cast<int64_t>(id.value);
        already_matched[id.value] = true;
        ++added;
    }
    return added;
}

}  // namespace voyahslam::features

#endif  // VOYAHSLAM_FEATURES_PROJECTION_MATCHER_H
