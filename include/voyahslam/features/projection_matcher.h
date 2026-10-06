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
#include <vector>
#include <voyahslam/defs.h>
#include <voyahslam/features/feature_grid.h>
#include <voyahslam/features/measurement.h>
#include <voyahslam/features/rotation_histogram.h>
#include <voyahslam/geometry/camera.h>
#include <voyahslam/geometry/rigid_transformation.h>
#include <voyahslam/map/map_point_id.h>

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
    geometry::Camera camera_;
    double min_cos_ray_;
    geometry::TPoint2DF min_;
    geometry::TPoint2DF max_;
};

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
    const double log_scale = std::log(options.level_scale);
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
        const int level = std::clamp(
            reference->level +
                static_cast<int>(std::lround(std::log(reference->distance / distance) / log_scale)),
            0, options.max_level);
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

}  // namespace voyahslam::features

#endif  // VOYAHSLAM_FEATURES_PROJECTION_MATCHER_H
