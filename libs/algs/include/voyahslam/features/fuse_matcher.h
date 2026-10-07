#ifndef VOYAHSLAM_FEATURES_FUSE_MATCHER_H
#define VOYAHSLAM_FEATURES_FUSE_MATCHER_H

#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <optional>
#include <vector>
#include <voyahslam/defs.h>
#include <voyahslam/features/feature_grid.h>
#include <voyahslam/features/measurement.h>
#include <voyahslam/features/projection_matcher.h>
#include <voyahslam/geometry/rigid_transformation.h>
#include <voyahslam/map/map_point_id.h>
#include <voyahslam/map/point_reference.h>
#include <voyahslam/optimization/pose_optimization.h>

namespace voyahslam::features {

// Thresholds of ORB-SLAM3 ORBmatcher::Fuse (spec §3).
struct FuseOptions {
    double radius_factor = 3.0;   // window radius_factor * level_scale^level raw px
    double chi2_mono = 5.991;     // 2 dof, 95 %
    double chi2_stereo = 7.815;   // 3 dof (left u, v + right u), 95 %
    int th_low = 50;              // max descriptor distance
    double scale_slack = 0.2;     // distance range [(1 - slack) min, (1 + slack) max]
    double min_view_cos = 0.5;    // cos(view ray, point normal)
    double max_ray_angle_deg = 85.0;
    double sigma_px = 1.0;        // sigma = sigma_px * level_scale^level of the feature
    double level_scale = 1.2;
    int max_level = 7;            // ORB 8 levels
};

// The right image of a binocular Frame: raw right points, the left -> right
// feature index map (-1 = no match) and the right camera with T_rl. The
// pointers must outlive the FuseMatches call.
struct StereoView {
    const geometry::TPoints2DF* right_points;
    const Eigen::VectorXi* right_indices;
    optimization::StereoCameras cameras;
};

// A map point projected into the Frame: world position, mean viewing
// direction (normal), representative descriptor and scale reference.
template <typename FE>
struct FuseCandidatePoint {
    map::MapPointId id;
    geometry::TPoint3DF position;
    geometry::TPoint3DF normal;
    typename FE::TDescriptor descriptor;
    map::PointReference reference;
};

// Point `point` matches Frame feature `feature`. `existing` is the point the
// feature already observes (a duplicate to merge), nullopt for a free feature
// (an observation to add).
struct FuseDecision {
    map::MapPointId point;
    uint32_t feature;
    std::optional<map::MapPointId> existing;
};

// ORB-SLAM3 ORBmatcher::Fuse(KeyFrame*, vector<MapPoint*>, th, bRight) for one
// Frame, as a pure function: projects each candidate into frame_left at T_cw
// (raw pixels through its Camera) and returns at most one decision per point.
// Gate per point: not observed by the Frame (feature_points, one entry per
// feature); in front, within the ray-angle limit and the raw-point bounding
// box; cos(view, normal) >= min_view_cos; distance within the reference's
// scale range +- scale_slack, which predicts the level l. Candidates are the
// features at levels [l - 1, l] within radius_factor * level_scale^l px whose
// raw-px chi2 (sigma from the feature's level) passes chi2_mono, or, for a
// feature with a right match in `stereo`, chi2_stereo over left u, v and
// right u (strictly below the threshold). The best descriptor distance must be
// <= th_low. When several points pick the same feature, the closest descriptor
// wins (the lower id on a tie); a losing point gets no decision at all, it does
// not fall back to its next-best feature. Decisions are in candidate order.
// Precondition: the ids in `points` are unique.
template <typename FE>
[[nodiscard]] std::vector<FuseDecision> FuseMatches(
    const TMonocularMeasurement<FE>& frame_left, const geometry::RigidTransformation& T_cw,
    const std::optional<StereoView>& stereo, const std::vector<std::optional<map::MapPointId>>& feature_points,
    const std::vector<FuseCandidatePoint<FE>>& points, const FuseOptions& options) {
    const Eigen::Index num_features = frame_left.points().cols();
    assert(static_cast<Eigen::Index>(feature_points.size()) == num_features);
    assert(!stereo || stereo->right_indices->size() == num_features);

    std::vector<map::MapPointId> observed;
    for (const std::optional<map::MapPointId>& id : feature_points) {
        if (id) {
            observed.push_back(*id);
        }
    }
    std::sort(observed.begin(), observed.end());

    const geometry::TPoints2DF& left_points = frame_left.distorted_points();
    const FeatureGrid grid(left_points, frame_left.levels());
    const detail::ProjectionGate gate(frame_left.camera(), left_points, options.max_ray_angle_deg);
    const geometry::TPoint3DF centre = T_cw.Inverse().translation();
    const double max_level_scale = std::pow(options.level_scale, options.max_level);
    constexpr double kInfinity = std::numeric_limits<double>::infinity();

    // Best claim per feature: the candidate index and its descriptor distance.
    std::vector<double> claim_distance(static_cast<size_t>(num_features), kInfinity);
    std::vector<size_t> claimed_by(static_cast<size_t>(num_features), points.size());
    std::vector<Eigen::Index> chosen(points.size(), -1);

    for (size_t k = 0; k < points.size(); ++k) {
        const FuseCandidatePoint<FE>& point = points[k];
        if (std::binary_search(observed.begin(), observed.end(), point.id)) {
            continue;
        }
        const geometry::TPoint3DF Xc = T_cw.Transform(point.position);
        const std::optional<geometry::TPoint2DF> uv = gate.Project(Xc);
        if (!uv) {
            continue;
        }
        const geometry::TPoint3DF view = point.position - centre;
        const double distance = view.norm();
        const double view_cos = view.dot(point.normal) / (distance * point.normal.norm());
        if (!(view_cos >= options.min_view_cos)) {
            continue;
        }
        const double max_distance =
            point.reference.distance * std::pow(options.level_scale, point.reference.level);
        const double min_distance = max_distance / max_level_scale;
        if (distance < (1.0 - options.scale_slack) * min_distance ||
            distance > (1.0 + options.scale_slack) * max_distance) {
            continue;
        }
        const int level = detail::PredictLevel(point.reference, distance, options.level_scale, options.max_level);
        const double radius = options.radius_factor * std::pow(options.level_scale, level);

        // Right-image u, if any candidate feature has a right match.
        std::optional<double> right_u;
        if (stereo) {
            const geometry::TPoint3DF Xr = stereo->cameras.T_rl.Transform(Xc);
            if (Xr.z() > 0.0) {
                right_u = stereo->cameras.right.Project(Xr).x();
            }
        }

        double best = kInfinity;
        Eigen::Index best_index = -1;
        for (const Eigen::Index j : grid.FeaturesInArea(*uv, radius, level - 1, level)) {
            const double sigma = options.sigma_px * std::pow(options.level_scale, frame_left.levels()(j));
            double squared_error = (*uv - left_points.col(j)).squaredNorm();
            double chi2_limit = options.chi2_mono;
            const int right = stereo ? (*stereo->right_indices)(j) : -1;
            if (right >= 0) {
                if (!right_u) {
                    continue;  // behind the right camera
                }
                const double e_right = *right_u - (*stereo->right_points)(0, right);
                squared_error += e_right * e_right;
                chi2_limit = options.chi2_stereo;
            }
            if (!(squared_error / (sigma * sigma) < chi2_limit)) {
                continue;
            }
            const typename FE::TDescriptor candidate = frame_left.descriptors().col(j);
            const double d = static_cast<double>(FE::Distance(point.descriptor, candidate));
            if (d < best) {
                best = d;
                best_index = j;
            }
        }
        if (best_index < 0 || best > options.th_low) {
            continue;
        }
        chosen[k] = best_index;
        const auto j = static_cast<size_t>(best_index);
        const bool wins = best < claim_distance[j] ||
                          (best == claim_distance[j] && point.id < points[claimed_by[j]].id);
        if (wins) {
            claim_distance[j] = best;
            claimed_by[j] = k;
        }
    }

    std::vector<FuseDecision> decisions;
    for (size_t k = 0; k < points.size(); ++k) {
        const Eigen::Index j = chosen[k];
        if (j < 0 || claimed_by[static_cast<size_t>(j)] != k) {
            continue;
        }
        decisions.push_back({points[k].id, static_cast<uint32_t>(j), feature_points[static_cast<size_t>(j)]});
    }
    return decisions;
}

}  // namespace voyahslam::features

#endif  // VOYAHSLAM_FEATURES_FUSE_MATCHER_H
