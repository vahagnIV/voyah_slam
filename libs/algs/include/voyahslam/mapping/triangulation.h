#ifndef VOYAHSLAM_MAPPING_TRIANGULATION_H
#define VOYAHSLAM_MAPPING_TRIANGULATION_H

#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <utility>
#include <vector>
#include <voyahslam/defs.h>
#include <voyahslam/features/measurement.h>
#include <voyahslam/features/rotation_histogram.h>
#include <voyahslam/features/stereo_matcher.h>
#include <voyahslam/geometry/camera.h>
#include <voyahslam/geometry/geometry.h>
#include <voyahslam/geometry/rigid_transformation.h>

namespace voyahslam::mapping {

struct TriangulationOptions {
    int th_low = 50;                  // max descriptor distance (ORB-SLAM3 TH_LOW)
    double nn_ratio = 0.6;            // best < nn_ratio * second best
    double min_parallax_cos = 0.9998; // two-view rays more parallel than this → rejected
    double chi2 = 5.991;              // per-image raw-pixel reprojection test, 2 dof
    double sigma_px = 1.0;            // level-0 keypoint noise (raw px)
    double level_scale = 1.2;         // σ_level = sigma_px · level_scale^level
    double scale_consistency = 1.5;   // distance ratio slack, × level_scale
    size_t histogram_bins = 12;       // rotation histogram, 12 bins of 30°
    double epipolar_sigmas = 3.0;     // epipolar band half-width, in σ_level
};

// One Frame's side of a triangulation: its left image, pose and per-feature
// state. Feature indices are columns of `measurement`.
template <typename TFeatureExtractor>
struct TriangulationView {
    const features::TMonocularMeasurement<TFeatureExtractor>* measurement;  // left image
    geometry::RigidTransformation T_cw;
    std::vector<bool> has_point;            // per feature: already observes a map point
    std::vector<precision_t> stereo_depth;  // per feature, < 0 if none; empty for mono
    double baseline = 0.0;                  // rig baseline, 0 for mono
};

struct TriangulatedPoint {
    geometry::TPoint3DF position;  // world
    uint32_t feature_a;
    uint32_t feature_b;
};

namespace detail {

// Below this camera-centre distance two Frames have no baseline at all.
constexpr double kMinTriangulationBaseline = 1e-9;

inline double LevelSigma(const TriangulationOptions& options, int level) {
    return options.sigma_px * std::pow(options.level_scale, static_cast<double>(level));
}

// The ray of a normalized (z = 1) point, unit length.
inline geometry::TVector3DF UnitRay(const geometry::THomogenousPointF& normalized) {
    return normalized.normalized();
}

}  // namespace detail

// ORB-SLAM3 ORBmatcher::SearchForTriangulation on rays: brute force over the
// features of a and b that observe no map point. A candidate b feature must
// lie within epipolar_sigmas · σ_level(b) of a's epipolar plane, measured as
// an angle; among those, the best descriptor distance must be ≤ th_low and
// < nn_ratio · the second best. Then one-to-one (smaller distance wins) and
// rotation-histogram consistency (top 3 bins). Returns (feature_a, feature_b)
// pairs, ascending in feature_a; none when the Frames share a centre.
//
// Epipolar band: the plane through b's centre containing a's centre and a's
// ray has unit normal n = normalize(t_ba × R_ba·ray_a); the test angle is
// asin(|n · ray_b|). For pixel noise e (σ_level per axis), b's unit ray moves
// by J⁺e, J the 2×3 ProjectJacobian at the *unit* ray (J·ray = 0, so J⁺e is
// tangent and its norm is an angle), and the out-of-plane angle n·J⁺e has
// standard deviation σ_level · ‖(J Jᵀ)⁻¹ J n‖. The band is that times
// epipolar_sigmas: an exact 3σ bound in the direction that matters, where
// σ_level / ‖J‖₂ would be the tightest direction's (too strict off-axis on a
// fisheye, whose radial and tangential pixel-per-radian rates differ).
template <typename TFeatureExtractor>
[[nodiscard]] std::vector<std::pair<uint32_t, uint32_t>> SearchForTriangulation(
    const TriangulationView<TFeatureExtractor>& a, const TriangulationView<TFeatureExtractor>& b,
    const TriangulationOptions& options) {
    using TMeasurement = features::TMonocularMeasurement<TFeatureExtractor>;
    using TDescriptor = typename TMeasurement::TDescriptor;
    const TMeasurement& ma = *a.measurement;
    const TMeasurement& mb = *b.measurement;
    const Eigen::Index num_a = ma.points().cols();
    const Eigen::Index num_b = mb.points().cols();
    assert(a.has_point.size() == static_cast<size_t>(num_a) && b.has_point.size() == static_cast<size_t>(num_b));
    std::vector<std::pair<uint32_t, uint32_t>> result;

    const geometry::RigidTransformation T_ba = b.T_cw * a.T_cw.Inverse();
    const geometry::TVector3DF& t_ba = T_ba.translation();
    if (num_a == 0 || num_b == 0 || t_ba.norm() < detail::kMinTriangulationBaseline) {
        return result;
    }

    // Per b feature: unit ray and the 2×3 map n ↦ (J Jᵀ)⁻¹ J n, pre-scaled by
    // the band width, so the band for a normal n is ‖band_b[j] · n‖ radians.
    std::vector<geometry::TVector3DF> ray_b(num_b);
    std::vector<Eigen::Matrix<precision_t, 2, 3>> band_b(num_b);
    for (Eigen::Index j = 0; j < num_b; ++j) {
        if (b.has_point[j]) {
            continue;
        }
        ray_b[j] = detail::UnitRay(mb.normalized_points().col(j));
        const Eigen::Matrix<precision_t, 2, 3> J = mb.camera().ProjectJacobian(ray_b[j]);
        const geometry::TMatrix22F JJt = J * J.transpose();
        band_b[j] = (options.epipolar_sigmas * detail::LevelSigma(options, mb.levels()(j))) * JJt.inverse() * J;
    }

    // 1. Best epipolar-consistent candidate per a feature, with the ratio test.
    constexpr double kInfinity = std::numeric_limits<double>::infinity();
    std::vector<Eigen::Index> candidate(num_a, -1);
    std::vector<double> candidate_distance(num_a, kInfinity);
    for (Eigen::Index i = 0; i < num_a; ++i) {
        if (a.has_point[i]) {
            continue;
        }
        const geometry::TVector3DF normal = t_ba.cross(T_ba.rotation() * detail::UnitRay(ma.normalized_points().col(i)));
        const double normal_norm = normal.norm();
        if (!(normal_norm > 1e-12)) {
            continue;  // a's ray points at b's centre (the epipole): no plane
        }
        const geometry::TVector3DF n = normal / normal_norm;
        const TDescriptor d_a = ma.descriptors().col(i);
        double best = kInfinity;
        double second_best = kInfinity;
        Eigen::Index best_index = -1;
        for (Eigen::Index j = 0; j < num_b; ++j) {
            if (b.has_point[j]) {
                continue;
            }
            const double angle = std::asin(std::min(1.0, std::abs(n.dot(ray_b[j]))));
            if (angle > (band_b[j] * n).norm()) {
                continue;
            }
            const TDescriptor d_b = mb.descriptors().col(j);
            const double distance = static_cast<double>(TFeatureExtractor::Distance(d_a, d_b));
            if (distance < best) {
                second_best = best;
                best = distance;
                best_index = j;
            } else if (distance < second_best) {
                second_best = distance;
            }
        }
        if (best_index >= 0 && best <= static_cast<double>(options.th_low) && best < options.nn_ratio * second_best) {
            candidate[i] = best_index;
            candidate_distance[i] = best;
        }
    }

    // 2. One-to-one: a b feature claimed twice goes to the smaller distance
    // (the earlier a feature on a tie).
    std::vector<Eigen::Index> claimant(num_b, -1);
    for (Eigen::Index i = 0; i < num_a; ++i) {
        const Eigen::Index j = candidate[i];
        if (j < 0) {
            continue;
        }
        const Eigen::Index previous = claimant[j];
        if (previous < 0) {
            claimant[j] = i;
        } else if (candidate_distance[i] < candidate_distance[previous]) {
            candidate[previous] = -1;
            claimant[j] = i;
        } else {
            candidate[i] = -1;
        }
    }

    // 3. Rotation consistency.
    features::RotationHistogram histogram(options.histogram_bins);
    for (Eigen::Index i = 0; i < num_a; ++i) {
        if (candidate[i] >= 0) {
            histogram.Add(static_cast<double>(ma.angles()(i)) - static_cast<double>(mb.angles()(candidate[i])), i);
        }
    }
    for (const Eigen::Index i : histogram.Inconsistent()) {
        candidate[i] = -1;
    }

    for (Eigen::Index i = 0; i < num_a; ++i) {
        if (candidate[i] >= 0) {
            result.emplace_back(static_cast<uint32_t>(i), static_cast<uint32_t>(candidate[i]));
        }
    }
    return result;
}

// ORB-SLAM3 LocalMapping::CreateNewMapPoints acceptance, on rays. Per match:
//   - Placement: DLT of the two rays when their parallax cos is
//     < min_parallax_cos. If a feature has a stereo depth whose parallax
//     (its rig baseline seen from that depth, cos(2·atan2(baseline/2, depth)))
//     is larger than the two-view one, the point is instead that feature's
//     stereo back-projection T_wc · (normalized · depth), the larger stereo
//     parallax winning when both have one.
//   - Positive depth in both views, raw-pixel χ² < chi2 in both with
//     σ = σ_level, and scale consistency: with
//     ratio_level = level_scale^(level_a − level_b), dist_b / dist_a lies
//     within ratio_level / (scale_consistency · level_scale) and
//     ratio_level · scale_consistency · level_scale (a feature found at a
//     coarser level looks larger, so its Frame is the closer one).
// Returns the accepted points, in match order.
template <typename TFeatureExtractor>
[[nodiscard]] std::vector<TriangulatedPoint> TriangulateMatches(
    const TriangulationView<TFeatureExtractor>& a, const TriangulationView<TFeatureExtractor>& b,
    const std::vector<std::pair<uint32_t, uint32_t>>& matches, const TriangulationOptions& options) {
    const auto& ma = *a.measurement;
    const auto& mb = *b.measurement;
    assert(a.stereo_depth.empty() || a.stereo_depth.size() == static_cast<size_t>(ma.points().cols()));
    assert(b.stereo_depth.empty() || b.stereo_depth.size() == static_cast<size_t>(mb.points().cols()));
    std::vector<TriangulatedPoint> result;

    const geometry::RigidTransformation T_wa = a.T_cw.Inverse();
    const geometry::RigidTransformation T_wb = b.T_cw.Inverse();
    const geometry::TMatrix34F P_a = a.T_cw.ToMatrix34();
    const geometry::TMatrix34F P_b = b.T_cw.ToMatrix34();
    const geometry::TVector3DF& centre_a = T_wa.translation();
    const geometry::TVector3DF& centre_b = T_wb.translation();
    const double ratio_factor = options.scale_consistency * options.level_scale;

    // cos of the parallax a rig of `baseline` sees at `depth`; +inf without a
    // depth, so it never wins (a ray cos can round above 1).
    const auto stereo_parallax_cos = [](const TriangulationView<TFeatureExtractor>& view, uint32_t feature) {
        if (view.stereo_depth.empty() || view.stereo_depth[feature] < 0.0) {
            return std::numeric_limits<double>::infinity();
        }
        return std::cos(2.0 * std::atan2(view.baseline / 2.0, static_cast<double>(view.stereo_depth[feature])));
    };

    for (const auto& [ia, ib] : matches) {
        const geometry::THomogenousPointF x_a = ma.normalized_points().col(ia);
        const geometry::THomogenousPointF x_b = mb.normalized_points().col(ib);
        const geometry::TVector3DF ray_a = T_wa.rotation() * detail::UnitRay(x_a);
        const geometry::TVector3DF ray_b = T_wb.rotation() * detail::UnitRay(x_b);
        const double rays_cos = ray_a.dot(ray_b);
        const double stereo_cos_a = stereo_parallax_cos(a, ia);
        const double stereo_cos_b = stereo_parallax_cos(b, ib);

        geometry::TPoint3DF X;
        if (std::min(stereo_cos_a, stereo_cos_b) < rays_cos) {
            X = stereo_cos_a <= stereo_cos_b ? T_wa.Transform(geometry::TPoint3DF(x_a * a.stereo_depth[ia]))
                                             : T_wb.Transform(geometry::TPoint3DF(x_b * b.stereo_depth[ib]));
        } else if (rays_cos > 0.0 && rays_cos < options.min_parallax_cos) {
            if (!geometry::Triangulate(x_a, x_b, P_a, P_b, X)) {
                continue;
            }
        } else {
            continue;
        }

        const geometry::TPoint3DF X_a = a.T_cw.Transform(X);
        const geometry::TPoint3DF X_b = b.T_cw.Transform(X);
        if (!(X_a.z() > 0.0) || !(X_b.z() > 0.0)) {
            continue;
        }

        const int level_a = ma.levels()(ia);
        const int level_b = mb.levels()(ib);
        if (features::detail::PixelChi2(ma.camera(), X_a, ma.distorted_points().col(ia),
                                        detail::LevelSigma(options, level_a)) >= options.chi2 ||
            features::detail::PixelChi2(mb.camera(), X_b, mb.distorted_points().col(ib),
                                        detail::LevelSigma(options, level_b)) >= options.chi2) {
            continue;
        }

        const double dist_a = (X - centre_a).norm();
        const double dist_b = (X - centre_b).norm();
        if (!(dist_a > 0.0) || !(dist_b > 0.0)) {
            continue;
        }
        const double ratio_dist = dist_b / dist_a;
        const double ratio_level = std::pow(options.level_scale, static_cast<double>(level_a - level_b));
        if (ratio_dist * ratio_factor < ratio_level || ratio_dist > ratio_level * ratio_factor) {
            continue;
        }

        result.push_back({X, ia, ib});
    }
    return result;
}

}  // namespace voyahslam::mapping

#endif  // VOYAHSLAM_MAPPING_TRIANGULATION_H
