#ifndef VOYAHSLAM_FEATURES_STEREO_MATCHER_H
#define VOYAHSLAM_FEATURES_STEREO_MATCHER_H

#include <cmath>
#include <cstddef>
#include <limits>
#include <vector>
#include <voyahslam/defs.h>
#include <voyahslam/features/measurement.h>
#include <voyahslam/features/stereo_matches.h>
#include <voyahslam/geometry/geometry.h>

namespace voyahslam::features {

struct StereoMatcherOptions {
    double nn_ratio = 0.7;
    double sigma_px = 1.0;              // level-0 keypoint noise (px)
    double level_scale = 1.2;           // σ_level = sigma_px · level_scale^level (ORB; SIFT: 2^(1/3))
    double max_chi2 = 5.991;            // per-image reprojection test, 2 dof
    double max_parallax_cos = 0.99998;  // rays more parallel than this → rejected
};

namespace detail {

// Squared normalized-plane reprojection error of X (in that camera's frame)
// against the measured homogeneous normalized point, in units of σ².
inline double NormalizedChi2(const geometry::TPoint3DF& X, const geometry::THomogenousPointF& measured,
                             double sigma_normalized) {
    const geometry::TPoint2DF residual = X.hnormalized() - measured.hnormalized();
    return residual.squaredNorm() / (sigma_normalized * sigma_normalized);
}

}  // namespace detail

// ORB-SLAM3 Frame::ComputeStereoFishEyeMatches + KannalaBrandt8::TriangulateMatches:
// brute-force descriptor matching with a ratio test, one-to-one resolution,
// then DLT triangulation gated by positive depth in both cameras, a per-image
// reprojection chi2 test and a minimum parallax. Works for any rig geometry
// (no rectification assumed). Returns, per left feature, the right index and
// the left-camera depth (-1 = none).
template <typename TFeatureExtractor>
[[nodiscard]] StereoMatches MatchStereo(const TBinocularMeasurement<TFeatureExtractor>& measurement,
                                        const StereoMatcherOptions& options) {
    const auto& left = measurement.left();
    const auto& right = measurement.right();
    const StereoRig& rig = measurement.rig();
    const Eigen::Index num_left = left.points().cols();
    const Eigen::Index num_right = right.points().cols();

    StereoMatches result;
    result.right_indices = Eigen::VectorXi::Constant(num_left, -1);
    result.depths = geometry::TRowVectorF::Constant(num_left, -1.0);
    if (num_left == 0 || num_right == 0) {
        return result;
    }

    // 1. Descriptor match with ratio test.
    constexpr double kInfinity = std::numeric_limits<double>::infinity();
    std::vector<Eigen::Index> candidate(num_left, -1);
    std::vector<double> candidate_distance(num_left, kInfinity);
    for (Eigen::Index i = 0; i < num_left; ++i) {
        const typename TMonocularMeasurement<TFeatureExtractor>::TDescriptor d_left = left.descriptors().col(i);
        double best = kInfinity;
        double second_best = kInfinity;
        Eigen::Index best_index = -1;
        for (Eigen::Index j = 0; j < num_right; ++j) {
            const typename TMonocularMeasurement<TFeatureExtractor>::TDescriptor d_right =
                right.descriptors().col(j);
            const double distance = static_cast<double>(TFeatureExtractor::Distance(d_left, d_right));
            if (distance < best) {
                second_best = best;
                best = distance;
                best_index = j;
            } else if (distance < second_best) {
                second_best = distance;
            }
        }
        if (best_index >= 0 && best < options.nn_ratio * second_best) {
            candidate[i] = best_index;
            candidate_distance[i] = best;
        }
    }

    // 2. One-to-one: a right feature claimed twice goes to the smaller distance
    // (the earlier left feature on a tie).
    std::vector<Eigen::Index> claimant(num_right, -1);
    for (Eigen::Index i = 0; i < num_left; ++i) {
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

    // 3–5. Triangulate, gate, record.
    geometry::TMatrix34F P_left = geometry::TMatrix34F::Zero();
    P_left.leftCols<3>().setIdentity();
    const geometry::TMatrix34F P_right = rig.T_rl.ToMatrix34();
    const geometry::TVector3DF centre_right = rig.T_rl.Inverse().translation();
    const double f_left = left.K()(0, 0);
    const double f_right = right.K()(0, 0);
    for (Eigen::Index i = 0; i < num_left; ++i) {
        const Eigen::Index j = candidate[i];
        if (j < 0) {
            continue;
        }
        const geometry::THomogenousPointF x_left = left.normalized_points().col(i);
        const geometry::THomogenousPointF x_right = right.normalized_points().col(j);
        geometry::TPoint3DF X;
        if (!geometry::Triangulate(x_left, x_right, P_left, P_right, X)) {
            continue;
        }

        const geometry::TPoint3DF X_right = rig.T_rl.Transform(X);
        if (!(X.z() > 0.0) || !(X_right.z() > 0.0)) {
            continue;
        }

        const double sigma_left =
            options.sigma_px * std::pow(options.level_scale, static_cast<double>(left.levels()(i))) / f_left;
        const double sigma_right =
            options.sigma_px * std::pow(options.level_scale, static_cast<double>(right.levels()(j))) / f_right;
        if (detail::NormalizedChi2(X, x_left, sigma_left) > options.max_chi2 ||
            detail::NormalizedChi2(X_right, x_right, sigma_right) > options.max_chi2) {
            continue;
        }

        const geometry::TVector3DF ray_left = X;  // left centre is the origin
        const geometry::TVector3DF ray_right = X - centre_right;
        const double parallax_cos = ray_left.dot(ray_right) / (ray_left.norm() * ray_right.norm());
        if (!(parallax_cos < options.max_parallax_cos)) {
            continue;
        }

        result.right_indices[i] = static_cast<int>(j);
        result.depths[i] = X.z();
        ++result.num_matches;
    }
    return result;
}

}  // namespace voyahslam::features

#endif  // VOYAHSLAM_FEATURES_STEREO_MATCHER_H
