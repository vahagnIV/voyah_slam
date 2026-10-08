#include "voyahslam/geometry/sim3_solver.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>
#include <random>
#include <utility>

#include <Eigen/Dense>

#include "geometry/ransac_support.h"

namespace voyahslam::geometry {

namespace {

constexpr int kSampleSize = 3;
// A point set whose centred scatter has its second eigenvalue below kMinCollinearSine² times
// the largest is (nearly) collinear: the rotation about its line is then undetermined. Same
// tolerance as SolveP3P's collinearity test.
constexpr double kMinCollinearSine = 1e-6;
// The rotation is unique only when N's largest eigenvalue is separated from the next one.
constexpr double kMinRelativeEigenGap = 1e-9;

TVector3DF Centroid(const std::vector<TPoint3DF>& points) {
    TVector3DF sum = TVector3DF::Zero();
    for (const TPoint3DF& p : points) sum += p;
    return sum / static_cast<double>(points.size());
}

// Whether the scatter Σ (p - c)(p - c)ᵀ spans at least a plane.
bool IsSpread(const std::vector<TPoint3DF>& points, const TVector3DF& centroid) {
    TMatrix33F scatter = TMatrix33F::Zero();
    for (const TPoint3DF& p : points) scatter += (p - centroid) * (p - centroid).transpose();
    if (!scatter.allFinite()) return false;
    const Eigen::SelfAdjointEigenSolver<TMatrix33F> eigen(scatter, Eigen::EigenvaluesOnly);
    if (eigen.info() != Eigen::Success) return false;
    // Ascending: (0) smallest, (1) second, (2) largest.
    return eigen.eigenvalues()(1) > kMinCollinearSine * kMinCollinearSine * eigen.eigenvalues()(2);
}

}  // namespace

std::optional<Sim3> AlignHorn(const std::vector<TPoint3DF>& from, const std::vector<TPoint3DF>& to, bool fix_scale) {
    const size_t n = from.size();
    if (n < 3 || to.size() != n) return std::nullopt;

    // Step 1: centroids; both sets must span a plane.
    const TVector3DF c_from = Centroid(from);
    const TVector3DF c_to = Centroid(to);
    if (!IsSpread(from, c_from) || !IsSpread(to, c_to)) return std::nullopt;

    // Step 2: M = Σ (from - c_from)(to - c_to)ᵀ (ORB-SLAM3: M = Pr2 Pr1ᵀ, set 2 → set 1).
    TMatrix33F M = TMatrix33F::Zero();
    for (size_t i = 0; i < n; ++i) M += (from[i] - c_from) * (to[i] - c_to).transpose();

    // Step 3: Horn's symmetric 4x4 N.
    Eigen::Matrix<precision_t, 4, 4> N;
    N(0, 0) = M(0, 0) + M(1, 1) + M(2, 2);
    N(0, 1) = M(1, 2) - M(2, 1);
    N(0, 2) = M(2, 0) - M(0, 2);
    N(0, 3) = M(0, 1) - M(1, 0);
    N(1, 1) = M(0, 0) - M(1, 1) - M(2, 2);
    N(1, 2) = M(0, 1) + M(1, 0);
    N(1, 3) = M(2, 0) + M(0, 2);
    N(2, 2) = -M(0, 0) + M(1, 1) - M(2, 2);
    N(2, 3) = M(1, 2) + M(2, 1);
    N(3, 3) = -M(0, 0) - M(1, 1) + M(2, 2);
    N(1, 0) = N(0, 1);
    N(2, 0) = N(0, 2);
    N(3, 0) = N(0, 3);
    N(2, 1) = N(1, 2);
    N(3, 1) = N(1, 3);
    N(3, 2) = N(2, 3);

    // Step 4: the rotation is the unit quaternion (w, x, y, z) of the largest eigenvalue.
    const Eigen::SelfAdjointEigenSolver<Eigen::Matrix<precision_t, 4, 4>> eigen(N);
    if (eigen.info() != Eigen::Success) return std::nullopt;
    const auto& values = eigen.eigenvalues();  // ascending
    const double magnitude = std::max(std::abs(values(0)), std::abs(values(3)));
    if (!(values(3) - values(2) > kMinRelativeEigenGap * magnitude)) return std::nullopt;
    const Eigen::Matrix<precision_t, 4, 1> q = eigen.eigenvectors().col(3);
    Sim3 S;
    S.R = Eigen::Quaternion<precision_t>(q(0), q(1), q(2), q(3)).normalized().toRotationMatrix();

    // Steps 5-6: the scale Σ Pr_to · (R Pr_from) / Σ ‖Pr_from‖² (rotation preserves norms).
    if (!fix_scale) {
        double numerator = 0.0;
        double denominator = 0.0;
        for (size_t i = 0; i < n; ++i) {
            const TVector3DF rotated = S.R * (from[i] - c_from);
            numerator += (to[i] - c_to).dot(rotated);
            denominator += rotated.squaredNorm();
        }
        S.scale = numerator / denominator;
        if (!(S.scale > 0.0 && std::isfinite(S.scale))) return std::nullopt;
    }

    // Step 7: the translation maps the centroids.
    S.t = c_to - S.scale * (S.R * c_from);
    if (!S.R.allFinite() || !S.t.allFinite()) return std::nullopt;
    return S;
}

Sim3RansacResult SolveSim3Ransac(const Camera& camera1, const Camera& camera2, const std::vector<TPoint3DF>& X1,
                                 const std::vector<TPoint2DF>& pixels1, const std::vector<double>& sigmas1,
                                 const std::vector<TPoint3DF>& X2, const std::vector<TPoint2DF>& pixels2,
                                 const std::vector<double>& sigmas2, const Sim3RansacOptions& options) {
    Sim3RansacResult result;
    const size_t n = X1.size();
    if (pixels1.size() != n || sigmas1.size() != n || X2.size() != n || pixels2.size() != n ||
        sigmas2.size() != n || n < static_cast<size_t>(kSampleSize)) {
        return result;
    }
    result.inlier.assign(n, false);

    std::vector<double> inv_sigma2_1(n);
    std::vector<double> inv_sigma2_2(n);
    for (size_t i = 0; i < n; ++i) {
        inv_sigma2_1[i] = 1.0 / (sigmas1[i] * sigmas1[i]);
        inv_sigma2_2[i] = 1.0 / (sigmas2[i] * sigmas2[i]);
    }

    // The χ² sum of pair i under S_12 (with S_21 its inverse) when it is an inlier in both images.
    const auto pair_chi2 = [&](const Sim3& S_12, const Sim3& S_21, size_t i) -> std::optional<double> {
        const double chi2_1 = detail::ReprojectionChi2(camera1, Apply(S_12, X2[i]), pixels1[i], inv_sigma2_1[i]);
        if (!(chi2_1 < options.chi2)) return std::nullopt;
        const double chi2_2 = detail::ReprojectionChi2(camera2, Apply(S_21, X1[i]), pixels2[i], inv_sigma2_2[i]);
        if (!(chi2_2 < options.chi2)) return std::nullopt;
        return chi2_1 + chi2_2;
    };

    std::mt19937_64 rng(options.seed);
    std::vector<size_t> indices(n);
    std::iota(indices.begin(), indices.end(), size_t{0});
    std::vector<TPoint3DF> sample1(kSampleSize);
    std::vector<TPoint3DF> sample2(kSampleSize);

    Sim3 best_S_12;
    size_t best_inliers = 0;
    double best_chi2 = std::numeric_limits<double>::infinity();
    int required = options.max_iterations;
    while (result.iterations < required) {
        ++result.iterations;

        // Partial Fisher-Yates: the first 3 entries are a uniform distinct sample.
        for (size_t i = 0; i < static_cast<size_t>(kSampleSize); ++i) {
            std::uniform_int_distribution<size_t> pick(i, n - 1);
            std::swap(indices[i], indices[pick(rng)]);
            sample1[i] = X1[indices[i]];
            sample2[i] = X2[indices[i]];
        }
        const std::optional<Sim3> S_12 = AlignHorn(sample2, sample1, options.fix_scale);
        if (!S_12) continue;
        const Sim3 S_21 = Inverse(*S_12);

        size_t num_inliers = 0;
        double total_chi2 = 0.0;
        for (size_t i = 0; i < n; ++i) {
            if (const std::optional<double> chi2 = pair_chi2(*S_12, S_21, i)) {
                ++num_inliers;
                total_chi2 += *chi2;
            }
        }
        if (num_inliers > best_inliers || (num_inliers == best_inliers && num_inliers > 0 && total_chi2 < best_chi2)) {
            best_S_12 = *S_12;
            best_inliers = num_inliers;
            best_chi2 = total_chi2;
            required = detail::AdaptiveIterations(best_inliers, n, kSampleSize, options.confidence,
                                                  options.max_iterations);
        }
    }

    if (best_inliers == 0 || best_inliers < options.min_inliers) return result;
    result.success = true;
    result.S_12 = best_S_12;
    result.num_inliers = best_inliers;
    const Sim3 best_S_21 = Inverse(best_S_12);
    for (size_t i = 0; i < n; ++i) result.inlier[i] = pair_chi2(best_S_12, best_S_21, i).has_value();
    return result;
}

}  // namespace voyahslam::geometry
