#include "voyahslam/estimators/essential_estimator.h"

#include <cmath>

#include "estimators/two_view_normalization.h"
#include "voyahslam/algebra/algebra.h"

namespace voyahslam::estimators {

namespace {

constexpr double kSingularRatio = 1e-10;
constexpr double kMinLineNorm2 = 1e-24;

// Squared point-to-line distance chi2 of x against homogeneous line l; false if the line is degenerate.
bool LineChi2(const geometry::TVector3DF& a_line, const geometry::TVector3DF& a_x, double a_sigma2,
              double& out_chi2) {
    const double norm2 = a_line.x() * a_line.x() + a_line.y() * a_line.y();
    if (!(norm2 >= kMinLineNorm2) || !std::isfinite(norm2)) return false;
    const double d = a_x.dot(a_line);
    out_chi2 = d * d / norm2 / a_sigma2;
    return std::isfinite(out_chi2);
}

} // namespace

std::vector<EssentialEstimator::TModel> EssentialEstimator::Estimate(
    const geometry::TwoViewCorrespondences& a_correspondences,
    std::span<const Eigen::Index> a_sample) const {
    if (a_sample.size() < kSampleSize) return {};

    const Eigen::Index n = static_cast<Eigen::Index>(kSampleSize);
    geometry::TPoints2DF p1(2, n), p2(2, n);
    for (Eigen::Index i = 0; i < n; ++i) {
        p1.col(i) = a_correspondences.x1.col(a_sample[i]);
        p2.col(i) = a_correspondences.x2.col(a_sample[i]);
    }

    geometry::TMatrix33F T1, T2;
    if (!internal::Normalize(p1, T1) || !internal::Normalize(p2, T2)) return {};

    // Row i: kron(b, a)^T so that A * vec_rowmajor(E) = b^T E a.
    Eigen::Matrix<double, 8, 9> A;
    for (Eigen::Index i = 0; i < n; ++i) {
        const geometry::TVector3DF a = T1 * geometry::TVector3DF(p1(0, i), p1(1, i), 1.0);
        const geometry::TVector3DF b = T2 * geometry::TVector3DF(p2(0, i), p2(1, i), 1.0);
        A.row(i) << b.x() * a.transpose(), b.y() * a.transpose(), b.z() * a.transpose();
    }
    if (!A.allFinite()) return {};

    // Degeneracy: the 8x9 system must have rank 8.
    const Eigen::JacobiSVD<Eigen::Matrix<double, 8, 9>> rank_svd(A);
    const auto& s = rank_svd.singularValues();
    if (!(s(7) >= kSingularRatio * s(0))) return {};

    const Eigen::Matrix<double, 9, 1> e = algebra::SolveHomogeneousSystem(A);
    geometry::TMatrix33F E_hat;
    E_hat << e(0), e(1), e(2), e(3), e(4), e(5), e(6), e(7), e(8);
    geometry::TMatrix33F E = T2.transpose() * E_hat * T1;
    if (!E.allFinite()) return {};

    // Project onto the essential manifold: singular values ((s1+s2)/2, (s1+s2)/2, 0).
    const Eigen::JacobiSVD<geometry::TMatrix33F> svd(E, Eigen::ComputeFullU | Eigen::ComputeFullV);
    const double sigma = 0.5 * (svd.singularValues()(0) + svd.singularValues()(1));
    if (!(sigma > 0.0)) return {};
    E = svd.matrixU() * geometry::TVector3DF(sigma, sigma, 0.0).asDiagonal() * svd.matrixV().transpose();
    if (!E.allFinite()) return {};
    return {E};
}

ModelScore EssentialEstimator::Score(const TModel& a_model,
                                     const geometry::TwoViewCorrespondences& a_correspondences) const {
    const Eigen::Index n = a_correspondences.size();
    ModelScore result;
    result.inliers = Eigen::Array<bool, Eigen::Dynamic, 1>::Constant(n, false);
    if (!a_model.allFinite()) return result;

    const double sigma2 = a_correspondences.sigma * a_correspondences.sigma;
    for (Eigen::Index i = 0; i < n; ++i) {
        const geometry::TVector3DF x1(a_correspondences.x1(0, i), a_correspondences.x1(1, i), 1.0);
        const geometry::TVector3DF x2(a_correspondences.x2(0, i), a_correspondences.x2(1, i), 1.0);
        bool both = true;
        double chi2 = 0.0;
        if (LineChi2(a_model * x1, x2, sigma2, chi2) && chi2 < kChi2Threshold) {
            result.score += kScoreThreshold - chi2;
        } else {
            both = false;
        }
        if (LineChi2(a_model.transpose() * x2, x1, sigma2, chi2) && chi2 < kChi2Threshold) {
            result.score += kScoreThreshold - chi2;
        } else {
            both = false;
        }
        result.inliers(i) = both;
        if (both) ++result.num_inliers;
    }
    return result;
}

} // namespace voyahslam::estimators
