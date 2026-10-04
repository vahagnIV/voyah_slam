#include "voyahslam/estimators/homography_estimator.h"

#include <cmath>

#include "voyahslam/algebra/algebra.h"

namespace voyahslam::estimators {

namespace {

constexpr double kMinW = 1e-12;
constexpr double kSingularRatio = 1e-10;

// Returns false if the points are (nearly) identical so no normalization exists.
bool Normalize(const geometry::TPoints2DF& a_pts, geometry::TMatrix33F& out_T) {
    const geometry::TPoint2DF centroid = a_pts.rowwise().mean();
    const double spread = (a_pts.colwise() - centroid).colwise().squaredNorm().mean();
    if (!std::isfinite(spread) || spread <= 0.0) return false;
    out_T = algebra::GetHartleyNormalizationMatrix(a_pts);
    return out_T.allFinite();
}

// Squared transfer error of x_from mapped by M, compared to x_to; false if mapped to infinity.
bool TransferChi2(const geometry::TMatrix33F& a_M, const geometry::TPoint2DF& a_from,
                  const geometry::TPoint2DF& a_to, double a_sigma2, double& out_chi2) {
    const geometry::TVector3DF p = a_M * geometry::TVector3DF(a_from.x(), a_from.y(), 1.0);
    if (!(std::abs(p.z()) >= kMinW) || !p.allFinite()) return false;
    out_chi2 = (a_to - p.head<2>() / p.z()).squaredNorm() / a_sigma2;
    return std::isfinite(out_chi2);
}

} // namespace

std::vector<HomographyEstimator::TModel> HomographyEstimator::Estimate(
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
    if (!Normalize(p1, T1) || !Normalize(p2, T2)) return {};

    Eigen::Matrix<double, 8, 9> A;
    for (Eigen::Index i = 0; i < n; ++i) {
        const geometry::TVector3DF a = T1 * geometry::TVector3DF(p1(0, i), p1(1, i), 1.0);
        const geometry::TVector3DF b = T2 * geometry::TVector3DF(p2(0, i), p2(1, i), 1.0);
        A.row(2 * i) << 0, 0, 0, -a.transpose(), b.y() * a.transpose();
        A.row(2 * i + 1) << a.transpose(), 0, 0, 0, -b.x() * a.transpose();
    }
    if (!A.allFinite()) return {};

    // Degeneracy: the 8x9 system must have rank 8 (second-smallest of the 9 singular values > 0).
    Eigen::JacobiSVD<Eigen::Matrix<double, 8, 9>> svd(A);
    const auto& s = svd.singularValues();
    if (!(s(7) >= kSingularRatio * s(0))) return {};

    const Eigen::Matrix<double, 9, 1> h = algebra::SolveHomogeneousSystem(A);
    geometry::TMatrix33F H_hat;
    H_hat << h(0), h(1), h(2), h(3), h(4), h(5), h(6), h(7), h(8);

    geometry::TMatrix33F H = T2.inverse() * H_hat * T1;
    if (std::abs(H(2, 2)) > 1e-12) H /= H(2, 2);
    if (!H.allFinite()) return {};
    return {H};
}

ModelScore HomographyEstimator::Score(const TModel& a_model,
                                      const geometry::TwoViewCorrespondences& a_correspondences) const {
    const Eigen::Index n = a_correspondences.size();
    ModelScore result;
    result.inliers = Eigen::Array<bool, Eigen::Dynamic, 1>::Constant(n, false);

    if (!a_model.allFinite() || std::abs(a_model.determinant()) < 1e-300) return result;
    const geometry::TMatrix33F H_inv = a_model.inverse();
    if (!H_inv.allFinite()) return result;

    const double sigma2 = a_correspondences.sigma * a_correspondences.sigma;
    for (Eigen::Index i = 0; i < n; ++i) {
        bool both = true;
        double chi2 = 0.0;
        if (TransferChi2(a_model, a_correspondences.x1.col(i), a_correspondences.x2.col(i), sigma2, chi2) &&
            chi2 < kChi2Threshold) {
            result.score += kChi2Threshold - chi2;
        } else {
            both = false;
        }
        if (TransferChi2(H_inv, a_correspondences.x2.col(i), a_correspondences.x1.col(i), sigma2, chi2) &&
            chi2 < kChi2Threshold) {
            result.score += kChi2Threshold - chi2;
        } else {
            both = false;
        }
        result.inliers(i) = both;
        if (both) ++result.num_inliers;
    }
    return result;
}

} // namespace voyahslam::estimators
