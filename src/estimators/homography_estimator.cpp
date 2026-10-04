#include "voyahslam/estimators/homography_estimator.h"

#include <cmath>
#include <vector>

#include "estimators/levenberg_marquardt.h"
#include "estimators/two_view_normalization.h"
#include "voyahslam/algebra/algebra.h"

namespace voyahslam::estimators {

namespace {

using internal::Normalize;

constexpr double kMinW = 1e-12;
constexpr double kSingularRatio = 1e-10;
constexpr double kMinConditionRatio = 1e-8;

// Point a_from mapped by M, dehomogenized; false if mapped to infinity.
bool Transfer(const geometry::TMatrix33F& a_M, const geometry::TPoint2DF& a_from, geometry::TPoint2DF& out_to) {
    const geometry::TVector3DF p = a_M * geometry::TVector3DF(a_from.x(), a_from.y(), 1.0);
    if (!(std::abs(p.z()) >= kMinW) || !p.allFinite()) return false;
    out_to = p.head<2>() / p.z();
    return true;
}

// Squared transfer error of x_from mapped by M, compared to x_to; false if mapped to infinity.
bool TransferChi2(const geometry::TMatrix33F& a_M, const geometry::TPoint2DF& a_from,
                  const geometry::TPoint2DF& a_to, double a_sigma2, double& out_chi2) {
    geometry::TPoint2DF mapped;
    if (!Transfer(a_M, a_from, mapped)) return false;
    out_chi2 = (a_to - mapped).squaredNorm() / a_sigma2;
    return std::isfinite(out_chi2);
}

// Scale-invariant conditioning: smallest/largest singular value of a 3x3 matrix.
bool IsWellConditioned(const geometry::TMatrix33F& a_M) {
    if (!a_M.allFinite()) return false;
    const Eigen::JacobiSVD<geometry::TMatrix33F> svd(a_M);
    const auto& s = svd.singularValues();
    return s(0) > 0.0 && s(2) >= kMinConditionRatio * s(0);
}

// Symmetric transfer residuals (4 per correspondence) of H; false if H is unusable.
bool TransferResiduals(const geometry::TMatrix33F& a_H, const geometry::TwoViewCorrespondences& a_correspondences,
                       const std::vector<Eigen::Index>& a_indices, Eigen::VectorXd& out_r) {
    if (!IsWellConditioned(a_H)) return false;
    const geometry::TMatrix33F H_inv = a_H.inverse();
    out_r.resize(4 * static_cast<Eigen::Index>(a_indices.size()));
    geometry::TPoint2DF mapped;
    for (size_t k = 0; k < a_indices.size(); ++k) {
        const Eigen::Index i = a_indices[k];
        const auto row = 4 * static_cast<Eigen::Index>(k);
        if (!Transfer(a_H, a_correspondences.x1.col(i), mapped)) return false;
        out_r.segment<2>(row) = a_correspondences.x2.col(i) - mapped;
        if (!Transfer(H_inv, a_correspondences.x2.col(i), mapped)) return false;
        out_r.segment<2>(row + 2) = a_correspondences.x1.col(i) - mapped;
    }
    return out_r.allFinite();
}

// Orthonormal basis (9x8) of the matrices orthogonal to a_H (as 9-vectors): a fixed-scale update.
Eigen::Matrix<double, 9, 8> TangentBasis(const geometry::TMatrix33F& a_H) {
    const Eigen::Map<const Eigen::Matrix<double, 9, 1>> h(a_H.data());
    const Eigen::HouseholderQR<Eigen::Matrix<double, 9, 1>> qr(h);
    const Eigen::Matrix<double, 9, 9> Q = qr.householderQ();
    return Q.rightCols<8>();
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

    // Three collinear points (plus one off the line) give a rank-1 solution with a rank-8 system.
    if (!IsWellConditioned(H_hat)) return {};

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

    if (!IsWellConditioned(a_model)) return result;
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

HomographyEstimator::TModel HomographyEstimator::Refine(const TModel& a_model,
                                                        const geometry::TwoViewCorrespondences& a_correspondences,
                                                        const Eigen::Array<bool, Eigen::Dynamic, 1>& a_inliers) const {
    if (a_inliers.size() != a_correspondences.size() || !IsWellConditioned(a_model)) return a_model;
    std::vector<Eigen::Index> indices;
    for (Eigen::Index i = 0; i < a_inliers.size(); ++i) {
        if (a_inliers(i)) indices.push_back(i);
    }
    if (indices.size() < kSampleSize) return a_model;

    using TParams = Eigen::Matrix<double, 8, 1>;
    geometry::TMatrix33F H = a_model / a_model.norm();
    Eigen::Matrix<double, 9, 8> basis = TangentBasis(H);
    const auto moved = [&](const TParams& a_delta) -> geometry::TMatrix33F {
        const Eigen::Matrix<double, 9, 1> step = basis * a_delta;
        return H + Eigen::Map<const geometry::TMatrix33F>(step.data());
    };
    const bool started = internal::MinimizeLevenbergMarquardt<8>(
        [&](const TParams& a_delta, Eigen::VectorXd& out_r) {
            return TransferResiduals(moved(a_delta), a_correspondences, indices, out_r);
        },
        [&](const TParams& a_delta) {
            H = moved(a_delta);
            H /= H.norm();
            basis = TangentBasis(H);
        });
    if (!started) return a_model;

    // Same scale convention as Estimate.
    if (std::abs(H(2, 2)) > 1e-12) H /= H(2, 2);
    if (!H.allFinite() || !IsWellConditioned(H)) return a_model;
    return H;
}

} // namespace voyahslam::estimators
