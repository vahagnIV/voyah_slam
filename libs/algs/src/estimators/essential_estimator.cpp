#include "voyahslam/estimators/essential_estimator.h"

#include <cmath>
#include <vector>

#include "estimators/levenberg_marquardt.h"
#include "estimators/two_view_normalization.h"
#include "voyahslam/algebra/algebra.h"
#include "voyahslam/geometry/two_view_decomposition.h"

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

geometry::TMatrix33F Skew(const geometry::TVector3DF& a_v) {
    geometry::TMatrix33F S;
    S << 0.0, -a_v.z(), a_v.y(), a_v.z(), 0.0, -a_v.x(), -a_v.y(), a_v.x(), 0.0;
    return S;
}

// R <- exp([omega]x) R.
geometry::TMatrix33F RotateBy(const geometry::TVector3DF& a_omega, const geometry::TMatrix33F& a_R) {
    const double angle = a_omega.norm();
    if (angle == 0.0) return a_R;
    return Eigen::AngleAxis<double>(angle, a_omega / angle).toRotationMatrix() * a_R;
}

// t on the unit sphere moved by (a, b) in its tangent plane, renormalized.
geometry::TVector3DF MoveOnSphere(const geometry::TVector3DF& a_t, double a_a, double a_b) {
    const geometry::TVector3DF b1 = a_t.unitOrthogonal();
    const geometry::TVector3DF b2 = a_t.cross(b1);
    return (a_t + a_a * b1 + a_b * b2).normalized();
}

// Signed Sampson residuals of E over the given correspondences; false if any is undefined.
bool SampsonResiduals(const geometry::TMatrix33F& a_E, const geometry::TwoViewCorrespondences& a_correspondences,
                      const std::vector<Eigen::Index>& a_indices, Eigen::VectorXd& out_r) {
    out_r.resize(static_cast<Eigen::Index>(a_indices.size()));
    for (size_t k = 0; k < a_indices.size(); ++k) {
        const Eigen::Index i = a_indices[k];
        const geometry::TVector3DF x1(a_correspondences.x1(0, i), a_correspondences.x1(1, i), 1.0);
        const geometry::TVector3DF x2(a_correspondences.x2(0, i), a_correspondences.x2(1, i), 1.0);
        const geometry::TVector3DF l2 = a_E * x1;
        const geometry::TVector3DF l1 = a_E.transpose() * x2;
        const double denominator = l2.head<2>().squaredNorm() + l1.head<2>().squaredNorm();
        if (!(denominator >= kMinLineNorm2)) return false;
        out_r(static_cast<Eigen::Index>(k)) = x2.dot(l2) / std::sqrt(denominator);
    }
    return out_r.allFinite();
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

EssentialEstimator::TModel EssentialEstimator::Refine(const TModel& a_model,
                                                      const geometry::TwoViewCorrespondences& a_correspondences,
                                                      const Eigen::Array<bool, Eigen::Dynamic, 1>& a_inliers) const {
    if (a_inliers.size() != a_correspondences.size() || !a_model.allFinite()) return a_model;
    std::vector<Eigen::Index> indices;
    for (Eigen::Index i = 0; i < a_inliers.size(); ++i) {
        if (a_inliers(i)) indices.push_back(i);
    }
    if (indices.size() < kSampleSize) return a_model;

    // Any of the 4 decomposition candidates gives E up to sign, which the Sampson error ignores.
    const geometry::RigidTransformation start = geometry::DecomposeEssential(a_model)[0];
    geometry::TMatrix33F R = start.rotation();
    geometry::TVector3DF t = start.translation().normalized();
    if (!R.allFinite() || !t.allFinite()) return a_model;

    using TParams = Eigen::Matrix<double, 5, 1>;
    const auto moved = [&](const TParams& a_delta) -> geometry::TMatrix33F {
        return Skew(MoveOnSphere(t, a_delta(3), a_delta(4))) * RotateBy(a_delta.head<3>(), R);
    };
    const bool started = internal::MinimizeLevenbergMarquardt<5>(
        [&](const TParams& a_delta, Eigen::VectorXd& out_r) {
            return SampsonResiduals(moved(a_delta), a_correspondences, indices, out_r);
        },
        [&](const TParams& a_delta) {
            R = RotateBy(a_delta.head<3>(), R);
            t = MoveOnSphere(t, a_delta(3), a_delta(4));
        });
    if (!started) return a_model;

    const geometry::TMatrix33F E = Skew(t) * R;
    const double norm = E.norm();
    if (!(norm > 0.0) || !E.allFinite()) return a_model;
    return E / norm;
}

} // namespace voyahslam::estimators
