#include "voyahslam/geometry/two_view_decomposition.h"

#include <cmath>

#include <Eigen/Dense>

#include "voyahslam/geometry/geometry.h"

namespace voyahslam::geometry {

namespace {

// Singular values closer than this ratio are treated as equal (ORB-SLAM3 ReconstructH).
constexpr double kMinSingularValueRatio = 1.00001;

} // namespace

std::array<RigidTransformation, 4> DecomposeEssential(const TMatrix33F& a_E) {
    const std::array<TMatrix34F, 4> poses = DecomposeEssentialToPoses(a_E);
    std::array<RigidTransformation, 4> candidates;
    for (size_t i = 0; i < poses.size(); ++i) {
        candidates[i] = RigidTransformation(poses[i].leftCols<3>(), poses[i].col(3));
    }
    return candidates;
}

std::vector<HomographyCandidate> DecomposeHomography(const TMatrix33F& a_H) {
    // Faugeras, Lustman. Motion and structure from motion in a piecewise planar environment. 1988.
    const Eigen::JacobiSVD<TMatrix33F> svd(a_H, Eigen::ComputeFullU | Eigen::ComputeFullV);
    const TMatrix33F& U = svd.matrixU();
    const TMatrix33F& V = svd.matrixV();
    const TVector3DF& w = svd.singularValues();
    const double s = U.determinant() * V.determinant();
    const double d1 = w(0);
    const double d2 = w(1);
    const double d3 = w(2);

    // Negated comparisons also reject NaN ratios (e.g. a zero matrix).
    if (!(d1 / d2 >= kMinSingularValueRatio) || !(d2 / d3 >= kMinSingularValueRatio)) return {};

    // n' = [x1 0 x3] with the 4 sign combinations of (e1, e3).
    const double aux1 = std::sqrt((d1 * d1 - d2 * d2) / (d1 * d1 - d3 * d3));
    const double aux3 = std::sqrt((d2 * d2 - d3 * d3) / (d1 * d1 - d3 * d3));
    const std::array<double, 4> x1{aux1, aux1, -aux1, -aux1};
    const std::array<double, 4> x3{aux3, -aux3, aux3, -aux3};

    std::vector<HomographyCandidate> candidates;
    candidates.reserve(8);
    const auto add_candidate = [&](const TMatrix33F& a_Rp, const TVector3DF& a_tp, size_t a_i) {
        const TMatrix33F R = s * U * a_Rp * V.transpose();
        const TVector3DF t = (U * a_tp).normalized();
        TVector3DF n = V * TVector3DF(x1[a_i], 0.0, x3[a_i]);
        if (n.z() < 0.0) n = -n;
        candidates.push_back({RigidTransformation(R, t), n});
    };

    // Case d' = d2.
    const double sin_theta = std::sqrt((d1 * d1 - d2 * d2) * (d2 * d2 - d3 * d3)) / ((d1 + d3) * d2);
    const double cos_theta = (d2 * d2 + d1 * d3) / ((d1 + d3) * d2);
    const std::array<double, 4> stheta{sin_theta, -sin_theta, -sin_theta, sin_theta};
    for (size_t i = 0; i < 4; ++i) {
        TMatrix33F Rp = TMatrix33F::Zero();
        Rp(0, 0) = cos_theta;
        Rp(0, 2) = -stheta[i];
        Rp(1, 1) = 1.0;
        Rp(2, 0) = stheta[i];
        Rp(2, 2) = cos_theta;
        add_candidate(Rp, TVector3DF(x1[i], 0.0, -x3[i]) * (d1 - d3), i);
    }

    // Case d' = -d2.
    const double sin_phi = std::sqrt((d1 * d1 - d2 * d2) * (d2 * d2 - d3 * d3)) / ((d1 - d3) * d2);
    const double cos_phi = (d1 * d3 - d2 * d2) / ((d1 - d3) * d2);
    const std::array<double, 4> sphi{sin_phi, -sin_phi, -sin_phi, sin_phi};
    for (size_t i = 0; i < 4; ++i) {
        TMatrix33F Rp = TMatrix33F::Zero();
        Rp(0, 0) = cos_phi;
        Rp(0, 2) = sphi[i];
        Rp(1, 1) = -1.0;
        Rp(2, 0) = sphi[i];
        Rp(2, 2) = -cos_phi;
        add_candidate(Rp, TVector3DF(x1[i], 0.0, x3[i]) * (d1 + d3), i);
    }
    return candidates;
}

} // namespace voyahslam::geometry
