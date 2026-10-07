#include "voyahslam/geometry/geometry.h"
#include "voyahslam/algebra/algebra.h"

namespace voyahslam::geometry {

TColVectorF SquaredSampsonError(const THomogenousPointsF &pts1,
                                const THomogenousPointsF &pts2,
                                const TMatrix33F &E) {
    const TPoints2DF pts1_2d = pts1.colwise().hnormalized();
    const TPoints2DF pts2_2d = pts2.colwise().hnormalized();
    return SquaredSampsonError(pts1_2d, pts2_2d, E);
}

TColVectorF SquaredSampsonError(const TPoints2DF &pts1,
                                const TPoints2DF &pts2,
                                const TMatrix33F &E) {
    assert(pts1.cols() == pts2.cols());
    THomogenousPointsF pts1_h = pts1.colwise().homogeneous();
    THomogenousPointsF pts2_h = pts2.colwise().homogeneous();
    const THomogenousPointsF E_x1 = E * pts1_h;
    const THomogenousPointsF Et_x2 = E.transpose() * pts2_h;
    TColArrayF num = (pts2_h.transpose() * E_x1).diagonal();
    num = num.square();
    TColArrayF denom = E_x1.row(0).array().square() + E_x1.row(1).array().square()
                       + Et_x2.row(0).array().square() + Et_x2.row(1).array().square();
    return num / denom;
}

precision_t SquaredReprojectionError(const TMatrix33F &a_K, const TMatrix34F &a_pose,
                                    const TPoint3DF &a_3d_point, const TPoint2DF a_image_point) {
    TPoint2DF projected = (a_K * a_pose * a_3d_point.homogeneous()).hnormalized();
    precision_t error = (projected - a_image_point).squaredNorm();
    return error;
}

bool Triangulate(const THomogenousPointF &pt1, const THomogenousPointF &pt2,
                 const TMatrix34F &P1, const TMatrix34F &P2,
                 TPoint3DF &out_triangulated) {
    TMatrix44F A;
    A.row(0) = pt1.x() * P1.row(2) - P1.row(0);
    A.row(1) = pt1.y() * P1.row(2) - P1.row(1);
    A.row(2) = pt2.x() * P2.row(2) - P2.row(0);
    A.row(3) = pt2.y() * P2.row(2) - P2.row(1);

    TVector4DF X = algebra::SolveHomogeneousSystem(A);

    if (!(std::isfinite(X.x()) && std::isfinite(X.y()) && std::isfinite(X.z()) &&
          std::isfinite(X.w())))
        return false;

    if (std::abs(X.w()) < 1e-7) return false;

    out_triangulated = X.hnormalized();
    return true;
}

std::array<TMatrix34F, 4> DecomposeEssentialToPoses(const TMatrix33F &E) {
    Eigen::BDCSVD<TMatrix33F> svd(E, Eigen::ComputeFullU | Eigen::ComputeFullV);
    TMatrix33F U = svd.matrixU();
    TMatrix33F Vt = svd.matrixV().transpose();

    TMatrix33F W;
    W << 0, -1, 0, 1, 0, 0, 0, 0, 1;

    TMatrix33F out_R1 = U * W * Vt;
    TMatrix33F out_R2 = U * W.transpose() * Vt;

    if (out_R1.determinant() < 0) out_R1 = -out_R1;
    if (out_R2.determinant() < 0) out_R2 = -out_R2;

    TVector3DF out_T = U.col(2);

    std::array<TMatrix34F, 4> out_poses;
    out_poses[0] << out_R1, out_T;
    out_poses[1] << out_R2, out_T;
    out_poses[2] << out_R1, -out_T;
    out_poses[3] << out_R2, -out_T;

    return out_poses;
}

precision_t ComputeParallaxCos(const TPoint3DF &a_p3d, const TMatrix33F &a_R, const TColVector3DF &a_T) {
    auto T_c1 = -a_R.transpose() * a_T;
    const TPoint3DF &v1 = a_p3d;
    const TPoint3DF v2 = a_p3d - T_c1;
    precision_t cos_parallax = v1.dot(v2) / (v1.norm() * v2.norm());
    return cos_parallax;
}

precision_t ComputeParallaxCos(const TPoint3DF &a_p3d, const TMatrix34F &a_P2) {
    auto R = a_P2.block<3, 3>(0, 0);
    auto T = a_P2.col(3);
    return ComputeParallaxCos(a_p3d, R, T);
}

} // namespace voyahslam::geometry
