#include <voyahslam/geometry/camera.h>

namespace voyahslam::geometry {

TPoint2DF Camera::Project(const TPoint3DF& Xc) const {
    if (distortion == nullptr) {
        return (K * Xc).hnormalized();
    }
    THomogenousPointsF normalized(3, 1);
    normalized.col(0) << Xc.x() / Xc.z(), Xc.y() / Xc.z(), 1.0;
    distortion->Distort(normalized);
    return (K * normalized.col(0)).hnormalized();
}

Eigen::Matrix<precision_t, 2, 3> Camera::ProjectJacobian(const TPoint3DF& Xc) const {
    const precision_t inv_z = 1.0 / Xc.z();
    Eigen::Matrix<precision_t, 2, 3> normalize_jacobian;
    normalize_jacobian << inv_z, 0.0, -Xc.x() * inv_z * inv_z,
                          0.0, inv_z, -Xc.y() * inv_z * inv_z;
    TMatrix22F distort_jacobian = TMatrix22F::Identity();
    if (distortion != nullptr) {
        distort_jacobian = distortion->DistortJacobian(TPoint2DF(Xc.x() * inv_z, Xc.y() * inv_z));
    }
    return K.topLeftCorner<2, 2>() * distort_jacobian * normalize_jacobian;
}

THomogenousPointF Camera::Unproject(const TPoint2DF& pixel) const {
    THomogenousPointsF normalized(3, 1);
    normalized.col(0) = K.inverse() * pixel.homogeneous();
    if (distortion != nullptr) {
        distortion->Undistort(normalized);
    }
    return normalized.col(0);
}

}  // namespace voyahslam::geometry
