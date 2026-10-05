#ifndef VOYAHSLAM_OPTIMIZATION_PROJECTION_H
#define VOYAHSLAM_OPTIMIZATION_PROJECTION_H

#include <Eigen/Dense>

#include "voyahslam/defs.h"
#include "voyahslam/geometry/rigid_transformation.h"

namespace voyahslam::optimization {

/// Projects the world point X through T_cw onto the normalized image plane: (x/z, y/z) of
/// Xc = R X + t. The caller must ensure the camera-frame depth is non-zero.
[[nodiscard]] inline geometry::TPoint2DF ProjectNormalized(
    const geometry::RigidTransformation& T_cw, const geometry::TPoint3DF& X) {
    const geometry::TPoint3DF xc = T_cw.Transform(X);
    return geometry::TPoint2DF(xc.x() / xc.z(), xc.y() / xc.z());
}

/// Jacobians of ProjectNormalized. out_pose is taken w.r.t. a left perturbation
/// T <- exp(xi) * T with xi = (omega, upsilon) (rotation first, then translation; the
/// g2o SE3Expmap update convention); out_point w.r.t. X.
inline void ProjectionJacobians(const geometry::RigidTransformation& T_cw,
                                const geometry::TPoint3DF& X,
                                Eigen::Matrix<precision_t, 2, 6>& out_pose,
                                Eigen::Matrix<precision_t, 2, 3>& out_point) {
    const geometry::TPoint3DF xc = T_cw.Transform(X);
    const precision_t inv_z = 1.0 / xc.z();
    const precision_t inv_z2 = inv_z * inv_z;

    Eigen::Matrix<precision_t, 2, 3> projection_jacobian;
    projection_jacobian << inv_z, 0.0, -xc.x() * inv_z2,
                           0.0, inv_z, -xc.y() * inv_z2;

    geometry::TMatrix33F skew;
    skew << 0.0, -xc.z(), xc.y(),
            xc.z(), 0.0, -xc.x(),
            -xc.y(), xc.x(), 0.0;

    out_pose.leftCols<3>() = projection_jacobian * (-skew);
    out_pose.rightCols<3>() = projection_jacobian;
    out_point = projection_jacobian * T_cw.rotation();
}

/// Projects the world point X into the right camera of a stereo rig: pi(T_rl * T_cw * X), with
/// T_rl mapping left-camera to right-camera coordinates. The right-camera depth must be non-zero.
[[nodiscard]] inline geometry::TPoint2DF ProjectNormalizedRight(
    const geometry::RigidTransformation& T_cw, const geometry::RigidTransformation& T_rl,
    const geometry::TPoint3DF& X) {
    const geometry::TPoint3DF xr = T_rl.Transform(T_cw.Transform(X));
    return geometry::TPoint2DF(xr.x() / xr.z(), xr.y() / xr.z());
}

/// Jacobian of ProjectNormalizedRight w.r.t. the left perturbation T_cw <- exp(xi) * T_cw,
/// xi = (omega, upsilon) as in ProjectionJacobians: with Xc = T_cw X and Xr = T_rl Xc it is
/// d pi(Xr) * R_rl * [-[Xc]x | I].
inline void ProjectionJacobianRightPose(const geometry::RigidTransformation& T_cw,
                                        const geometry::RigidTransformation& T_rl,
                                        const geometry::TPoint3DF& X,
                                        Eigen::Matrix<precision_t, 2, 6>& out_pose) {
    const geometry::TPoint3DF xc = T_cw.Transform(X);
    const geometry::TPoint3DF xr = T_rl.Transform(xc);
    const precision_t inv_z = 1.0 / xr.z();
    const precision_t inv_z2 = inv_z * inv_z;

    Eigen::Matrix<precision_t, 2, 3> projection_jacobian;
    projection_jacobian << inv_z, 0.0, -xr.x() * inv_z2,
                           0.0, inv_z, -xr.y() * inv_z2;
    const Eigen::Matrix<precision_t, 2, 3> chained = projection_jacobian * T_rl.rotation();

    geometry::TMatrix33F skew;
    skew << 0.0, -xc.z(), xc.y(),
            xc.z(), 0.0, -xc.x(),
            -xc.y(), xc.x(), 0.0;

    out_pose.leftCols<3>() = chained * (-skew);
    out_pose.rightCols<3>() = chained;
}

}  // namespace voyahslam::optimization

#endif  // VOYAHSLAM_OPTIMIZATION_PROJECTION_H
