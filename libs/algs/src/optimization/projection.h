#ifndef VOYAHSLAM_OPTIMIZATION_PROJECTION_H
#define VOYAHSLAM_OPTIMIZATION_PROJECTION_H

#include <Eigen/Dense>

#include "voyahslam/defs.h"
#include "voyahslam/geometry/camera.h"
#include "voyahslam/geometry/rigid_transformation.h"
#include "voyahslam/geometry/sim3.h"

namespace voyahslam::optimization {

namespace internal {

/// d(Xc)/d(xi) = [-[Xc]x | I] for the left perturbation T <- exp(xi) * T, xi = (omega, upsilon)
/// (rotation first, then translation; the g2o SE3Expmap update convention).
[[nodiscard]] inline Eigen::Matrix<precision_t, 3, 6> PointPoseJacobian(
    const geometry::TPoint3DF& xc) {
    Eigen::Matrix<precision_t, 3, 6> jacobian;
    jacobian << 0.0, xc.z(), -xc.y(), 1.0, 0.0, 0.0,
                -xc.z(), 0.0, xc.x(), 0.0, 1.0, 0.0,
                xc.y(), -xc.x(), 0.0, 0.0, 0.0, 1.0;
    return jacobian;
}

/// d(y)/d(delta) = [-[y]x | I | y] for the left perturbation S <- exp(delta) * S applied to a
/// point y = S X, delta = (omega, upsilon, sigma) (the g2o VertexSim3Expmap update convention).
[[nodiscard]] inline Eigen::Matrix<precision_t, 3, 7> PointSim3Jacobian(const geometry::TPoint3DF& y) {
    Eigen::Matrix<precision_t, 3, 7> jacobian;
    jacobian.leftCols<6>() = PointPoseJacobian(y);
    jacobian.col(6) = y;
    return jacobian;
}

}  // namespace internal

/// Projects the world point X through T_cw and the camera to a raw (distorted) pixel:
/// camera.Project(T_cw X). The caller must ensure the camera-frame depth is positive.
[[nodiscard]] inline geometry::TPoint2DF ProjectPixel(const geometry::Camera& camera,
                                                      const geometry::RigidTransformation& T_cw,
                                                      const geometry::TPoint3DF& X) {
    return camera.Project(T_cw.Transform(X));
}

/// Jacobians of ProjectPixel. With Xc = T_cw X: out_pose = ProjectJacobian(Xc) * [-[Xc]x | I]
/// w.r.t. the left perturbation T_cw <- exp(xi) * T_cw, xi = (omega, upsilon) (g2o SE3Expmap);
/// out_point = ProjectJacobian(Xc) * R w.r.t. X.
inline void PixelJacobians(const geometry::Camera& camera,
                           const geometry::RigidTransformation& T_cw,
                           const geometry::TPoint3DF& X,
                           Eigen::Matrix<precision_t, 2, 6>& out_pose,
                           Eigen::Matrix<precision_t, 2, 3>& out_point) {
    const geometry::TPoint3DF xc = T_cw.Transform(X);
    const Eigen::Matrix<precision_t, 2, 3> projection_jacobian = camera.ProjectJacobian(xc);
    out_pose = projection_jacobian * internal::PointPoseJacobian(xc);
    out_point = projection_jacobian * T_cw.rotation();
}

/// Projects the world point X into the right camera of a stereo rig as a raw pixel:
/// right_camera.Project(T_rl T_cw X), with T_rl mapping left-camera to right-camera
/// coordinates. The right-camera depth must be positive.
[[nodiscard]] inline geometry::TPoint2DF ProjectPixelRight(
    const geometry::Camera& right_camera, const geometry::RigidTransformation& T_cw,
    const geometry::RigidTransformation& T_rl, const geometry::TPoint3DF& X) {
    return right_camera.Project(T_rl.Transform(T_cw.Transform(X)));
}

/// Jacobian of ProjectPixelRight w.r.t. the left perturbation T_cw <- exp(xi) * T_cw (as in
/// PixelJacobians): with Xc = T_cw X and Xr = T_rl Xc it is
/// right_camera.ProjectJacobian(Xr) * R_rl * [-[Xc]x | I].
inline void PixelJacobianRightPose(const geometry::Camera& right_camera,
                                   const geometry::RigidTransformation& T_cw,
                                   const geometry::RigidTransformation& T_rl,
                                   const geometry::TPoint3DF& X,
                                   Eigen::Matrix<precision_t, 2, 6>& out_pose) {
    const geometry::TPoint3DF xc = T_cw.Transform(X);
    const geometry::TPoint3DF xr = T_rl.Transform(xc);
    out_pose = right_camera.ProjectJacobian(xr) * T_rl.rotation() *
               internal::PointPoseJacobian(xc);
}

/// Stereo pixel (u_l, v_l, u_r) of the world point X: ProjectPixel through the left camera
/// stacked over the x of ProjectPixelRight. Both depths must be positive.
[[nodiscard]] inline Eigen::Matrix<precision_t, 3, 1> StereoPixel(
    const geometry::Camera& left_camera, const geometry::Camera& right_camera,
    const geometry::RigidTransformation& T_cw, const geometry::RigidTransformation& T_rl,
    const geometry::TPoint3DF& X) {
    const geometry::TPoint3DF xc = T_cw.Transform(X);
    Eigen::Matrix<precision_t, 3, 1> pixel;
    pixel.head<2>() = left_camera.Project(xc);
    pixel.z() = right_camera.Project(T_rl.Transform(xc)).x();
    return pixel;
}

/// Jacobians of StereoPixel: the rows of PixelJacobians over the u row of the right camera,
/// right_camera.ProjectJacobian(Xr).row(0) * R_rl * [-[Xc]x | I] (pose, left perturbation as
/// in PixelJacobians) and right_camera.ProjectJacobian(Xr).row(0) * R_rl * R_cw (point).
inline void StereoPixelJacobians(const geometry::Camera& left_camera,
                                 const geometry::Camera& right_camera,
                                 const geometry::RigidTransformation& T_cw,
                                 const geometry::RigidTransformation& T_rl,
                                 const geometry::TPoint3DF& X,
                                 Eigen::Matrix<precision_t, 3, 6>& out_pose,
                                 Eigen::Matrix<precision_t, 3, 3>& out_point) {
    const geometry::TPoint3DF xc = T_cw.Transform(X);
    const geometry::TPoint3DF xr = T_rl.Transform(xc);
    const Eigen::Matrix<precision_t, 2, 3> left_jacobian = left_camera.ProjectJacobian(xc);
    const Eigen::Matrix<precision_t, 1, 3> right_u =
        right_camera.ProjectJacobian(xr).row(0) * T_rl.rotation();
    const Eigen::Matrix<precision_t, 3, 6> point_pose = internal::PointPoseJacobian(xc);
    out_pose.topRows<2>() = left_jacobian * point_pose;
    out_pose.row(2) = right_u * point_pose;
    out_point.topRows<2>() = left_jacobian * T_cw.rotation();
    out_point.row(2) = right_u * T_cw.rotation();
}

/// Raw pixel in camera 1 of the camera-2 point X2: camera1.Project(S_12 X2). The depth of
/// S_12 X2 must be positive.
[[nodiscard]] inline geometry::TPoint2DF Sim3Pixel(const geometry::Camera& camera1, const geometry::Sim3& S_12,
                                                   const geometry::TPoint3DF& X2) {
    return camera1.Project(geometry::Apply(S_12, X2));
}

/// Jacobian of Sim3Pixel w.r.t. the left perturbation S_12 <- exp(delta) * S_12,
/// delta = (omega, upsilon, sigma): ProjectJacobian(y) * [-[y]x | I | y], y = S_12 X2.
[[nodiscard]] inline Eigen::Matrix<precision_t, 2, 7> Sim3PixelJacobian(const geometry::Camera& camera1,
                                                                         const geometry::Sim3& S_12,
                                                                         const geometry::TPoint3DF& X2) {
    const geometry::TPoint3DF y = geometry::Apply(S_12, X2);
    return camera1.ProjectJacobian(y) * internal::PointSim3Jacobian(y);
}

/// Raw pixel in camera 2 of the camera-1 point X1: camera2.Project(S_12⁻¹ X1). The depth of
/// S_12⁻¹ X1 must be positive.
[[nodiscard]] inline geometry::TPoint2DF InverseSim3Pixel(const geometry::Camera& camera2,
                                                          const geometry::Sim3& S_12,
                                                          const geometry::TPoint3DF& X1) {
    return camera2.Project(geometry::Apply(geometry::Inverse(S_12), X1));
}

/// Jacobian of InverseSim3Pixel w.r.t. the same left perturbation of S_12: S_12⁻¹ becomes
/// S_12⁻¹ exp(-delta), so with z = S_12⁻¹ X1 it is
/// ProjectJacobian(z) * (1/s) Rᵀ * [[X1]x | -I | -X1].
[[nodiscard]] inline Eigen::Matrix<precision_t, 2, 7> InverseSim3PixelJacobian(const geometry::Camera& camera2,
                                                                                const geometry::Sim3& S_12,
                                                                                const geometry::TPoint3DF& X1) {
    const geometry::Sim3 S_21 = geometry::Inverse(S_12);
    const geometry::TPoint3DF z = geometry::Apply(S_21, X1);
    return camera2.ProjectJacobian(z) * (S_21.scale * S_21.R) * -internal::PointSim3Jacobian(X1);
}

}  // namespace voyahslam::optimization

#endif  // VOYAHSLAM_OPTIMIZATION_PROJECTION_H
