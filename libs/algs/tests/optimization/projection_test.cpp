#include <gtest/gtest.h>

#include <Eigen/Dense>
#include <cmath>
#include <memory>
#include <random>
#include <utility>

#include "optimization/projection.h"
#include "voyahslam/geometry/camera.h"
#include "voyahslam/geometry/distortion_models/kannala_brandt_distortion_model.h"
#include "voyahslam/geometry/rigid_transformation.h"

namespace voyahslam::optimization {
namespace {

using geometry::RigidTransformation;
using geometry::TMatrix33F;
using geometry::TPoint3DF;
using geometry::TVector3DF;

// TUM-VI cam0 (equidistant fisheye).
geometry::distortion_models::TDistortionModelPtr TumViModel() {
    return std::make_shared<const geometry::distortion_models::KannalaBrandtDistortionModel>(
        0.010171079892421483, -0.010816440029919381, 0.005942781769412756, -0.001662284667857643);
}

geometry::Camera TumViCamera(geometry::distortion_models::TDistortionModelPtr model) {
    geometry::Camera camera;
    camera.K << 380.81042871360756, 0.0, 510.29465304840727,
                0.0, 380.81194179427075, 514.3304630538506,
                0.0, 0.0, 1.0;
    camera.distortion = std::move(model);
    return camera;
}

/// A random pose and a world point whose camera-frame ray is up to 80 degrees off the optical
/// axis, 2-10 m away.
std::pair<RigidTransformation, TPoint3DF> RandomPoseAndWidePoint(std::mt19937& rng) {
    std::uniform_real_distribution<double> unit(-1.0, 1.0);
    const Eigen::Vector3d axis = Eigen::Vector3d(unit(rng), unit(rng), unit(rng)).normalized();
    const TMatrix33F rotation = Eigen::AngleAxisd(0.5 * unit(rng), axis).toRotationMatrix();
    const RigidTransformation pose(rotation, TVector3DF(unit(rng), unit(rng), unit(rng)));
    const double theta = 80.0 * M_PI / 180.0 * 0.5 * (unit(rng) + 1.0);
    const double phi = M_PI * unit(rng);
    const double range = 2.0 + 8.0 * (0.5 * (unit(rng) + 1.0));
    const TPoint3DF xc = range * TPoint3DF(std::sin(theta) * std::cos(phi),
                                           std::sin(theta) * std::sin(phi), std::cos(theta));
    return {pose, pose.Inverse().Transform(xc)};
}

/// exp(delta) * T with delta = (omega, upsilon), the g2o SE3Expmap left update.
RigidTransformation LeftPerturbed(const RigidTransformation& pose, int k, double step) {
    Eigen::Matrix<double, 6, 1> delta = Eigen::Matrix<double, 6, 1>::Zero();
    delta[k] = step;
    const TMatrix33F dr = delta.head<3>().norm() == 0.0
                              ? TMatrix33F(TMatrix33F::Identity())
                              : TMatrix33F(Eigen::AngleAxisd(delta.head<3>().norm(),
                                                             delta.head<3>().normalized()));
    return RigidTransformation(dr * pose.rotation(), dr * pose.translation() + delta.tail<3>());
}

template <typename TMatrix>
double RelativeError(const TMatrix& analytic, const TMatrix& numeric) {
    return (analytic - numeric).cwiseAbs().maxCoeff() / numeric.cwiseAbs().maxCoeff();
}

TEST(ProjectionTest, ProjectPixelGoesThroughTheCamera) {
    const auto model = TumViModel();
    const geometry::Camera camera = TumViCamera(model);
    TMatrix33F rotation;
    rotation << 0, -1, 0, 1, 0, 0, 0, 0, 1;
    const RigidTransformation pose(rotation, TVector3DF(1.0, 1.0, 1.0));
    const TPoint3DF x(1.0, 2.0, 3.0);
    const TPoint3DF xc = pose.Transform(x);
    EXPECT_TRUE(ProjectPixel(camera, pose, x).isApprox(camera.Project(xc), 1e-12));

    const RigidTransformation T_rl(
        TMatrix33F(Eigen::AngleAxisd(5.0 * M_PI / 180.0, Eigen::Vector3d::UnitY())),
        TVector3DF(-0.11, 0.0, 0.0));
    EXPECT_TRUE(ProjectPixelRight(camera, pose, T_rl, x)
                    .isApprox(camera.Project(T_rl.Transform(xc)), 1e-12));
}

TEST(ProjectionTest, PixelJacobiansMatchFiniteDifferences) {
    const auto model = TumViModel();
    const geometry::Camera camera = TumViCamera(model);
    std::mt19937 rng(42);
    const double h = 1e-6;

    for (int trial = 0; trial < 10; ++trial) {
        const auto [pose, x] = RandomPoseAndWidePoint(rng);

        Eigen::Matrix<precision_t, 2, 6> pose_jacobian;
        Eigen::Matrix<precision_t, 2, 3> point_jacobian;
        PixelJacobians(camera, pose, x, pose_jacobian, point_jacobian);

        Eigen::Matrix<precision_t, 2, 6> pose_numeric;
        for (int k = 0; k < 6; ++k) {
            pose_numeric.col(k) = (ProjectPixel(camera, LeftPerturbed(pose, k, h), x) -
                                   ProjectPixel(camera, LeftPerturbed(pose, k, -h), x)) /
                                  (2.0 * h);
        }

        Eigen::Matrix<precision_t, 2, 3> point_numeric;
        for (int k = 0; k < 3; ++k) {
            TPoint3DF xp = x;
            TPoint3DF xm = x;
            xp[k] += h;
            xm[k] -= h;
            point_numeric.col(k) =
                (ProjectPixel(camera, pose, xp) - ProjectPixel(camera, pose, xm)) / (2.0 * h);
        }

        EXPECT_LT(RelativeError(pose_jacobian, pose_numeric), 1e-6) << "trial " << trial;
        EXPECT_LT(RelativeError(point_jacobian, point_numeric), 1e-6) << "trial " << trial;
    }
}

TEST(ProjectionTest, PixelJacobianRightMatchesFiniteDifferences) {
    const auto model = TumViModel();
    const geometry::Camera camera = TumViCamera(model);
    std::mt19937 rng(7);
    const double h = 1e-6;
    // A rig with a 5 degree yaw between the cameras and a 0.11 m baseline.
    const RigidTransformation T_rl(
        TMatrix33F(Eigen::AngleAxisd(5.0 * M_PI / 180.0, Eigen::Vector3d::UnitY())),
        TVector3DF(-0.11, 0.0, 0.0));

    for (int trial = 0; trial < 10; ++trial) {
        const auto [pose, x] = RandomPoseAndWidePoint(rng);

        Eigen::Matrix<precision_t, 2, 6> pose_jacobian;
        PixelJacobianRightPose(camera, pose, T_rl, x, pose_jacobian);

        Eigen::Matrix<precision_t, 2, 6> pose_numeric;
        for (int k = 0; k < 6; ++k) {
            pose_numeric.col(k) =
                (ProjectPixelRight(camera, LeftPerturbed(pose, k, h), T_rl, x) -
                 ProjectPixelRight(camera, LeftPerturbed(pose, k, -h), T_rl, x)) /
                (2.0 * h);
        }

        EXPECT_LT(RelativeError(pose_jacobian, pose_numeric), 1e-6) << "trial " << trial;
    }
}

}  // namespace
}  // namespace voyahslam::optimization
