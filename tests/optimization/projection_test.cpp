#include <gtest/gtest.h>

#include <Eigen/Dense>
#include <random>

#include "optimization/projection.h"
#include "voyahslam/geometry/rigid_transformation.h"

namespace voyahslam::optimization {
namespace {

using geometry::RigidTransformation;
using geometry::TMatrix33F;
using geometry::TPoint3DF;
using geometry::TVector3DF;

TEST(ProjectionTest, ProjectsToNormalizedPlane) {
    const geometry::TPoint2DF p =
        ProjectNormalized(RigidTransformation::Identity(), TPoint3DF(2.0, -1.0, 4.0));
    EXPECT_NEAR(p.x(), 0.5, 1e-12);
    EXPECT_NEAR(p.y(), -0.25, 1e-12);

    // 90 degrees about z: R*(1,2,3) = (-2,1,3); plus t = (1,1,1) -> (-1,2,4).
    TMatrix33F rotation;
    rotation << 0, -1, 0, 1, 0, 0, 0, 0, 1;
    const RigidTransformation pose(rotation, TVector3DF(1.0, 1.0, 1.0));
    const geometry::TPoint2DF q = ProjectNormalized(pose, TPoint3DF(1.0, 2.0, 3.0));
    EXPECT_NEAR(q.x(), -0.25, 1e-12);
    EXPECT_NEAR(q.y(), 0.5, 1e-12);
}

TEST(ProjectionTest, JacobiansMatchFiniteDifferences) {
    std::mt19937 rng(42);
    std::uniform_real_distribution<double> unit(-1.0, 1.0);
    const double h = 1e-6;

    for (int trial = 0; trial < 10; ++trial) {
        const Eigen::Vector3d axis = Eigen::Vector3d(unit(rng), unit(rng), unit(rng)).normalized();
        const TMatrix33F rotation = Eigen::AngleAxisd(0.5 * unit(rng), axis).toRotationMatrix();
        const RigidTransformation pose(rotation, TVector3DF(unit(rng), unit(rng), unit(rng)));
        const double depth = 2.0 + 8.0 * (0.5 * (unit(rng) + 1.0));
        // Choose the world point so that its camera-frame depth is in [2, 10].
        const TPoint3DF xc(unit(rng), unit(rng), depth);
        const TPoint3DF x = pose.Inverse().Transform(xc);

        Eigen::Matrix<precision_t, 2, 6> pose_jacobian;
        Eigen::Matrix<precision_t, 2, 3> point_jacobian;
        ProjectionJacobians(pose, x, pose_jacobian, point_jacobian);

        Eigen::Matrix<precision_t, 2, 6> pose_numeric;
        for (int k = 0; k < 6; ++k) {
            auto perturbed = [&](double step) {
                Eigen::Matrix<double, 6, 1> delta = Eigen::Matrix<double, 6, 1>::Zero();
                delta[k] = step;
                const TMatrix33F dr =
                    delta.head<3>().norm() == 0.0
                        ? TMatrix33F(TMatrix33F::Identity())
                        : TMatrix33F(Eigen::AngleAxisd(delta.head<3>().norm(),
                                                       delta.head<3>().normalized()));
                // exp(delta) * T: rotate everything, then add the translation part.
                const RigidTransformation p(dr * pose.rotation(),
                                            dr * pose.translation() + delta.tail<3>());
                return ProjectNormalized(p, x);
            };
            pose_numeric.col(k) = (perturbed(h) - perturbed(-h)) / (2.0 * h);
        }

        Eigen::Matrix<precision_t, 2, 3> point_numeric;
        for (int k = 0; k < 3; ++k) {
            TPoint3DF xp = x;
            TPoint3DF xm = x;
            xp[k] += h;
            xm[k] -= h;
            point_numeric.col(k) =
                (ProjectNormalized(pose, xp) - ProjectNormalized(pose, xm)) / (2.0 * h);
        }

        EXPECT_LT((pose_jacobian - pose_numeric).cwiseAbs().maxCoeff(), 1e-6);
        EXPECT_LT((point_jacobian - point_numeric).cwiseAbs().maxCoeff(), 1e-6);
    }
}

}  // namespace
}  // namespace voyahslam::optimization
