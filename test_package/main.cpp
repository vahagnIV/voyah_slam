// Consumer smoke test: a Brown-Conrady camera (voyah_common), a pose
// optimization (voyah_algs, so g2o must link) and the umbrella's version.
#include <cmath>
#include <cstdio>
#include <vector>

#include <voyahslam/geometry/camera.h>
#include <voyahslam/geometry/distortion_models/brown_conrady_distortion_model.h>
#include <voyahslam/geometry/rigid_transformation.h>
#include <voyahslam/optimization/pose_optimization.h>
#include <voyahslam/voyahslam.h>

int main() {
    using namespace voyahslam;
    const geometry::distortion_models::BrownConradyDistortionModel distortion(
        -0.28, 0.07, 0.0002, 0.00002);
    geometry::Camera camera;
    camera.K << 458.0, 0.0, 367.0, 0.0, 457.0, 248.0, 0.0, 0.0, 1.0;
    camera.distortion = &distortion;

    const geometry::TPoint3DF Xc(0.3, -0.2, 2.0);
    const geometry::TPoint2DF pixel = camera.Project(Xc);
    const geometry::THomogenousPointF ray = camera.Unproject(pixel);
    const double ray_error = (ray.head<2>() - Xc.head<2>() / Xc.z()).norm();

    // Points in front of the identity camera; start the optimizer off by 5 cm.
    std::vector<optimization::PoseObservation> observations;
    for (int i = 0; i < 30; ++i) {
        const geometry::TPoint3DF X(-1.0 + 0.07 * i, 0.5 * std::sin(i), 3.0 + 0.1 * (i % 7));
        observations.push_back({X, camera.Project(X), std::nullopt, 1.0});
    }
    const geometry::RigidTransformation start(geometry::TMatrix33F::Identity(),
                                              geometry::TVector3DF(0.05, 0.0, 0.0));
    const auto result = optimization::OptimizePose(start, camera, std::nullopt, observations,
                                                   optimization::PoseOptimizationOptions{});
    const double pose_error = result.T_cw.translation().norm();

    std::printf("voyahslam %d.%d: unproject error %.2e, pose error %.2e, inliers %zu\n",
                version_major(), version_minor(), ray_error, pose_error, result.num_inliers);
    return (ray_error < 1e-6 && result.success && pose_error < 1e-4) ? 0 : 1;
}
