#include <gtest/gtest.h>
#include <cmath>
#include <memory>
#include <utility>
#include "voyahslam/defs.h"
#include "voyahslam/geometry/camera.h"
#include "voyahslam/geometry/distortion_models/brown_conrady_distortion_model.h"
#include "voyahslam/geometry/distortion_models/kannala_brandt_distortion_model.h"

using namespace voyahslam::geometry;
using voyahslam::geometry::distortion_models::BrownConradyDistortionModel;
using voyahslam::geometry::distortion_models::KannalaBrandtDistortionModel;
using voyahslam::precision_t;

namespace {

constexpr precision_t kDeg = 3.14159265358979323846 / 180.0;

// TUM-VI cam0 (equidistant fisheye, 1024x1024).
Camera TumViCamera(distortion_models::TDistortionModelPtr model) {
    Camera camera;
    camera.K << 380.81, 0, 510.29, 0, 380.81, 514.33, 0, 0, 1;
    camera.distortion = std::move(model);
    return camera;
}

distortion_models::TDistortionModelPtr TumViModel() {
    return std::make_shared<const KannalaBrandtDistortionModel>(0.010171079892421483, -0.010816440029919381, 0.005942781769412756,
                                        -0.001662284667857643);
}

}  // namespace

TEST(CameraTest, PinholeCameraProjects) {
    Camera camera = TumViCamera(nullptr);
    camera.K(0, 1) = 0.5;  // skew
    const TPoint3DF Xc(0.3, -0.2, 2.0);
    EXPECT_TRUE(camera.Project(Xc).isApprox((camera.K * Xc).hnormalized(), 1e-12));
}

TEST(CameraTest, ProjectUnprojectRoundTripKB) {
    const auto model = TumViModel();
    const Camera camera = TumViCamera(model);
    std::vector<precision_t> thetas;
    for (int d = 0; d <= 80; d += 10) thetas.push_back(d * kDeg);
    thetas.push_back(85 * kDeg);
    for (precision_t theta : thetas) {
        for (int a = 0; a < 8; ++a) {
            const precision_t phi = a * (2 * 3.14159265358979323846 / 8);
            const TPoint3DF Xc(std::sin(theta) * std::cos(phi), std::sin(theta) * std::sin(phi), std::cos(theta));
            const THomogenousPointF ray = camera.Unproject(camera.Project(Xc));
            const precision_t angle = std::atan2(ray.cross(Xc).norm(), ray.dot(Xc));
            EXPECT_LT(angle, 1e-6) << "theta=" << theta / kDeg << " azimuth=" << a;
        }
    }
}

TEST(CameraTest, ProjectJacobianMatchesFiniteDifferences) {
    const auto model = TumViModel();
    for (const distortion_models::TDistortionModelPtr& m : {model, distortion_models::TDistortionModelPtr()}) {
        const Camera camera = TumViCamera(m);
        for (int i = 0; i < 10; ++i) {
            const precision_t theta = (0.5 + 79.5 * (i / 9.0)) * kDeg;
            const precision_t phi = 0.7 * i;
            const precision_t depth = 1.0 + 0.3 * i;
            const TPoint3DF Xc = depth * TPoint3DF(std::tan(theta) * std::cos(phi), std::tan(theta) * std::sin(phi), 1.0);
            const auto analytic = camera.ProjectJacobian(Xc);
            Eigen::Matrix<precision_t, 2, 3> numeric;
            for (int c = 0; c < 3; ++c) {
                const precision_t h = 1e-6 * std::max<precision_t>(1.0, Xc.norm());
                TPoint3DF plus = Xc, minus = Xc;
                plus(c) += h;
                minus(c) -= h;
                numeric.col(c) = (camera.Project(plus) - camera.Project(minus)) / (2 * h);
            }
            EXPECT_LT((analytic - numeric).norm(), 1e-6 * analytic.norm())
                << "i=" << i << (m ? " KB" : " pinhole");
        }
    }
}

// EuRoC MH_01 cam0 (radial-tangential, 752x480): every raw pixel unprojects
// to a ray that projects back onto it, and the projection Jacobian matches
// finite differences.
TEST(CameraTest, ProjectUnprojectRoundTripBrownConrady) {
    Camera camera;
    camera.K << 458.654880721, 0, 367.215803962, 0, 457.296696463, 248.375340610, 0, 0, 1;
    camera.distortion =
        std::make_shared<BrownConradyDistortionModel>(-0.28340811, 0.07395907, 0.00019359, 1.76187114e-05);
    for (int u = 0; u <= 752; u += 47) {
        for (int v = 0; v <= 480; v += 40) {
            const TPoint2DF pixel(u, v);
            const THomogenousPointF ray = camera.Unproject(pixel);
            ASSERT_DOUBLE_EQ(ray.z(), 1.0);
            const TPoint3DF Xc = 2.5 * ray;
            EXPECT_LT((camera.Project(Xc) - pixel).norm(), 1e-8) << "pixel " << u << ", " << v;

            const auto analytic = camera.ProjectJacobian(Xc);
            Eigen::Matrix<precision_t, 2, 3> numeric;
            for (int c = 0; c < 3; ++c) {
                const precision_t h = 1e-6;
                TPoint3DF plus = Xc, minus = Xc;
                plus(c) += h;
                minus(c) -= h;
                numeric.col(c) = (camera.Project(plus) - camera.Project(minus)) / (2 * h);
            }
            EXPECT_LT((analytic - numeric).norm(), 1e-6 * analytic.norm()) << "pixel " << u << ", " << v;
        }
    }
}
