#include <gtest/gtest.h>

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <random>
#include <vector>

#include <Eigen/Dense>

#include "optimization/projection.h"
#include "voyahslam/geometry/camera.h"
#include "voyahslam/geometry/distortion_models/kannala_brandt_distortion_model.h"
#include "voyahslam/geometry/sim3.h"
#include "voyahslam/optimization/sim3_optimization.h"

namespace voyahslam::optimization {
namespace {

using geometry::Camera;
using geometry::Sim3;
using geometry::TMatrix33F;
using geometry::TPoint2DF;
using geometry::TPoint3DF;
using geometry::TVector3DF;

// TUM-VI cam0 (equidistant fisheye).
geometry::distortion_models::KannalaBrandtDistortionModel TumViModel() {
    return geometry::distortion_models::KannalaBrandtDistortionModel(
        0.010171079892421483, -0.010816440029919381, 0.005942781769412756, -0.001662284667857643);
}

Camera TumViCamera(const geometry::distortion_models::DistortionModel* model) {
    Camera camera;
    camera.K << 380.81042871360756, 0.0, 510.29465304840727,
                0.0, 380.81194179427075, 514.3304630538506,
                0.0, 0.0, 1.0;
    camera.distortion = model;
    return camera;
}

Camera PinholeCamera() {
    Camera camera;
    camera.K << 450.0, 0.0, 320.0, 0.0, 450.0, 240.0, 0.0, 0.0, 1.0;
    return camera;
}

TVector3DF RandomUnit(std::mt19937& rng) {
    std::normal_distribution<double> normal(0.0, 1.0);
    return TVector3DF(normal(rng), normal(rng), normal(rng)).normalized();
}

/// A camera-frame point up to max_angle_deg off the optical axis, 2-10 m away.
TPoint3DF RandomPointInView(std::mt19937& rng, double max_angle_deg) {
    std::uniform_real_distribution<double> unit(0.0, 1.0);
    const double theta = max_angle_deg * M_PI / 180.0 * unit(rng);
    const double phi = 2.0 * M_PI * unit(rng);
    const double range = 2.0 + 8.0 * unit(rng);
    return range * TPoint3DF(std::sin(theta) * std::cos(phi), std::sin(theta) * std::sin(phi), std::cos(theta));
}

/// exp(delta) ∘ S, delta = (omega, upsilon, sigma): rotation exp(omega), translation upsilon,
/// scale exp(sigma). Its derivative at delta = 0 is g2o's VertexSim3Expmap update
/// (Sim3(update) * estimate), whose V(omega, sigma) is the identity at 0.
Sim3 LeftPerturbed(const Sim3& S, int k, double step) {
    Eigen::Matrix<double, 7, 1> delta = Eigen::Matrix<double, 7, 1>::Zero();
    delta[k] = step;
    const double angle = delta.head<3>().norm();
    const TMatrix33F R = angle == 0.0 ? TMatrix33F(TMatrix33F::Identity())
                                      : TMatrix33F(Eigen::AngleAxisd(angle, delta.head<3>() / angle));
    return Compose(Sim3{std::exp(delta[6]), R, delta.segment<3>(3)}, S);
}

Sim3 RandomSim3(std::mt19937& rng, double scale) {
    std::uniform_real_distribution<double> unit(-1.0, 1.0);
    return Sim3{scale, TMatrix33F(Eigen::AngleAxisd(0.3 * unit(rng), RandomUnit(rng))),
                TVector3DF(0.5 * unit(rng), 0.3 * unit(rng), 0.3 * unit(rng))};
}

/// S perturbed by 3 degrees, 0.1 m and (unless fix_scale) a scale factor of 1.1.
Sim3 Perturbed(const Sim3& S, std::mt19937& rng, bool fix_scale) {
    return Sim3{fix_scale ? S.scale : 1.1 * S.scale,
                TMatrix33F(Eigen::AngleAxisd(3.0 * M_PI / 180.0, RandomUnit(rng))) * S.R,
                S.t + 0.1 * RandomUnit(rng)};
}

double RotationErrorDeg(const Sim3& a, const Sim3& b) {
    return Eigen::AngleAxisd(a.R.transpose() * b.R).angle() * 180.0 / M_PI;
}

struct Scene {
    Sim3 S_12;                       ///< Ground truth.
    std::vector<Sim3Match> matches;  ///< Exact pixels (plus noise_px), sigma = 1.2^level.
};

/// num_matches pairs seen by both cameras (camera 1 within 60 degrees, camera 2 within 70):
/// X1 = S_12 X2, each observed at its exact raw pixel plus Gaussian noise of noise_px * sigma.
Scene MakeScene(const Camera& camera1, const Camera& camera2, const Sim3& S_12, size_t num_matches,
                double noise_px, std::mt19937& rng) {
    std::uniform_int_distribution<int> level(0, 3);
    std::normal_distribution<double> noise(0.0, 1.0);
    const Sim3 S_21 = Inverse(S_12);
    Scene scene{S_12, {}};
    while (scene.matches.size() < num_matches) {
        const TPoint3DF X1 = RandomPointInView(rng, 60.0);
        const TPoint3DF X2 = Apply(S_21, X1);
        if (X2.z() < std::cos(70.0 * M_PI / 180.0) * X2.norm()) {
            continue;
        }
        const double sigma1 = std::pow(1.2, level(rng));
        const double sigma2 = std::pow(1.2, level(rng));
        const TPoint2DF pixel1 = camera1.Project(X1) + noise_px * sigma1 * TPoint2DF(noise(rng), noise(rng));
        const TPoint2DF pixel2 = camera2.Project(X2) + noise_px * sigma2 * TPoint2DF(noise(rng), noise(rng));
        scene.matches.push_back({X1, pixel1, sigma1, X2, pixel2, sigma2});
    }
    return scene;
}

template <typename TMatrix>
double RelativeError(const TMatrix& analytic, const TMatrix& numeric) {
    return (analytic - numeric).cwiseAbs().maxCoeff() / numeric.cwiseAbs().maxCoeff();
}

// The edges' residuals and Jacobians are Sim3Pixel / InverseSim3Pixel and their Jacobians
// verbatim; the vertex update is g2o's left exp(delta) ∘ S.
TEST(Sim3OptimizationTest, JacobianMatchesFiniteDifference) {
    const auto model = TumViModel();
    const Camera fisheye = TumViCamera(&model);
    const Camera pinhole = PinholeCamera();
    std::mt19937 rng(11);
    const double h = 1e-6;
    for (int trial = 0; trial < 20; ++trial) {
        const Camera& camera1 = trial % 2 == 0 ? fisheye : pinhole;
        const Camera& camera2 = trial % 2 == 0 ? pinhole : fisheye;
        const Sim3 S_12 = RandomSim3(rng, 0.5 + 0.1 * trial);
        const Scene scene = MakeScene(camera1, camera2, S_12, 1, 0.0, rng);
        const TPoint3DF& X1 = scene.matches[0].X1;
        const TPoint3DF& X2 = scene.matches[0].X2;

        EXPECT_TRUE(Sim3Pixel(camera1, S_12, X2).isApprox(camera1.Project(X1), 1e-9));
        EXPECT_TRUE(InverseSim3Pixel(camera2, S_12, X1).isApprox(camera2.Project(X2), 1e-9));

        const Eigen::Matrix<precision_t, 2, 7> jacobian12 = Sim3PixelJacobian(camera1, S_12, X2);
        const Eigen::Matrix<precision_t, 2, 7> jacobian21 = InverseSim3PixelJacobian(camera2, S_12, X1);
        Eigen::Matrix<precision_t, 2, 7> numeric12;
        Eigen::Matrix<precision_t, 2, 7> numeric21;
        for (int k = 0; k < 7; ++k) {
            const Sim3 plus = LeftPerturbed(S_12, k, h);
            const Sim3 minus = LeftPerturbed(S_12, k, -h);
            numeric12.col(k) = (Sim3Pixel(camera1, plus, X2) - Sim3Pixel(camera1, minus, X2)) / (2.0 * h);
            numeric21.col(k) =
                (InverseSim3Pixel(camera2, plus, X1) - InverseSim3Pixel(camera2, minus, X1)) / (2.0 * h);
        }
        EXPECT_LT(RelativeError(jacobian12, numeric12), 1e-6) << "trial " << trial;
        EXPECT_LT(RelativeError(jacobian21, numeric21), 1e-6) << "trial " << trial;
    }
}

// Noise-free matches, fisheye camera 1 and pinhole camera 2, the start 3 degrees, 0.1 m and
// 10 % in scale off: every seed of the range recovers S_12 to numerical precision.
TEST(Sim3OptimizationTest, RecoversPerturbedSim3) {
    const auto model = TumViModel();
    const Camera camera1 = TumViCamera(&model);
    const Camera camera2 = PinholeCamera();
    for (uint32_t seed = 0; seed < 20; ++seed) {
        std::mt19937 rng(seed);
        const Scene scene = MakeScene(camera1, camera2, RandomSim3(rng, 0.7 + 0.05 * seed), 60, 0.0, rng);
        const Sim3 initial = Perturbed(scene.S_12, rng, false);

        const Sim3OptimizationResult result = OptimizeSim3(camera1, camera2, scene.matches, initial, false);
        ASSERT_TRUE(result.success) << "seed " << seed;
        EXPECT_EQ(result.num_inliers, scene.matches.size()) << "seed " << seed;
        EXPECT_NEAR(result.S_12.scale, scene.S_12.scale, 1e-6) << "seed " << seed;
        EXPECT_LT(RotationErrorDeg(result.S_12, scene.S_12), 1e-5) << "seed " << seed;
        EXPECT_LT((result.S_12.t - scene.S_12.t).norm(), 1e-6) << "seed " << seed;
    }
}

// Stereo: the scale stays exactly at the start's, even when the data pulls it elsewhere;
// with a true scale of 1 the rotation and translation are recovered.
TEST(Sim3OptimizationTest, FixScaleKeepsScale) {
    const auto model = TumViModel();
    const Camera camera = TumViCamera(&model);
    for (uint32_t seed = 0; seed < 10; ++seed) {
        std::mt19937 rng(seed);
        const Scene metric = MakeScene(camera, camera, RandomSim3(rng, 1.0), 60, 0.0, rng);
        const Sim3OptimizationResult result =
            OptimizeSim3(camera, camera, metric.matches, Perturbed(metric.S_12, rng, true), true);
        ASSERT_TRUE(result.success) << "seed " << seed;
        EXPECT_EQ(result.S_12.scale, 1.0) << "seed " << seed;
        EXPECT_LT(RotationErrorDeg(result.S_12, metric.S_12), 1e-5) << "seed " << seed;
        EXPECT_LT((result.S_12.t - metric.S_12.t).norm(), 1e-6) << "seed " << seed;

        const Scene scaled = MakeScene(camera, camera, RandomSim3(rng, 1.3), 60, 0.0, rng);
        Sim3 initial = scaled.S_12;
        initial.scale = 1.0;
        EXPECT_EQ(OptimizeSim3(camera, camera, scaled.matches, initial, true).S_12.scale, 1.0) << "seed " << seed;
    }
}

// 30 % of the matches have one observation moved 20-60 sigma (alternating images), every
// observation 1-sigma Gaussian noise. Over a fixed seed range every outlier is rejected, the
// inlier recall stays >= 95 % (chi2 10 at 2 dof in two images keeps ~98.7 %) and S_12 is that
// of the outlier-free matches (equal unless a true inlier was rejected; at most 0.6 % in scale
// and 0.02 degrees over the range).
TEST(Sim3OptimizationTest, RejectsOutliers) {
    const auto model = TumViModel();
    const Camera camera1 = TumViCamera(&model);
    const Camera camera2 = PinholeCamera();
    size_t true_inliers = 0;
    size_t kept_inliers = 0;
    for (uint32_t seed = 0; seed < 20; ++seed) {
        std::mt19937 rng(seed);
        Scene scene = MakeScene(camera1, camera2, RandomSim3(rng, 0.8 + 0.03 * seed), 100, 1.0, rng);
        std::uniform_real_distribution<double> offset(20.0, 60.0);
        std::vector<bool> outlier(scene.matches.size(), false);
        for (size_t i = 0; i < scene.matches.size(); i += 10) {
            for (size_t j = i; j < i + 3; ++j) {
                Sim3Match& match = scene.matches[j];
                const TPoint2DF shift = offset(rng) * RandomUnit(rng).head<2>().normalized();
                if (j % 2 == 0) {
                    match.pixel1 += match.sigma1 * shift;
                } else {
                    match.pixel2 += match.sigma2 * shift;
                }
                outlier[j] = true;
            }
        }
        const Sim3OptimizationResult result =
            OptimizeSim3(camera1, camera2, scene.matches, Perturbed(scene.S_12, rng, false), false);
        ASSERT_TRUE(result.success) << "seed " << seed;
        ASSERT_EQ(result.inlier.size(), scene.matches.size());
        size_t num_inliers = 0;
        for (size_t i = 0; i < outlier.size(); ++i) {
            num_inliers += result.inlier[i] ? 1 : 0;
            if (outlier[i]) {
                EXPECT_FALSE(result.inlier[i]) << "seed " << seed << " match " << i;
            } else {
                ++true_inliers;
                kept_inliers += result.inlier[i] ? 1 : 0;
            }
        }
        EXPECT_EQ(result.num_inliers, num_inliers) << "seed " << seed;
        // The noise alone moves the optimum (~1-2 % in scale); the outliers must not.
        std::vector<Sim3Match> clean;
        for (size_t i = 0; i < outlier.size(); ++i) {
            if (!outlier[i]) clean.push_back(scene.matches[i]);
        }
        const Sim3OptimizationResult reference = OptimizeSim3(camera1, camera2, clean, scene.S_12, false);
        ASSERT_TRUE(reference.success) << "seed " << seed;
        EXPECT_LT(RotationErrorDeg(result.S_12, reference.S_12), 0.1) << "seed " << seed;
        EXPECT_NEAR(result.S_12.scale / reference.S_12.scale, 1.0, 0.01) << "seed " << seed;
    }
    EXPECT_GE(static_cast<double>(kept_inliers), 0.95 * static_cast<double>(true_inliers));
}

// Fewer than 10 matches survive the first stage (ORB-SLAM3): failure, nothing is an inlier,
// the start is returned unchanged.
TEST(Sim3OptimizationTest, TooFewMatchesFails) {
    const Camera camera = PinholeCamera();
    std::mt19937 rng(3);
    const Scene scene = MakeScene(camera, camera, RandomSim3(rng, 1.0), 9, 0.0, rng);
    const Sim3OptimizationResult result = OptimizeSim3(camera, camera, scene.matches, scene.S_12, false);
    EXPECT_FALSE(result.success);
    EXPECT_EQ(result.num_inliers, 0u);
    EXPECT_EQ(result.inlier, std::vector<bool>(scene.matches.size(), false));
    EXPECT_EQ(result.S_12.scale, scene.S_12.scale);
    EXPECT_EQ(result.S_12.t, scene.S_12.t);
}

}  // namespace
}  // namespace voyahslam::optimization
