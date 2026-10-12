#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <memory>
#include <optional>
#include <random>
#include <utility>
#include <vector>

#include <Eigen/Dense>

#include "support/synthetic_sequence.h"
#include "voyahslam/geometry/camera.h"
#include "voyahslam/geometry/distortion_models/kannala_brandt_distortion_model.h"
#include "voyahslam/geometry/rigid_transformation.h"
#include "voyahslam/optimization/pose_optimization.h"

namespace voyahslam::optimization {
namespace {

using geometry::Camera;
using geometry::RigidTransformation;
using geometry::TMatrix33F;
using geometry::TPoint2DF;
using geometry::TPoint3DF;
using geometry::TVector3DF;

constexpr double kFocal = 380.0;   // SyntheticStereoIntrinsics().
constexpr double kSigma = 1.0;     // px

Camera PinholeCamera() { return Camera{test::SyntheticStereoIntrinsics(), nullptr}; }

// TUM-VI cam0 (equidistant fisheye).
geometry::distortion_models::TDistortionModelPtr TumViModel() {
    return std::make_shared<const geometry::distortion_models::KannalaBrandtDistortionModel>(
        0.010171079892421483, -0.010816440029919381, 0.005942781769412756, -0.001662284667857643);
}

Camera TumViCamera(geometry::distortion_models::TDistortionModelPtr model) {
    Camera camera;
    camera.K << 380.81042871360756, 0.0, 510.29465304840727,
                0.0, 380.81194179427075, 514.3304630538506,
                0.0, 0.0, 1.0;
    camera.distortion = std::move(model);
    return camera;
}

TVector3DF RandomUnit(std::mt19937& rng) {
    std::normal_distribution<double> normal(0.0, 1.0);
    return TVector3DF(normal(rng), normal(rng), normal(rng)).normalized();
}

double RotationErrorDeg(const RigidTransformation& a, const RigidTransformation& b) {
    return Eigen::AngleAxisd(a.rotation().transpose() * b.rotation()).angle() * 180.0 / M_PI;
}

double TranslationError(const RigidTransformation& a, const RigidTransformation& b) {
    return (a.translation() - b.translation()).norm();
}

struct Scene {
    RigidTransformation T_cw;           ///< Ground truth.
    RigidTransformation initial_T_cw;   ///< Perturbed by 3 degrees and 0.2 m.
    Camera camera = PinholeCamera();
    StereoCameras stereo;
    std::vector<PoseObservation> observations;
    double mean_depth = 0.0;
};

/// num_points world points 4-8 m in front of a pinhole camera with a non-trivial T_cw, their
/// exact pixel left (and, with stereo, right) observations, sigma = 1 px.
Scene MakeScene(size_t num_points, bool stereo, uint32_t seed) {
    std::mt19937 rng(seed);
    std::uniform_real_distribution<double> unit(-1.0, 1.0);
    Scene scene;
    scene.T_cw = RigidTransformation(
        TMatrix33F(Eigen::AngleAxisd(0.3, TVector3DF(0.2, 1.0, -0.1).normalized())),
        TVector3DF(0.4, -0.2, 1.3));
    scene.initial_T_cw = RigidTransformation(
        TMatrix33F(Eigen::AngleAxisd(3.0 * M_PI / 180.0, RandomUnit(rng))) * scene.T_cw.rotation(),
        scene.T_cw.translation() + 0.2 * RandomUnit(rng));
    scene.stereo = StereoCameras{PinholeCamera(), test::SyntheticStereoRig(0.11).T_rl};

    const RigidTransformation T_wc = scene.T_cw.Inverse();
    for (size_t i = 0; i < num_points; ++i) {
        const double depth = 6.0 + 2.0 * unit(rng);
        const TPoint3DF xc(depth * 0.6 * unit(rng), depth * 0.6 * unit(rng), depth);
        scene.mean_depth += depth / static_cast<double>(num_points);
        PoseObservation observation{T_wc.Transform(xc), scene.camera.Project(xc), std::nullopt,
                                    kSigma};
        if (stereo) {
            observation.right = scene.stereo.right.Project(scene.stereo.T_rl.Transform(xc));
        }
        scene.observations.push_back(observation);
    }
    return scene;
}

TEST(PoseOptimizationTest, MonoRecoversPose) {
    const Scene scene = MakeScene(100, false, 1);
    const PoseOptimizationResult result =
        OptimizePose(scene.initial_T_cw, scene.camera, std::nullopt, scene.observations, {});
    ASSERT_TRUE(result.success);
    EXPECT_LT(RotationErrorDeg(result.T_cw, scene.T_cw), 1e-6);
    EXPECT_LT(TranslationError(result.T_cw, scene.T_cw), 1e-6);
    EXPECT_EQ(result.num_inliers, 100U);
    ASSERT_EQ(result.inlier.size(), 100U);
    for (bool inlier : result.inlier) {
        EXPECT_TRUE(inlier);
    }
}

TEST(PoseOptimizationTest, StereoRecoversMetricPose) {
    const Scene scene = MakeScene(100, true, 2);
    const PoseOptimizationResult result =
        OptimizePose(scene.initial_T_cw, scene.camera, scene.stereo, scene.observations, {});
    ASSERT_TRUE(result.success);
    EXPECT_LT(RotationErrorDeg(result.T_cw, scene.T_cw), 1e-6);
    EXPECT_LT(TranslationError(result.T_cw, scene.T_cw), 1e-6);
    EXPECT_EQ(result.num_inliers, 100U);
}

TEST(PoseOptimizationTest, FlagsOutliers) {
    Scene scene = MakeScene(200, false, 3);
    std::mt19937 rng(33);
    std::normal_distribution<double> noise(0.0, 0.5);   // px
    std::uniform_real_distribution<double> position(-0.6, 0.6);
    std::vector<bool> is_outlier(scene.observations.size(), false);
    for (size_t i = 0; i < scene.observations.size(); ++i) {
        PoseObservation& observation = scene.observations[i];
        if (i % 5 == 0) {   // 20 %
            observation.left = scene.camera.Project(TPoint3DF(position(rng), position(rng), 1.0));
            is_outlier[i] = true;
        } else {
            observation.left += TPoint2DF(noise(rng), noise(rng));
        }
    }

    const PoseOptimizationResult result =
        OptimizePose(scene.initial_T_cw, scene.camera, std::nullopt, scene.observations, {});
    ASSERT_TRUE(result.success);
    ASSERT_EQ(result.inlier.size(), scene.observations.size());
    size_t num_outliers = 0;
    size_t num_flagged = 0;
    for (size_t i = 0; i < is_outlier.size(); ++i) {
        if (is_outlier[i]) {
            ++num_outliers;
            num_flagged += result.inlier[i] ? 0 : 1;
        }
    }
    EXPECT_GE(static_cast<double>(num_flagged), 0.95 * static_cast<double>(num_outliers));
    EXPECT_LT(RotationErrorDeg(result.T_cw, scene.T_cw), 0.2);
    EXPECT_LT(TranslationError(result.T_cw, scene.T_cw), 0.01 * scene.mean_depth);
}

TEST(PoseOptimizationTest, StereoWithoutRightObservationsBehavesAsMono) {
    Scene scene = MakeScene(100, false, 4);
    std::mt19937 rng(44);
    std::normal_distribution<double> noise(0.0, 0.5);   // px
    for (PoseObservation& observation : scene.observations) {
        observation.left += TPoint2DF(noise(rng), noise(rng));
    }
    const PoseOptimizationResult mono =
        OptimizePose(scene.initial_T_cw, scene.camera, std::nullopt, scene.observations, {});
    const PoseOptimizationResult stereo =
        OptimizePose(scene.initial_T_cw, scene.camera, scene.stereo, scene.observations, {});
    ASSERT_TRUE(mono.success);
    ASSERT_TRUE(stereo.success);
    EXPECT_LT((mono.T_cw.rotation() - stereo.T_cw.rotation()).cwiseAbs().maxCoeff(), 1e-9);
    EXPECT_LT((mono.T_cw.translation() - stereo.T_cw.translation()).cwiseAbs().maxCoeff(), 1e-9);
    EXPECT_EQ(mono.inlier, stereo.inlier);
    EXPECT_EQ(mono.num_inliers, stereo.num_inliers);
}

TEST(PoseOptimizationTest, InvalidInputFails) {
    const Scene scene = MakeScene(20, true, 5);
    const double nan = std::numeric_limits<double>::quiet_NaN();

    EXPECT_FALSE(OptimizePose(scene.initial_T_cw, scene.camera, scene.stereo, {}, {}).success);

    std::vector<PoseObservation> with_nan = scene.observations;
    with_nan[3].left.x() = nan;
    EXPECT_FALSE(OptimizePose(scene.initial_T_cw, scene.camera, scene.stereo, with_nan, {})
                     .success);

    std::vector<PoseObservation> with_nan_point = scene.observations;
    with_nan_point[4].point.z() = nan;
    EXPECT_FALSE(OptimizePose(scene.initial_T_cw, scene.camera, scene.stereo, with_nan_point, {})
                     .success);

    const RigidTransformation nan_pose(scene.initial_T_cw.rotation(), TVector3DF(nan, 0.0, 0.0));
    EXPECT_FALSE(OptimizePose(nan_pose, scene.camera, scene.stereo, scene.observations, {})
                     .success);

    std::vector<PoseObservation> zero_sigma = scene.observations;
    zero_sigma[5].sigma = 0.0;
    EXPECT_FALSE(OptimizePose(scene.initial_T_cw, scene.camera, scene.stereo, zero_sigma, {})
                     .success);

    Camera nan_camera = scene.camera;
    nan_camera.K(0, 0) = nan;
    EXPECT_FALSE(OptimizePose(scene.initial_T_cw, nan_camera, scene.stereo, scene.observations, {})
                     .success);
    StereoCameras nan_rig = scene.stereo;
    nan_rig.T_rl = RigidTransformation(nan_rig.T_rl.rotation(), TVector3DF(nan, 0.0, 0.0));
    EXPECT_FALSE(OptimizePose(scene.initial_T_cw, scene.camera, nan_rig, scene.observations, {})
                     .success);

    // Sanity: the same observations with T_rl succeed.
    EXPECT_TRUE(OptimizePose(scene.initial_T_cw, scene.camera, scene.stereo, scene.observations, {})
                    .success);
}

TEST(PoseOptimizationTest, TooFewInliersFails) {
    Scene scene = MakeScene(2, false, 6);
    const PoseOptimizationResult result =
        OptimizePose(scene.initial_T_cw, scene.camera, std::nullopt, scene.observations, {});
    EXPECT_FALSE(result.success);
}

TEST(PoseOptimizationTest, RightWithoutStereoCamerasFails) {
    const Scene scene = MakeScene(20, true, 7);
    EXPECT_FALSE(OptimizePose(scene.initial_T_cw, scene.camera, std::nullopt, scene.observations, {})
                     .success);
    EXPECT_TRUE(OptimizePose(scene.initial_T_cw, scene.camera, scene.stereo, scene.observations, {})
                    .success);
}

/// Pixel chi2 of every observation at pose T_cw (mono rows only).
std::vector<double> MonoChi2(const Camera& camera, const RigidTransformation& T_cw,
                             const std::vector<PoseObservation>& observations) {
    std::vector<double> chi2;
    for (const PoseObservation& observation : observations) {
        const TPoint2DF error =
            camera.Project(T_cw.Transform(observation.point)) - observation.left;
        chi2.push_back(error.squaredNorm() / (observation.sigma * observation.sigma));
    }
    return chi2;
}

// With an identity-K pinhole camera, normalized measurements and sigma / f, the pixel
// formulation is exactly the normalized-plane one: both must agree (pinhole invariance).
TEST(PoseOptimizationTest, PinholeInvariance) {
    Scene pixel = MakeScene(200, false, 8);
    std::mt19937 rng(88);
    std::normal_distribution<double> noise(0.0, 0.5);   // px
    std::uniform_real_distribution<double> position(-0.6, 0.6);
    for (size_t i = 0; i < pixel.observations.size(); ++i) {
        PoseObservation& observation = pixel.observations[i];
        if (i % 10 == 0) {   // 10 %
            observation.left = pixel.camera.Project(TPoint3DF(position(rng), position(rng), 1.0));
        } else {
            observation.left += TPoint2DF(noise(rng), noise(rng));
        }
    }

    const Camera normalized_camera{TMatrix33F::Identity(), nullptr};
    const TMatrix33F K_inverse = pixel.camera.K.inverse();
    std::vector<PoseObservation> normalized = pixel.observations;
    for (PoseObservation& observation : normalized) {
        observation.left = (K_inverse * observation.left.homogeneous()).hnormalized();
        observation.sigma = kSigma / kFocal;
    }

    const PoseOptimizationResult a =
        OptimizePose(pixel.initial_T_cw, pixel.camera, std::nullopt, pixel.observations, {});
    const PoseOptimizationResult b =
        OptimizePose(pixel.initial_T_cw, normalized_camera, std::nullopt, normalized, {});
    ASSERT_TRUE(a.success);
    ASSERT_TRUE(b.success);
    EXPECT_LT((a.T_cw.rotation() - b.T_cw.rotation()).cwiseAbs().maxCoeff(), 1e-9);
    EXPECT_LT((a.T_cw.translation() - b.T_cw.translation()).cwiseAbs().maxCoeff(), 1e-9);
    EXPECT_EQ(a.inlier, b.inlier);
    EXPECT_EQ(a.num_inliers, b.num_inliers);
    EXPECT_LT(a.num_inliers, pixel.observations.size());   // the outliers were flagged

    const std::vector<double> chi2_a = MonoChi2(pixel.camera, a.T_cw, pixel.observations);
    const std::vector<double> chi2_b = MonoChi2(normalized_camera, b.T_cw, normalized);
    ASSERT_EQ(chi2_a.size(), chi2_b.size());
    for (size_t i = 0; i < chi2_a.size(); ++i) {
        EXPECT_NEAR(chi2_a[i], chi2_b[i], 1e-9 * std::max(1.0, chi2_a[i])) << "observation " << i;
    }
}

/// num_points world points whose left-camera rays are up to 80 degrees off-axis (4-8 m away),
/// projected noise-free through the TUM-VI Kannala-Brandt cameras; with stereo, right
/// observations through a 5 degree yaw, 0.11 m rig (kept when in front of the right camera).
Scene MakeFisheyeScene(const geometry::distortion_models::TDistortionModelPtr& model,
                       size_t num_points, bool stereo, uint32_t seed) {
    std::mt19937 rng(seed);
    std::uniform_real_distribution<double> unit(-1.0, 1.0);
    Scene scene;
    scene.camera = TumViCamera(model);
    scene.stereo = StereoCameras{
        TumViCamera(model),
        RigidTransformation(TMatrix33F(Eigen::AngleAxisd(5.0 * M_PI / 180.0, TVector3DF::UnitY())),
                            TVector3DF(-0.11, 0.0, 0.0))};
    scene.T_cw = RigidTransformation(
        TMatrix33F(Eigen::AngleAxisd(0.3, TVector3DF(0.2, 1.0, -0.1).normalized())),
        TVector3DF(0.4, -0.2, 1.3));
    scene.initial_T_cw = RigidTransformation(
        TMatrix33F(Eigen::AngleAxisd(3.0 * M_PI / 180.0, RandomUnit(rng))) * scene.T_cw.rotation(),
        scene.T_cw.translation() + 0.2 * RandomUnit(rng));

    const RigidTransformation T_wc = scene.T_cw.Inverse();
    size_t num_wide = 0;
    for (size_t i = 0; i < num_points; ++i) {
        const double theta = 80.0 * M_PI / 180.0 * 0.5 * (unit(rng) + 1.0);
        const double phi = M_PI * unit(rng);
        const double range = 6.0 + 2.0 * unit(rng);
        const TPoint3DF xc = range * TPoint3DF(std::sin(theta) * std::cos(phi),
                                               std::sin(theta) * std::sin(phi), std::cos(theta));
        num_wide += theta > 60.0 * M_PI / 180.0 ? 1 : 0;
        PoseObservation observation{T_wc.Transform(xc), scene.camera.Project(xc), std::nullopt,
                                    kSigma};
        const TPoint3DF xr = scene.stereo.T_rl.Transform(xc);
        if (stereo && xr.z() > 0.0) {
            observation.right = scene.stereo.right.Project(xr);
        }
        scene.observations.push_back(observation);
    }
    EXPECT_GT(num_wide, num_points / 10);   // the wide-angle region is exercised
    return scene;
}

TEST(PoseOptimizationTest, FisheyeMonoRecoversPose) {
    const auto model = TumViModel();
    const Scene scene = MakeFisheyeScene(model, 100, false, 9);
    const PoseOptimizationResult result =
        OptimizePose(scene.initial_T_cw, scene.camera, std::nullopt, scene.observations, {});
    ASSERT_TRUE(result.success);
    EXPECT_LT(RotationErrorDeg(result.T_cw, scene.T_cw), 1e-6);
    EXPECT_LT(TranslationError(result.T_cw, scene.T_cw), 1e-6);
    EXPECT_EQ(result.num_inliers, 100U);
}

TEST(PoseOptimizationTest, FisheyeStereoRecoversPose) {
    const auto model = TumViModel();
    const Scene scene = MakeFisheyeScene(model, 100, true, 10);
    const PoseOptimizationResult result =
        OptimizePose(scene.initial_T_cw, scene.camera, scene.stereo, scene.observations, {});
    ASSERT_TRUE(result.success);
    EXPECT_LT(RotationErrorDeg(result.T_cw, scene.T_cw), 1e-6);
    EXPECT_LT(TranslationError(result.T_cw, scene.T_cw), 1e-6);
    EXPECT_EQ(result.num_inliers, 100U);
}

}  // namespace
}  // namespace voyahslam::optimization
