#include <gtest/gtest.h>

#include <cmath>
#include <cstddef>
#include <limits>
#include <optional>
#include <random>
#include <vector>

#include <Eigen/Dense>

#include "support/synthetic_sequence.h"
#include "voyahslam/geometry/rigid_transformation.h"
#include "voyahslam/optimization/pose_optimization.h"

namespace voyahslam::optimization {
namespace {

using geometry::RigidTransformation;
using geometry::TMatrix33F;
using geometry::TPoint2DF;
using geometry::TPoint3DF;
using geometry::TVector3DF;

constexpr double kFocal = 380.0;   // SyntheticStereoIntrinsics().
constexpr double kSigma = 1.0 / kFocal;

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

TPoint2DF Project(const TPoint3DF& xc) { return TPoint2DF(xc.x() / xc.z(), xc.y() / xc.z()); }

struct Scene {
    RigidTransformation T_cw;           ///< Ground truth.
    RigidTransformation initial_T_cw;   ///< Perturbed by 3 degrees and 0.2 m.
    RigidTransformation T_rl;
    std::vector<PoseObservation> observations;
    double mean_depth = 0.0;
};

/// num_points world points 4-8 m in front of a camera with a non-trivial T_cw, their exact
/// normalized left (and, with stereo, right) observations, sigma = 1 px.
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
    scene.T_rl = test::SyntheticStereoRig(0.11).T_rl;

    const RigidTransformation T_wc = scene.T_cw.Inverse();
    for (size_t i = 0; i < num_points; ++i) {
        const double depth = 6.0 + 2.0 * unit(rng);
        const TPoint3DF xc(depth * 0.6 * unit(rng), depth * 0.6 * unit(rng), depth);
        scene.mean_depth += depth / static_cast<double>(num_points);
        PoseObservation observation{T_wc.Transform(xc), Project(xc), std::nullopt, kSigma};
        if (stereo) {
            observation.right = Project(scene.T_rl.Transform(xc));
        }
        scene.observations.push_back(observation);
    }
    return scene;
}

TEST(PoseOptimizationTest, MonoRecoversPose) {
    const Scene scene = MakeScene(100, false, 1);
    const PoseOptimizationResult result =
        OptimizePose(scene.initial_T_cw, std::nullopt, scene.observations, {});
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
        OptimizePose(scene.initial_T_cw, scene.T_rl, scene.observations, {});
    ASSERT_TRUE(result.success);
    EXPECT_LT(RotationErrorDeg(result.T_cw, scene.T_cw), 1e-6);
    EXPECT_LT(TranslationError(result.T_cw, scene.T_cw), 1e-6);
    EXPECT_EQ(result.num_inliers, 100U);
}

TEST(PoseOptimizationTest, FlagsOutliers) {
    Scene scene = MakeScene(200, false, 3);
    std::mt19937 rng(33);
    std::normal_distribution<double> noise(0.0, 0.5 / kFocal);
    std::uniform_real_distribution<double> position(-0.6, 0.6);
    std::vector<bool> is_outlier(scene.observations.size(), false);
    for (size_t i = 0; i < scene.observations.size(); ++i) {
        PoseObservation& observation = scene.observations[i];
        if (i % 5 == 0) {   // 20 %
            observation.left = TPoint2DF(position(rng), position(rng));
            is_outlier[i] = true;
        } else {
            observation.left += TPoint2DF(noise(rng), noise(rng));
        }
    }

    const PoseOptimizationResult result =
        OptimizePose(scene.initial_T_cw, std::nullopt, scene.observations, {});
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
    std::normal_distribution<double> noise(0.0, 0.5 / kFocal);
    for (PoseObservation& observation : scene.observations) {
        observation.left += TPoint2DF(noise(rng), noise(rng));
    }
    const PoseOptimizationResult mono =
        OptimizePose(scene.initial_T_cw, std::nullopt, scene.observations, {});
    const PoseOptimizationResult stereo =
        OptimizePose(scene.initial_T_cw, scene.T_rl, scene.observations, {});
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

    EXPECT_FALSE(OptimizePose(scene.initial_T_cw, scene.T_rl, {}, {}).success);

    std::vector<PoseObservation> with_nan = scene.observations;
    with_nan[3].left.x() = nan;
    EXPECT_FALSE(OptimizePose(scene.initial_T_cw, scene.T_rl, with_nan, {}).success);

    std::vector<PoseObservation> with_nan_point = scene.observations;
    with_nan_point[4].point.z() = nan;
    EXPECT_FALSE(OptimizePose(scene.initial_T_cw, scene.T_rl, with_nan_point, {}).success);

    const RigidTransformation nan_pose(scene.initial_T_cw.rotation(), TVector3DF(nan, 0.0, 0.0));
    EXPECT_FALSE(OptimizePose(nan_pose, scene.T_rl, scene.observations, {}).success);

    std::vector<PoseObservation> zero_sigma = scene.observations;
    zero_sigma[5].sigma = 0.0;
    EXPECT_FALSE(OptimizePose(scene.initial_T_cw, scene.T_rl, zero_sigma, {}).success);

    // `right` set without T_rl.
    EXPECT_FALSE(OptimizePose(scene.initial_T_cw, std::nullopt, scene.observations, {}).success);

    // Sanity: the same observations with T_rl succeed.
    EXPECT_TRUE(OptimizePose(scene.initial_T_cw, scene.T_rl, scene.observations, {}).success);
}

TEST(PoseOptimizationTest, TooFewInliersFails) {
    Scene scene = MakeScene(2, false, 6);
    const PoseOptimizationResult result =
        OptimizePose(scene.initial_T_cw, std::nullopt, scene.observations, {});
    EXPECT_FALSE(result.success);
}

}  // namespace
}  // namespace voyahslam::optimization
