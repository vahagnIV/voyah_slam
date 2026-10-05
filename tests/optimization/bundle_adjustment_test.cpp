#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <random>

#include <Eigen/Dense>

#include "support/two_view_scene.h"
#include "voyahslam/geometry/rigid_transformation.h"
#include "voyahslam/optimization/bundle_adjustment.h"

namespace voyahslam::optimization {
namespace {

using geometry::RigidTransformation;
using geometry::TMatrix33F;
using geometry::TPoint2DF;
using geometry::TPoint3DF;
using geometry::TPoints3DF;
using geometry::TVector3DF;

TVector3DF RandomUnit(std::mt19937& rng) {
    std::normal_distribution<double> normal(0.0, 1.0);
    return TVector3DF(normal(rng), normal(rng), normal(rng)).normalized();
}

double RotationAngle(const TMatrix33F& a, const TMatrix33F& b) {
    return Eigen::AngleAxisd(a.transpose() * b).angle();
}

double AngleBetween(const TVector3DF& a, const TVector3DF& b) {
    return std::acos(std::clamp(a.normalized().dot(b.normalized()), -1.0, 1.0));
}

/// Two keyframes (world = camera 1; keyframe 0 identity and fixed, keyframe 1 = T_21 perturbed
/// by 2 degrees and 10 % of its translation norm), the scene points scaled by 1.05 and jittered
/// by 2 %, and the x1 / x2 observations of every point.
BaProblem MakePerturbedProblem(const test::TwoViewScene& scene, uint32_t seed) {
    std::mt19937 rng(seed);
    const TVector3DF t_true = scene.T_21.translation();
    const TMatrix33F rotation =
        Eigen::AngleAxisd(2.0 * M_PI / 180.0, RandomUnit(rng)).toRotationMatrix() *
        scene.T_21.rotation();
    const TVector3DF translation = t_true + 0.1 * t_true.norm() * RandomUnit(rng);

    BaProblem problem;
    problem.keyframe_poses = {RigidTransformation::Identity(),
                              RigidTransformation(rotation, translation)};
    problem.keyframe_fixed = {true, false};
    problem.points = 1.05 * scene.points;
    for (Eigen::Index i = 0; i < problem.points.cols(); ++i) {
        problem.points.col(i) += 0.02 * scene.points.col(i).norm() * RandomUnit(rng);
    }
    const double sigma = test::NormalizedSigma(1.0);
    for (Eigen::Index i = 0; i < scene.points.cols(); ++i) {
        const auto point = static_cast<size_t>(i);
        problem.observations.push_back({0, point, scene.correspondences.x1.col(i), sigma});
        problem.observations.push_back({1, point, scene.correspondences.x2.col(i), sigma});
    }
    return problem;
}

/// Checks the result against the noise-free scene up to the monocular scale.
void ExpectMatchesScene(const BaResult& result, const test::TwoViewScene& scene,
                        Eigen::Index num_points, double tolerance) {
    ASSERT_EQ(result.keyframe_poses.size(), 2u);
    EXPECT_EQ(result.keyframe_poses[0].rotation(), TMatrix33F::Identity());
    EXPECT_EQ(result.keyframe_poses[0].translation(), TVector3DF::Zero());

    const TVector3DF t_true = scene.T_21.translation();
    const TVector3DF t_est = result.keyframe_poses[1].translation();
    const double scale = t_true.norm() / t_est.norm();
    EXPECT_LT(RotationAngle(result.keyframe_poses[1].rotation(), scene.T_21.rotation()),
              tolerance);
    EXPECT_LT((scale * t_est - t_true).norm(), tolerance);
    for (Eigen::Index i = 0; i < num_points; ++i) {
        EXPECT_LT((scale * result.points.col(i) - scene.points.col(i)).norm(), tolerance)
            << "point " << i;
    }
}

TEST(BundleAdjustTest, RecoversPerturbedNoiseFree) {
    const test::TwoViewScene scene =
        test::MakeGeneralScene(200, 0.0, 0.0, 1, test::DefaultTransform());
    const BaProblem problem = MakePerturbedProblem(scene, 11);
    BundleAdjustmentOptions options;
    options.iterations = 50;

    const BaResult result = BundleAdjust(problem, options);

    ASSERT_TRUE(result.success);
    EXPECT_LE(result.final_cost, result.initial_cost);
    ASSERT_EQ(result.points.cols(), scene.points.cols());
    ASSERT_EQ(result.chi2.size(), problem.observations.size());
    ASSERT_EQ(result.inlier.size(), problem.observations.size());
    ExpectMatchesScene(result, scene, scene.points.cols(), 1e-6);
}

TEST(BundleAdjustTest, FlagsOutliersAndStaysAccurate) {
    const test::TwoViewScene scene = test::MakeGeneralScene(
        300, 0.10, test::NormalizedSigma(0.5), 2, test::DefaultTransform());
    const BaProblem problem = MakePerturbedProblem(scene, 12);

    const BaResult result = BundleAdjust(problem, BundleAdjustmentOptions{});

    ASSERT_TRUE(result.success);
    EXPECT_LT(result.final_cost, result.initial_cost);

    int outliers = 0;
    int flagged = 0;
    for (Eigen::Index i = 0; i < scene.points.cols(); ++i) {
        if (!scene.is_outlier(i)) {
            continue;
        }
        ++outliers;
        const auto first = static_cast<size_t>(2 * i);
        if (!result.inlier[first] || !result.inlier[first + 1]) {
            ++flagged;
        }
    }
    ASSERT_GT(outliers, 0);
    EXPECT_GE(flagged, static_cast<int>(std::ceil(0.95 * outliers)));

    // Oracle: the same adjustment (same start, same options) on the inlier observations only.
    BaProblem inliers_only = problem;
    inliers_only.observations.clear();
    for (const BaObservation& observation : problem.observations) {
        if (!scene.is_outlier(static_cast<Eigen::Index>(observation.point))) {
            inliers_only.observations.push_back(observation);
        }
    }
    const BaResult oracle = BundleAdjust(inliers_only, BundleAdjustmentOptions{});
    ASSERT_TRUE(oracle.success);

    const double degree = M_PI / 180.0;
    const double direction_error = AngleBetween(result.keyframe_poses[1].translation(),
                                                scene.T_21.translation());
    const double oracle_direction_error = AngleBetween(oracle.keyframe_poses[1].translation(),
                                                       scene.T_21.translation());
    EXPECT_LE(direction_error, oracle_direction_error + 0.5 * degree);
    EXPECT_LT(direction_error, 2.0 * degree);
}

TEST(BundleAdjustTest, RejectsInvalidInput) {
    EXPECT_FALSE(BundleAdjust(BaProblem{}, BundleAdjustmentOptions{}).success);

    const test::TwoViewScene scene =
        test::MakeGeneralScene(20, 0.0, 0.0, 3, test::DefaultTransform());
    const BaProblem valid = MakePerturbedProblem(scene, 13);
    ASSERT_TRUE(BundleAdjust(valid, BundleAdjustmentOptions{}).success);

    BaProblem nan_measurement = valid;
    nan_measurement.observations[3].measurement.x() = std::numeric_limits<double>::quiet_NaN();
    EXPECT_FALSE(BundleAdjust(nan_measurement, BundleAdjustmentOptions{}).success);

    BaProblem zero_sigma = valid;
    zero_sigma.observations[5].sigma = 0.0;
    EXPECT_FALSE(BundleAdjust(zero_sigma, BundleAdjustmentOptions{}).success);

    BaProblem bad_point_index = valid;
    bad_point_index.observations[7].point = static_cast<size_t>(valid.points.cols());
    EXPECT_FALSE(BundleAdjust(bad_point_index, BundleAdjustmentOptions{}).success);

    BaProblem bad_keyframe_index = valid;
    bad_keyframe_index.observations[8].keyframe = 2;
    EXPECT_FALSE(BundleAdjust(bad_keyframe_index, BundleAdjustmentOptions{}).success);

    BaProblem nan_point = valid;
    nan_point.points(1, 4) = std::numeric_limits<double>::infinity();
    EXPECT_FALSE(BundleAdjust(nan_point, BundleAdjustmentOptions{}).success);
}

TEST(BundleAdjustTest, PointBehindCameraIsOutlier) {
    const test::TwoViewScene scene =
        test::MakeGeneralScene(100, 0.0, 0.0, 4, test::DefaultTransform());
    BaProblem problem = MakePerturbedProblem(scene, 14);

    // In front of keyframe 0 but behind keyframe 1.
    const TPoint3DF behind(10.0, 0.0, 0.05);
    const TPoint3DF behind_1 = scene.T_21.Transform(behind);
    ASSERT_GT(behind.z(), 0.0);
    ASSERT_LT(behind_1.z(), 0.0);

    const auto index = static_cast<size_t>(problem.points.cols());
    problem.points.conservativeResize(Eigen::NoChange, problem.points.cols() + 1);
    problem.points.col(static_cast<Eigen::Index>(index)) = behind;
    const double sigma = test::NormalizedSigma(1.0);
    problem.observations.push_back(
        {0, index, TPoint2DF(behind.x() / behind.z(), behind.y() / behind.z()), sigma});
    problem.observations.push_back(
        {1, index, TPoint2DF(behind_1.x() / behind_1.z(), behind_1.y() / behind_1.z()), sigma});

    const BaResult result = BundleAdjust(problem, BundleAdjustmentOptions{});

    ASSERT_TRUE(result.success);
    EXPECT_FALSE(result.inlier.back());
}

TEST(BundleAdjustTest, SingleObservationPointStaysFinite) {
    const test::TwoViewScene scene =
        test::MakeGeneralScene(200, 0.0, 0.0, 1, test::DefaultTransform());
    BaProblem problem = MakePerturbedProblem(scene, 11);

    const TPoint3DF lonely(0.5, -0.3, 6.0);
    const auto index = static_cast<size_t>(problem.points.cols());
    problem.points.conservativeResize(Eigen::NoChange, problem.points.cols() + 1);
    problem.points.col(static_cast<Eigen::Index>(index)) = lonely * 1.05;
    problem.observations.push_back(
        {0, index, TPoint2DF(lonely.x() / lonely.z(), lonely.y() / lonely.z()),
         test::NormalizedSigma(1.0)});
    BundleAdjustmentOptions options;
    options.iterations = 50;

    const BaResult result = BundleAdjust(problem, options);

    ASSERT_TRUE(result.success);
    ASSERT_EQ(result.points.cols(), scene.points.cols() + 1);
    EXPECT_TRUE(result.points.col(static_cast<Eigen::Index>(index)).allFinite());
    ExpectMatchesScene(result, scene, scene.points.cols(), 1e-5);
}

TEST(BundleAdjustTest, RobustFlagChangesCost) {
    const test::TwoViewScene scene = test::MakeGeneralScene(
        300, 0.10, test::NormalizedSigma(0.5), 2, test::DefaultTransform());
    const BaProblem problem = MakePerturbedProblem(scene, 12);
    BundleAdjustmentOptions robust;
    BundleAdjustmentOptions plain;
    plain.robust = false;

    const BaResult robust_result = BundleAdjust(problem, robust);
    const BaResult plain_result = BundleAdjust(problem, plain);

    ASSERT_TRUE(robust_result.success);
    ASSERT_TRUE(plain_result.success);
    EXPECT_GT(std::abs(robust_result.final_cost - plain_result.final_cost),
              1e-6 * std::max(robust_result.final_cost, plain_result.final_cost));
}

}  // namespace
}  // namespace voyahslam::optimization
