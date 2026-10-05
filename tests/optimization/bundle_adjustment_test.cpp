#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <random>

#include <Eigen/Dense>

#include "support/two_view_scene.h"
#include "voyahslam/geometry/camera.h"
#include "voyahslam/geometry/distortion_models/kannala_brandt_distortion_model.h"
#include "voyahslam/geometry/rigid_transformation.h"
#include "voyahslam/optimization/bundle_adjustment.h"

namespace voyahslam::optimization {
namespace {

using geometry::Camera;
using geometry::RigidTransformation;
using geometry::TMatrix33F;
using geometry::TPoint2DF;
using geometry::TPoint3DF;
using geometry::TPoints3DF;
using geometry::TVector3DF;

/// The scenes' normalized coordinates as pixels: identity K, so measurements stay normalized.
const Camera kNormalizedCamera{TMatrix33F::Identity(), nullptr};

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
/// by 2 degrees and 10 % of its translation norm) and the scene points scaled by 1.05 and
/// jittered by 2 %; no cameras or observations yet.
BaProblem MakePerturbedStart(const test::TwoViewScene& scene, uint32_t seed) {
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
    return problem;
}

/// MakePerturbedStart with identity-K cameras and the normalized x1 / x2 observations of every
/// point (sigma = 1 px at the scenes' 190 px focal length, in normalized units).
BaProblem MakePerturbedProblem(const test::TwoViewScene& scene, uint32_t seed) {
    BaProblem problem = MakePerturbedStart(scene, seed);
    problem.keyframe_cameras = {kNormalizedCamera, kNormalizedCamera};
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

    BaProblem nan_camera = valid;
    nan_camera.keyframe_cameras[1].K(0, 0) = std::numeric_limits<double>::quiet_NaN();
    EXPECT_FALSE(BundleAdjust(nan_camera, BundleAdjustmentOptions{}).success);
}

TEST(BundleAdjustTest, CameraCountMismatchFails) {
    const test::TwoViewScene scene =
        test::MakeGeneralScene(20, 0.0, 0.0, 3, test::DefaultTransform());
    const BaProblem valid = MakePerturbedProblem(scene, 13);
    ASSERT_TRUE(BundleAdjust(valid, BundleAdjustmentOptions{}).success);

    BaProblem no_cameras = valid;
    no_cameras.keyframe_cameras.clear();
    EXPECT_FALSE(BundleAdjust(no_cameras, BundleAdjustmentOptions{}).success);

    BaProblem one_camera = valid;
    one_camera.keyframe_cameras.pop_back();
    EXPECT_FALSE(BundleAdjust(one_camera, BundleAdjustmentOptions{}).success);

    BaProblem three_cameras = valid;
    three_cameras.keyframe_cameras.push_back(kNormalizedCamera);
    EXPECT_FALSE(BundleAdjust(three_cameras, BundleAdjustmentOptions{}).success);
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

// With identity-K cameras, normalized measurements and sigma / f, the pixel formulation is
// exactly the normalized-plane one: a real pinhole K with pixel measurements and sigma in px
// must give the same result (pinhole invariance).
TEST(BundleAdjustTest, PinholeInvariance) {
    constexpr double kFocal = 190.0;   // NormalizedSigma's focal length.
    const test::TwoViewScene scene = test::MakeGeneralScene(
        300, 0.10, test::NormalizedSigma(0.5), 2, test::DefaultTransform());
    const BaProblem normalized = MakePerturbedProblem(scene, 12);

    Camera pinhole;
    pinhole.K << kFocal, 0.0, 320.0,
                 0.0, kFocal, 240.0,
                 0.0, 0.0, 1.0;
    BaProblem pixel = normalized;
    pixel.keyframe_cameras = {pinhole, pinhole};
    for (BaObservation& observation : pixel.observations) {
        observation.measurement = (pinhole.K * observation.measurement.homogeneous()).hnormalized();
        observation.sigma = 1.0;   // px
    }

    const BaResult a = BundleAdjust(pixel, BundleAdjustmentOptions{});
    const BaResult b = BundleAdjust(normalized, BundleAdjustmentOptions{});
    ASSERT_TRUE(a.success);
    ASSERT_TRUE(b.success);
    ASSERT_EQ(a.keyframe_poses.size(), b.keyframe_poses.size());
    for (size_t k = 0; k < a.keyframe_poses.size(); ++k) {
        EXPECT_LT((a.keyframe_poses[k].rotation() - b.keyframe_poses[k].rotation())
                      .cwiseAbs().maxCoeff(), 1e-9) << "keyframe " << k;
        EXPECT_LT((a.keyframe_poses[k].translation() - b.keyframe_poses[k].translation())
                      .cwiseAbs().maxCoeff(), 1e-9) << "keyframe " << k;
    }
    ASSERT_EQ(a.points.cols(), b.points.cols());
    for (Eigen::Index i = 0; i < a.points.cols(); ++i) {
        EXPECT_LT((a.points.col(i) - b.points.col(i)).norm(),
                  1e-9 * std::max(1.0, b.points.col(i).norm())) << "point " << i;
    }
    EXPECT_EQ(a.inlier, b.inlier);
    EXPECT_EQ(a.iterations, b.iterations);
    ASSERT_EQ(a.chi2.size(), b.chi2.size());
    for (size_t i = 0; i < a.chi2.size(); ++i) {
        EXPECT_NEAR(a.chi2[i], b.chi2[i], 1e-9 * std::max(1.0, b.chi2[i])) << "observation " << i;
    }
    EXPECT_NEAR(a.initial_cost, b.initial_cost, 1e-9 * b.initial_cost);
    EXPECT_NEAR(a.final_cost, b.final_cost, 1e-9 * b.final_cost);
    // The outliers were flagged, so the comparison covers both stages.
    EXPECT_GT(std::count(a.inlier.begin(), a.inlier.end(), false), 0);
}

/// Points whose rays in camera 1 are up to 80 degrees off-axis (4-8 m away) and within 85
/// degrees of camera 2's axis (camera 2 = DefaultTransform()), observed noise-free through the
/// TUM-VI Kannala-Brandt cameras.
test::TwoViewScene MakeFisheyeScene(size_t num_points, uint32_t seed) {
    std::mt19937 rng(seed);
    std::uniform_real_distribution<double> unit(0.0, 1.0);
    test::TwoViewScene scene;
    scene.T_21 = test::DefaultTransform();
    scene.points.resize(3, static_cast<Eigen::Index>(num_points));
    const double max_angle_2 = 85.0 * M_PI / 180.0;
    size_t num_wide = 0;
    for (Eigen::Index i = 0; i < scene.points.cols();) {
        const double theta = 80.0 * M_PI / 180.0 * unit(rng);
        const double phi = 2.0 * M_PI * unit(rng);
        const double range = 4.0 + 4.0 * unit(rng);
        const TPoint3DF x1 = range * TPoint3DF(std::sin(theta) * std::cos(phi),
                                               std::sin(theta) * std::sin(phi), std::cos(theta));
        const TPoint3DF x2 = scene.T_21.Transform(x1);
        if (AngleBetween(x2, TVector3DF::UnitZ()) >= max_angle_2) {
            continue;
        }
        num_wide += theta > 60.0 * M_PI / 180.0 ? 1 : 0;
        scene.points.col(i++) = x1;
    }
    EXPECT_GT(num_wide, num_points / 10);   // the wide-angle region is exercised
    return scene;
}

TEST(BundleAdjustTest, FisheyeRecoversPerturbedNoiseFree) {
    const auto model = TumViModel();
    const Camera camera = TumViCamera(&model);
    const test::TwoViewScene scene = MakeFisheyeScene(200, 21);
    BaProblem problem = MakePerturbedStart(scene, 11);
    problem.keyframe_cameras = {camera, camera};
    for (Eigen::Index i = 0; i < scene.points.cols(); ++i) {
        const auto point = static_cast<size_t>(i);
        const TPoint3DF x1 = scene.points.col(i);
        problem.observations.push_back({0, point, camera.Project(x1), 1.0});
        problem.observations.push_back({1, point, camera.Project(scene.T_21.Transform(x1)), 1.0});
    }
    BundleAdjustmentOptions options;
    options.iterations = 50;

    const BaResult result = BundleAdjust(problem, options);

    ASSERT_TRUE(result.success);
    EXPECT_LE(result.final_cost, result.initial_cost);
    ASSERT_EQ(result.points.cols(), scene.points.cols());
    ExpectMatchesScene(result, scene, scene.points.cols(), 1e-6);
}

}  // namespace
}  // namespace voyahslam::optimization
