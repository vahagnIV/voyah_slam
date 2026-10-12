#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>
#include <memory>
#include <optional>
#include <random>
#include <utility>
#include <vector>

#include <Eigen/Dense>

#include "optimization/projection.h"
#include "support/synthetic_sequence.h"
#include "support/two_view_scene.h"
#include "voyahslam/geometry/camera.h"
#include "voyahslam/geometry/distortion_models/kannala_brandt_distortion_model.h"
#include "voyahslam/geometry/rigid_transformation.h"
#include "voyahslam/optimization/bundle_adjustment.h"
#include "voyahslam/optimization/pose_optimization.h"

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
    const Camera camera = TumViCamera(model);
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

/// exp(delta) * T with delta = (omega, upsilon), the g2o SE3Expmap left update.
RigidTransformation LeftPerturbed(const RigidTransformation& pose, int k, double step) {
    Eigen::Matrix<double, 6, 1> delta = Eigen::Matrix<double, 6, 1>::Zero();
    delta[k] = step;
    const TMatrix33F dr(Eigen::AngleAxisd(delta.head<3>().norm(),
                                          delta.head<3>().norm() == 0.0
                                              ? TVector3DF(TVector3DF::UnitX())
                                              : TVector3DF(delta.head<3>().normalized())));
    return RigidTransformation(dr * pose.rotation(), dr * pose.translation() + delta.tail<3>());
}

template <typename TMatrix>
double RelativeError(const TMatrix& analytic, const TMatrix& numeric) {
    return (analytic - numeric).cwiseAbs().maxCoeff() / numeric.cwiseAbs().maxCoeff();
}

// The stereo edge's residual and Jacobians are StereoPixel / StereoPixelJacobians verbatim.
TEST(BundleAdjustTest, StereoEdgeJacobianMatchesFiniteDifference) {
    const auto model = TumViModel();
    const Camera left = TumViCamera(model);
    const Camera right = TumViCamera(model);
    // A rig with a 5 degree yaw between the cameras and a 0.11 m baseline.
    const RigidTransformation T_rl(
        TMatrix33F(Eigen::AngleAxisd(5.0 * M_PI / 180.0, Eigen::Vector3d::UnitY())),
        TVector3DF(-0.11, 0.0, 0.0));
    std::mt19937 rng(5);
    std::uniform_real_distribution<double> unit(-1.0, 1.0);
    const double h = 1e-6;

    for (int trial = 0; trial < 10; ++trial) {
        const RigidTransformation pose(
            TMatrix33F(Eigen::AngleAxisd(0.5 * unit(rng), RandomUnit(rng))),
            TVector3DF(unit(rng), unit(rng), unit(rng)));
        // Up to 60 degrees off the left axis, 2-10 m away: in front of both cameras.
        const double theta = 60.0 * M_PI / 180.0 * 0.5 * (unit(rng) + 1.0);
        const double phi = M_PI * unit(rng);
        const double range = 2.0 + 4.0 * (unit(rng) + 1.0);
        const TPoint3DF xc = range * TPoint3DF(std::sin(theta) * std::cos(phi),
                                               std::sin(theta) * std::sin(phi), std::cos(theta));
        const TPoint3DF x = pose.Inverse().Transform(xc);

        const Eigen::Vector3d pixel = StereoPixel(left, right, pose, T_rl, x);
        EXPECT_TRUE(pixel.head<2>().isApprox(left.Project(xc), 1e-12));
        EXPECT_NEAR(pixel.z(), right.Project(T_rl.Transform(xc)).x(), 1e-9);

        Eigen::Matrix<precision_t, 3, 6> pose_jacobian;
        Eigen::Matrix<precision_t, 3, 3> point_jacobian;
        StereoPixelJacobians(left, right, pose, T_rl, x, pose_jacobian, point_jacobian);

        Eigen::Matrix<precision_t, 3, 6> pose_numeric;
        for (int k = 0; k < 6; ++k) {
            pose_numeric.col(k) = (StereoPixel(left, right, LeftPerturbed(pose, k, h), T_rl, x) -
                                   StereoPixel(left, right, LeftPerturbed(pose, k, -h), T_rl, x)) /
                                  (2.0 * h);
        }
        Eigen::Matrix<precision_t, 3, 3> point_numeric;
        for (int k = 0; k < 3; ++k) {
            TPoint3DF xp = x;
            TPoint3DF xm = x;
            xp[k] += h;
            xm[k] -= h;
            point_numeric.col(k) = (StereoPixel(left, right, pose, T_rl, xp) -
                                    StereoPixel(left, right, pose, T_rl, xm)) /
                                   (2.0 * h);
        }

        EXPECT_LT(RelativeError(pose_jacobian, pose_numeric), 1e-6) << "trial " << trial;
        EXPECT_LT(RelativeError(point_jacobian, point_numeric), 1e-6) << "trial " << trial;
    }
}

struct StereoScene {
    std::vector<RigidTransformation> T_cw;   ///< Ground truth.
    TPoints3DF points;                       ///< Ground truth.
    BaProblem problem;                       ///< Perturbed start.
};

/// Three pinhole stereo keyframes (keyframe 0 = world, fixed) moving sideways, num_points
/// points 4-8 m away observed noise-free by all of them (left pixel + right x, sigma 1 px).
/// The start perturbs keyframes 1-2 by 2 degrees and 0.1 m, the points by 2 %.
StereoScene MakeStereoScene(size_t num_points, uint32_t seed) {
    std::mt19937 rng(seed);
    std::uniform_real_distribution<double> unit(-1.0, 1.0);
    const Camera camera{test::SyntheticStereoIntrinsics(), nullptr};
    const StereoCameras stereo{camera, test::SyntheticStereoRig(0.11).T_rl};

    StereoScene scene;
    scene.T_cw = {RigidTransformation::Identity(),
                  RigidTransformation(TMatrix33F(Eigen::AngleAxisd(0.05, TVector3DF::UnitY())),
                                      TVector3DF(-0.3, 0.02, 0.05)),
                  RigidTransformation(TMatrix33F(Eigen::AngleAxisd(0.1, TVector3DF::UnitY())),
                                      TVector3DF(-0.6, 0.05, 0.1))};
    scene.points.resize(3, static_cast<Eigen::Index>(num_points));
    for (Eigen::Index i = 0; i < scene.points.cols(); ++i) {
        const double depth = 6.0 + 2.0 * unit(rng);
        scene.points.col(i) = TPoint3DF(depth * 0.5 * unit(rng), depth * 0.5 * unit(rng), depth);
    }

    BaProblem& problem = scene.problem;
    problem.keyframe_poses = scene.T_cw;
    for (size_t k = 1; k < problem.keyframe_poses.size(); ++k) {
        const RigidTransformation& truth = scene.T_cw[k];
        problem.keyframe_poses[k] = RigidTransformation(
            TMatrix33F(Eigen::AngleAxisd(2.0 * M_PI / 180.0, RandomUnit(rng))) * truth.rotation(),
            truth.translation() + 0.1 * RandomUnit(rng));
    }
    problem.keyframe_fixed = {true, false, false};
    problem.keyframe_cameras = {camera, camera, camera};
    problem.keyframe_stereo = {stereo, stereo, stereo};
    problem.points = scene.points;
    for (Eigen::Index i = 0; i < problem.points.cols(); ++i) {
        problem.points.col(i) += 0.02 * scene.points.col(i).norm() * RandomUnit(rng);
    }
    for (size_t k = 0; k < scene.T_cw.size(); ++k) {
        for (Eigen::Index i = 0; i < scene.points.cols(); ++i) {
            const TPoint3DF xc = scene.T_cw[k].Transform(TPoint3DF(scene.points.col(i)));
            problem.observations.push_back(
                {k, static_cast<size_t>(i), camera.Project(xc), 1.0,
                 stereo.right.Project(stereo.T_rl.Transform(xc)).x()});
        }
    }
    return scene;
}

TEST(BundleAdjustTest, StereoRecoversPerturbedNoiseFree) {
    const StereoScene scene = MakeStereoScene(100, 31);
    BundleAdjustmentOptions options;
    options.iterations = 50;

    const BaResult result = BundleAdjust(scene.problem, options);

    ASSERT_TRUE(result.success);
    EXPECT_FALSE(result.interrupted);
    EXPECT_EQ(result.stage_reached, 2);
    EXPECT_LE(result.final_cost, result.initial_cost);
    ASSERT_EQ(result.keyframe_poses.size(), scene.T_cw.size());
    // Stereo fixes the scale: the metric poses and points are recovered.
    for (size_t k = 0; k < scene.T_cw.size(); ++k) {
        EXPECT_LT(RotationAngle(result.keyframe_poses[k].rotation(), scene.T_cw[k].rotation()),
                  1e-6) << "keyframe " << k;
        EXPECT_LT((result.keyframe_poses[k].translation() - scene.T_cw[k].translation()).norm(),
                  1e-6) << "keyframe " << k;
    }
    ASSERT_EQ(result.points.cols(), scene.points.cols());
    for (Eigen::Index i = 0; i < scene.points.cols(); ++i) {
        EXPECT_LT((result.points.col(i) - scene.points.col(i)).norm(), 1e-6) << "point " << i;
    }
    ASSERT_EQ(result.inlier.size(), scene.problem.observations.size());
    EXPECT_EQ(std::count(result.inlier.begin(), result.inlier.end(), false), 0);
}

// A right x off by 5 px (chi2 25 > 7.815) is an outlier only through the stereo threshold.
TEST(BundleAdjustTest, StereoOutlierUsesStereoThreshold) {
    StereoScene scene = MakeStereoScene(100, 32);
    scene.problem.observations[17].right_x = *scene.problem.observations[17].right_x + 5.0;

    const BaResult result = BundleAdjust(scene.problem, BundleAdjustmentOptions{});

    ASSERT_TRUE(result.success);
    EXPECT_GT(result.chi2[17], 7.815);
    EXPECT_FALSE(result.inlier[17]);
    EXPECT_EQ(std::count(result.inlier.begin(), result.inlier.end(), false), 1);
}

TEST(BundleAdjustTest, StereoInvalidInputFails) {
    const StereoScene scene = MakeStereoScene(20, 33);
    ASSERT_TRUE(BundleAdjust(scene.problem, BundleAdjustmentOptions{}).success);

    BaProblem wrong_size = scene.problem;
    wrong_size.keyframe_stereo.pop_back();
    EXPECT_FALSE(BundleAdjust(wrong_size, BundleAdjustmentOptions{}).success);

    BaProblem right_without_rig = scene.problem;
    right_without_rig.keyframe_stereo[1] = std::nullopt;
    EXPECT_FALSE(BundleAdjust(right_without_rig, BundleAdjustmentOptions{}).success);

    BaProblem right_without_any_rig = scene.problem;
    right_without_any_rig.keyframe_stereo.clear();
    EXPECT_FALSE(BundleAdjust(right_without_any_rig, BundleAdjustmentOptions{}).success);

    BaProblem nan_right = scene.problem;
    nan_right.observations[4].right_x = std::numeric_limits<double>::quiet_NaN();
    EXPECT_FALSE(BundleAdjust(nan_right, BundleAdjustmentOptions{}).success);

    BaProblem nan_rig = scene.problem;
    nan_rig.keyframe_stereo[2]->T_rl =
        RigidTransformation(TMatrix33F::Identity(),
                            TVector3DF(std::numeric_limits<double>::quiet_NaN(), 0.0, 0.0));
    EXPECT_FALSE(BundleAdjust(nan_rig, BundleAdjustmentOptions{}).success);

    // A rig without right observations is fine (the keyframe is used as mono).
    BaProblem mono_with_rig = scene.problem;
    for (BaObservation& observation : mono_with_rig.observations) {
        observation.right_x = std::nullopt;
    }
    EXPECT_TRUE(BundleAdjust(mono_with_rig, BundleAdjustmentOptions{}).success);
}

TEST(BundleAdjustTest, PreSetStopInterruptsAtStageZero) {
    const StereoScene scene = MakeStereoScene(50, 34);
    BundleAdjustmentOptions options;
    int calls = 0;
    options.should_stop = [&calls] {
        ++calls;
        return true;
    };

    const BaResult result = BundleAdjust(scene.problem, options);

    ASSERT_TRUE(result.success);
    EXPECT_TRUE(result.interrupted);
    EXPECT_EQ(result.stage_reached, 0);
    EXPECT_EQ(result.iterations, 0);
    EXPECT_EQ(calls, 1);
    ASSERT_EQ(result.keyframe_poses.size(), scene.problem.keyframe_poses.size());
    for (size_t k = 0; k < result.keyframe_poses.size(); ++k) {
        EXPECT_LT((result.keyframe_poses[k].rotation() - scene.problem.keyframe_poses[k].rotation())
                      .cwiseAbs().maxCoeff(), 1e-12) << "keyframe " << k;
        EXPECT_LT((result.keyframe_poses[k].translation() -
                   scene.problem.keyframe_poses[k].translation()).cwiseAbs().maxCoeff(), 1e-12)
            << "keyframe " << k;
    }
    EXPECT_EQ(result.points, scene.problem.points);
    EXPECT_EQ(result.chi2.size(), scene.problem.observations.size());
    EXPECT_EQ(result.inlier.size(), scene.problem.observations.size());
}

// should_stop is evaluated once before the first iteration (call 1) and after every LM
// iteration (calls 2, 3, ...): true from the 3rd call stops after the 2nd iteration.
TEST(BundleAdjustTest, StopMidRunStopsWithinOneIteration) {
    const StereoScene scene = MakeStereoScene(50, 35);
    BundleAdjustmentOptions options;
    int calls = 0;
    options.should_stop = [&calls] { return ++calls >= 3; };

    const BaResult result = BundleAdjust(scene.problem, options);

    ASSERT_TRUE(result.success);
    EXPECT_TRUE(result.interrupted);
    EXPECT_EQ(result.stage_reached, 0);
    EXPECT_LE(result.iterations, 3);
    EXPECT_EQ(result.iterations, 2);
    EXPECT_EQ(calls, 3);
    EXPECT_LT(result.final_cost, result.initial_cost);
}

TEST(BundleAdjustTest, StopInPlainStageReportsStageOne) {
    const StereoScene scene = MakeStereoScene(50, 36);
    BundleAdjustmentOptions options;
    int calls = 0;
    // Call 1 before the start, calls 2-6 after the 5 robust iterations, call 8 after the
    // 2nd plain iteration.
    options.should_stop = [&calls] { return ++calls >= 8; };

    const BaResult result = BundleAdjust(scene.problem, options);

    ASSERT_TRUE(result.success);
    EXPECT_TRUE(result.interrupted);
    EXPECT_EQ(result.stage_reached, 1);
    EXPECT_EQ(result.iterations, options.robust_iterations + 2);
}

TEST(BundleAdjustTest, NeverStoppingCallbackChangesNothing) {
    const test::TwoViewScene scene = test::MakeGeneralScene(
        300, 0.10, test::NormalizedSigma(0.5), 2, test::DefaultTransform());
    const BaProblem problem = MakePerturbedProblem(scene, 12);
    BundleAdjustmentOptions with_callback;
    with_callback.should_stop = [] { return false; };

    const BaResult a = BundleAdjust(problem, BundleAdjustmentOptions{});
    const BaResult b = BundleAdjust(problem, with_callback);

    ASSERT_TRUE(a.success);
    ASSERT_TRUE(b.success);
    EXPECT_FALSE(b.interrupted);
    EXPECT_EQ(a.stage_reached, 2);
    EXPECT_EQ(b.stage_reached, 2);
    EXPECT_EQ(a.iterations, b.iterations);
    EXPECT_EQ(a.points, b.points);
    EXPECT_EQ(a.chi2, b.chi2);
    EXPECT_EQ(a.inlier, b.inlier);
    for (size_t k = 0; k < a.keyframe_poses.size(); ++k) {
        EXPECT_EQ(a.keyframe_poses[k].rotation(), b.keyframe_poses[k].rotation());
        EXPECT_EQ(a.keyframe_poses[k].translation(), b.keyframe_poses[k].translation());
    }
}

}  // namespace
}  // namespace voyahslam::optimization
