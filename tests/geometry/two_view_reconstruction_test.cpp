#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <vector>

#include <voyahslam/geometry/two_view_reconstruction.h>

#include "support/two_view_scene.h"

namespace voyahslam::geometry {
namespace {

constexpr double kRadToDeg = 180.0 / M_PI;

double RotationErrorDeg(const TMatrix33F& a_R, const TMatrix33F& a_R_gt) {
    return Eigen::AngleAxis<double>(a_R * a_R_gt.transpose()).angle() * kRadToDeg;
}

double DirectionErrorDeg(const TVector3DF& a_t, const TVector3DF& a_t_gt) {
    const double cos_angle = std::clamp(a_t.normalized().dot(a_t_gt.normalized()), -1.0, 1.0);
    return std::acos(cos_angle) * kRadToDeg;
}

size_t CountTrueInliers(const test::TwoViewScene& a_scene) {
    return static_cast<size_t>((!a_scene.is_outlier).count());
}

void ExpectRecovered(const test::TwoViewScene& a_scene, const TwoViewReconstruction& a_result) {
    EXPECT_LT(RotationErrorDeg(a_result.T_21.rotation(), a_scene.T_21.rotation()), 1.0);
    EXPECT_LT(DirectionErrorDeg(a_result.T_21.translation(), a_scene.T_21.translation()), 2.0);
    EXPECT_NEAR(a_result.T_21.translation().norm(), 1.0, 1e-9);
    // Triangulated points are a subset of the model's inliers, and RANSAC does not flag every
    // true inlier at sigma = noise sigma.
    EXPECT_GE(static_cast<double>(a_result.num_triangulated), 0.5 * static_cast<double>(CountTrueInliers(a_scene)));
    EXPECT_EQ(a_result.num_triangulated, static_cast<size_t>(a_result.triangulated.count()));
    EXPECT_EQ(a_result.points.cols(), a_scene.correspondences.size());
    EXPECT_GE(a_result.parallax_deg, TwoViewOptions{}.min_parallax_deg);
}

TEST(TwoViewReconstructionTest, GeneralSceneSelectsEssential) {
    // Wide field of view: a narrow one leaves rotation and translation ambiguous.
    const auto scene =
        test::MakeGeneralScene(300, 0.2, test::NormalizedSigma(1.0), 21, test::DefaultTransform(), 1.0);
    const TwoViewReconstruction result = TwoViewReconstructor().Reconstruct(scene.correspondences);
    ASSERT_TRUE(result.success);
    EXPECT_EQ(result.model, TwoViewModel::Essential);
    ExpectRecovered(scene, result);
}

TEST(TwoViewReconstructionTest, PlanarSceneSelectsHomography) {
    // Wide field of view, strong tilt and a 1 m baseline break the two-fold planar ambiguity.
    const RigidTransformation T_21(test::DefaultTransform().rotation(),
                                   test::DefaultTransform().translation().normalized());
    const auto scene = test::MakePlanarScene(300, 0.2, test::NormalizedSigma(1.0), 22, T_21, 30.0, 1.0);
    const TwoViewReconstruction result = TwoViewReconstructor().Reconstruct(scene.correspondences);
    ASSERT_TRUE(result.success);
    EXPECT_EQ(result.model, TwoViewModel::Homography);
    ExpectRecovered(scene, result);
}

TEST(TwoViewReconstructionTest, NarrowFrontalPlanarSceneRejected) {
    // The two homography decompositions explain the points almost equally well.
    const auto scene = test::MakePlanarScene(300, 0.2, test::NormalizedSigma(1.0), 30, test::DefaultTransform());
    EXPECT_FALSE(TwoViewReconstructor().Reconstruct(scene.correspondences).success);
}

TEST(TwoViewReconstructionTest, PureRotationRejected) {
    const RigidTransformation rotation_only(test::DefaultTransform().rotation(), TVector3DF::Zero());
    const auto scene = test::MakeGeneralScene(300, 0.2, test::NormalizedSigma(1.0), 23, rotation_only);
    EXPECT_FALSE(TwoViewReconstructor().Reconstruct(scene.correspondences).success);
}

TEST(TwoViewReconstructionTest, TinyBaselineRejected) {
    const RigidTransformation T_21(test::DefaultTransform().rotation(),
                                   test::DefaultTransform().translation().normalized() * 0.01);
    const auto scene = test::MakeGeneralScene(300, 0.2, test::NormalizedSigma(1.0), 24, T_21);
    EXPECT_FALSE(TwoViewReconstructor().Reconstruct(scene.correspondences).success);
}

TEST(TwoViewReconstructionTest, TooFewCorrespondencesRejected) {
    const auto scene = test::MakeGeneralScene(30, 0.0, test::NormalizedSigma(1.0), 25, test::DefaultTransform());
    EXPECT_FALSE(TwoViewReconstructor().Reconstruct(scene.correspondences).success);
}

TEST(TwoViewReconstructionTest, Deterministic) {
    const auto scene = test::MakeGeneralScene(300, 0.2, test::NormalizedSigma(1.0), 26, test::DefaultTransform());
    const TwoViewReconstructor reconstructor;
    const TwoViewReconstruction first = reconstructor.Reconstruct(scene.correspondences);
    const TwoViewReconstruction second = reconstructor.Reconstruct(scene.correspondences);
    ASSERT_TRUE(first.success);
    ASSERT_TRUE(second.success);
    EXPECT_EQ(first.model, second.model);
    EXPECT_EQ(first.T_21.ToMatrix34(), second.T_21.ToMatrix34());
    EXPECT_EQ(first.num_triangulated, second.num_triangulated);
}

TEST(TwoViewReconstructionTest, NonPositiveSigmaFails) {
    auto scene = test::MakeGeneralScene(300, 0.0, test::NormalizedSigma(1.0), 27, test::DefaultTransform());
    scene.correspondences.sigma = 0.0;
    EXPECT_FALSE(TwoViewReconstructor().Reconstruct(scene.correspondences).success);
    scene.correspondences.sigma = -1.0;
    EXPECT_FALSE(TwoViewReconstructor().Reconstruct(scene.correspondences).success);
}

TEST(TwoViewReconstructionTest, IdenticalCorrespondencesAreNotTriangulated) {
    // Noise-free scene, every point an inlier, scored with sigma = 1 px; the true pose is the only
    // candidate so the selection result is directly comparable.
    auto scene = test::MakeGeneralScene(300, 0.0, 0.0, 28, test::DefaultTransform());
    scene.correspondences.sigma = test::NormalizedSigma(1.0);
    const RigidTransformation T_21(scene.T_21.rotation(), scene.T_21.translation().normalized());
    const std::array<RigidTransformation, 1> candidates{T_21};
    const Eigen::Array<bool, Eigen::Dynamic, 1> inliers =
        Eigen::Array<bool, Eigen::Dynamic, 1>::Constant(scene.correspondences.size(), true);
    const TwoViewOptions options;

    const PoseSelection reference = SelectPose(candidates, scene.correspondences, inliers, 0.7, options);
    ASSERT_EQ(reference.num_good, static_cast<size_t>(scene.correspondences.size()));

    constexpr Eigen::Index kNumIdentical = 20;
    for (Eigen::Index i = 0; i < kNumIdentical; ++i) {
        scene.correspondences.x2.col(i) = scene.correspondences.x1.col(i);
    }
    const PoseSelection selection = SelectPose(candidates, scene.correspondences, inliers, 0.7, options);
    for (Eigen::Index i = 0; i < kNumIdentical; ++i) {
        EXPECT_FALSE(selection.triangulated(i)) << "index " << i;
    }
    EXPECT_EQ(selection.num_good, reference.num_good - static_cast<size_t>(kNumIdentical));
    EXPECT_TRUE(selection.success);
}

TEST(SelectPoseTest, AmbiguousCandidatesRejected) {
    const auto scene = test::MakeGeneralScene(300, 0.0, 0.0, 29, test::DefaultTransform());
    auto correspondences = scene.correspondences;
    correspondences.sigma = test::NormalizedSigma(1.0);
    const RigidTransformation T_21(scene.T_21.rotation(), scene.T_21.translation().normalized());
    const Eigen::Array<bool, Eigen::Dynamic, 1> inliers =
        Eigen::Array<bool, Eigen::Dynamic, 1>::Constant(correspondences.size(), true);
    const TwoViewOptions options;

    const std::array<RigidTransformation, 1> unique{T_21};
    EXPECT_TRUE(SelectPose(unique, correspondences, inliers, options.essential_uniqueness, options).success);

    const std::array<RigidTransformation, 2> duplicated{T_21, T_21};
    const PoseSelection selection =
        SelectPose(duplicated, correspondences, inliers, options.essential_uniqueness, options);
    EXPECT_FALSE(selection.success);
}

} // namespace
} // namespace voyahslam::geometry
