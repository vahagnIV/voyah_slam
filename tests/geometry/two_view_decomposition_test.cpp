#include <gtest/gtest.h>

#include <cmath>

#include <voyahslam/geometry/two_view_decomposition.h>

#include "support/two_view_scene.h"

namespace voyahslam::geometry {
namespace {

constexpr double kTolerance = 1e-9;

TMatrix33F Skew(const TVector3DF& a_v) {
    TMatrix33F m;
    m << 0.0, -a_v.z(), a_v.y(), a_v.z(), 0.0, -a_v.x(), -a_v.y(), a_v.x(), 0.0;
    return m;
}

double RotationError(const TMatrix33F& a_R, const TMatrix33F& a_R_gt) {
    return (a_R - a_R_gt).norm();
}

// Distance between unit directions; the sign matters.
double DirectionError(const TVector3DF& a_t, const TVector3DF& a_t_gt) {
    return (a_t.normalized() - a_t_gt.normalized()).norm();
}

// Ground-truth plane of test::MakePlanarScene: normal (sin t, 0, cos t) through (0, 0, 5).
struct Plane {
    TVector3DF normal;
    double offset;
};

Plane ScenePlane(double a_tilt_deg) {
    const double tilt = a_tilt_deg * M_PI / 180.0;
    const TVector3DF normal(std::sin(tilt), 0.0, std::cos(tilt));
    return {normal, normal.dot(TVector3DF(0.0, 0.0, 5.0))};
}

TMatrix33F GroundTruthHomography(const RigidTransformation& a_T_21, const Plane& a_plane) {
    return a_T_21.rotation() + a_T_21.translation() * a_plane.normal.transpose() / a_plane.offset;
}

TEST(TwoViewDecompositionTest, EssentialContainsTruePose) {
    const RigidTransformation T_21 = test::DefaultTransform();
    const TMatrix33F E = Skew(T_21.translation()) * T_21.rotation();
    const auto candidates = DecomposeEssential(E);
    int matches = 0;
    for (const auto& candidate : candidates) {
        if (RotationError(candidate.rotation(), T_21.rotation()) < kTolerance &&
            DirectionError(candidate.translation(), T_21.translation()) < kTolerance) {
            ++matches;
        }
    }
    EXPECT_EQ(matches, 1);
}

TEST(TwoViewDecompositionTest, EssentialCandidatesAreProperRotations) {
    const RigidTransformation T_21 = test::DefaultTransform();
    // Scale and sign of E are arbitrary.
    const TMatrix33F E = -3.0 * Skew(T_21.translation()) * T_21.rotation();
    for (const auto& candidate : DecomposeEssential(E)) {
        const TMatrix33F& R = candidate.rotation();
        EXPECT_NEAR(R.determinant(), 1.0, kTolerance);
        EXPECT_TRUE((R * R.transpose()).isApprox(TMatrix33F::Identity(), kTolerance));
        EXPECT_NEAR(candidate.translation().norm(), 1.0, kTolerance);
    }
}

TEST(TwoViewDecompositionTest, HomographyContainsTruePose) {
    const RigidTransformation T_21 = test::DefaultTransform();
    const Plane plane = ScenePlane(10.0);
    const auto scene = test::MakePlanarScene(20, 0.0, 0.0, 3, T_21, 10.0);
    const TMatrix33F H = GroundTruthHomography(T_21, plane);
    // The scene's own correspondences obey H (sanity check of the ground truth).
    for (Eigen::Index i = 0; i < scene.correspondences.size(); ++i) {
        const TVector3DF x1(scene.correspondences.x1(0, i), scene.correspondences.x1(1, i), 1.0);
        const TVector3DF p = H * x1;
        ASSERT_NEAR((p.head<2>() / p.z() - scene.correspondences.x2.col(i)).norm(), 0.0, 1e-12);
    }

    const auto candidates = DecomposeHomography(H);
    ASSERT_EQ(candidates.size(), 8u);
    int matches = 0;
    for (const auto& candidate : candidates) {
        if (RotationError(candidate.T_21.rotation(), T_21.rotation()) < kTolerance &&
            DirectionError(candidate.T_21.translation(), T_21.translation()) < kTolerance) {
            ++matches;
            EXPECT_LT(DirectionError(candidate.plane_normal, plane.normal), kTolerance);
        }
    }
    EXPECT_EQ(matches, 1);
}

TEST(TwoViewDecompositionTest, HomographyCandidatesAreProperRotations) {
    const RigidTransformation T_21 = test::DefaultTransform();
    // Scale and sign of H are arbitrary.
    const TMatrix33F H = -2.5 * GroundTruthHomography(T_21, ScenePlane(10.0));
    const auto candidates = DecomposeHomography(H);
    ASSERT_EQ(candidates.size(), 8u);
    for (const auto& candidate : candidates) {
        const TMatrix33F& R = candidate.T_21.rotation();
        EXPECT_NEAR(R.determinant(), 1.0, kTolerance);
        EXPECT_TRUE((R * R.transpose()).isApprox(TMatrix33F::Identity(), kTolerance));
        EXPECT_NEAR(candidate.T_21.translation().norm(), 1.0, kTolerance);
        EXPECT_NEAR(candidate.plane_normal.norm(), 1.0, kTolerance);
    }
}

TEST(TwoViewDecompositionTest, PureRotationHomographyGivesNoCandidates) {
    const RigidTransformation T_21 = test::DefaultTransform();
    EXPECT_TRUE(DecomposeHomography(T_21.rotation()).empty());
    EXPECT_TRUE(DecomposeHomography(TMatrix33F(2.0 * T_21.rotation())).empty());
}

} // namespace
} // namespace voyahslam::geometry
