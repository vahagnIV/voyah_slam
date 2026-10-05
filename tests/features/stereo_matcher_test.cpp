#include <gtest/gtest.h>
#include <algorithm>
#include <cmath>
#include <iostream>
#include <numbers>
#include <random>
#include <vector>
#include "support/synthetic_sequence.h"
#include "support/test_measurements.h"
#include "voyahslam/features/stereo_matcher.h"
#include "voyahslam/geometry/camera.h"
#include "voyahslam/geometry/distortion_models/kannala_brandt_distortion_model.h"

namespace voyahslam::features::test {

namespace {

using voyahslam::test::MakeSyntheticStereoFrame;
using voyahslam::test::SyntheticStereoIntrinsics;
using voyahslam::test::SyntheticStereoRig;
using voyahslam::test::TestExtractor;
using voyahslam::test::TestImageFeatures;
using Measurement = TBinocularMeasurement<TestExtractor>;
using Descriptor = TestImageFeatures::TDescriptor;

struct Spec {
    geometry::TPoint2DF pixel = geometry::TPoint2DF(512.0, 512.0);
    int bits = 0;   // descriptor = `bits` low bits set; distance between two = |difference|
    int seed = -1;  // >= 0: random descriptor instead
    int level = 0;
};

Descriptor MakeDescriptor(const Spec& spec) {
    Descriptor d = Descriptor::Zero();
    if (spec.seed >= 0) {
        std::mt19937 rng(static_cast<uint32_t>(spec.seed));
        for (Eigen::Index i = 0; i < d.size(); ++i) {
            d[i] = static_cast<uint8_t>(rng() & 0xFFU);
        }
        return d;
    }
    for (int b = 0; b < spec.bits; ++b) {
        d[b / 8] |= static_cast<uint8_t>(1U << (b % 8));
    }
    return d;
}

TestImageFeatures MakeFeatures(const std::vector<Spec>& specs) {
    const auto n = static_cast<Eigen::Index>(specs.size());
    geometry::TPoints2DF points(2, n);
    TestImageFeatures::TDescriptors descriptors(TestExtractor::DescriptorSize, n);
    TLevels levels(n);
    for (Eigen::Index i = 0; i < n; ++i) {
        points.col(i) = specs[i].pixel;
        descriptors.col(i) = MakeDescriptor(specs[i]);
        levels[i] = static_cast<uint8_t>(specs[i].level);
    }
    return TestImageFeatures(std::move(points), geometry::TRowVectorF::Ones(n),
                             geometry::TRowVectorF::Zero(n), std::move(descriptors),
                             std::move(levels));
}

Measurement Make(const std::vector<Spec>& left, const std::vector<Spec>& right,
                 const StereoRig& rig = SyntheticStereoRig()) {
    return Measurement(MakeFeatures(left), MakeFeatures(right), rig, 0.0);
}

geometry::TPoint2DF ProjectLeft(const geometry::TPoint3DF& X, const StereoRig& rig = SyntheticStereoRig()) {
    return (rig.K_left * X).hnormalized();
}

geometry::TPoint2DF ProjectRight(const geometry::TPoint3DF& X, const StereoRig& rig = SyntheticStereoRig()) {
    return (rig.K_right * rig.T_rl.Transform(X)).hnormalized();
}

const geometry::TPoint3DF kPoint(0.3, -0.2, 4.0);

// TUM-VI cam0 (equidistant fisheye, 1024x1024).
geometry::TMatrix33F TumViIntrinsics() {
    geometry::TMatrix33F K = geometry::TMatrix33F::Identity();
    K(0, 0) = 380.81042871360756;
    K(1, 1) = 380.81194179427075;
    K(0, 2) = 510.29465304840727;
    K(1, 2) = 514.3304630538506;
    return K;
}

geometry::distortion_models::KannalaBrandtDistortionModel TumViModel() {
    return geometry::distortion_models::KannalaBrandtDistortionModel(
        0.010171079892421483, -0.010816440029919381, 0.005942781769412756, -0.001662284667857643);
}

}  // namespace

TEST(StereoMatcherTest, RightCameraAtPlusXGivesPositiveDisparity) {
    // Independent of RigidTransformation semantics: a right camera at +x of the
    // left sees a point at depth z shifted by -fx*b/z, same row.
    const StereoRig rig = SyntheticStereoRig();
    const double fx = SyntheticStereoIntrinsics()(0, 0);
    const double z = 3.0;
    const geometry::TPoint2DF left_pixel(600.0, 450.0);
    const geometry::TPoint2DF right_pixel(left_pixel.x() - fx * rig.baseline / z, left_pixel.y());
    const Measurement m = Make({{.pixel = left_pixel, .bits = 0}}, {{.pixel = right_pixel, .bits = 0}});
    const StereoMatches matches = MatchStereo<TestExtractor>(m, {});
    ASSERT_EQ(matches.num_matches, 1U);
    EXPECT_EQ(matches.right_indices[0], 0);
    EXPECT_NEAR(matches.depths[0], z, 1e-9);
}

TEST(StereoMatcherTest, RatioTestRejectsAmbiguous) {
    // Both right candidates sit at the geometrically correct pixel, so only the
    // ratio test decides.
    const geometry::TPoint2DF left = ProjectLeft(kPoint);
    const geometry::TPoint2DF right = ProjectRight(kPoint);
    {
        const Measurement m = Make({{.pixel = left, .bits = 0}},
                                   {{.pixel = right, .bits = 10}, {.pixel = right, .bits = 12}});
        const StereoMatches matches = MatchStereo<TestExtractor>(m, {});
        EXPECT_EQ(matches.num_matches, 0U);
        EXPECT_EQ(matches.right_indices[0], -1);
        EXPECT_EQ(matches.depths[0], -1.0);
    }
    {
        const Measurement m = Make({{.pixel = left, .bits = 0}},
                                   {{.pixel = right, .bits = 10}, {.pixel = right, .bits = 20}});
        const StereoMatches matches = MatchStereo<TestExtractor>(m, {});
        EXPECT_EQ(matches.num_matches, 1U);
        EXPECT_EQ(matches.right_indices[0], 0);
        EXPECT_NEAR(matches.depths[0], kPoint.z(), 1e-6);
    }
}

TEST(StereoMatcherTest, OneToOneKeepsCloserClaim) {
    // Left 0 and 1 both observe kPoint; right 0 is their best match (distances 9
    // and 5), right 1 is far in descriptor space (distances 89 and 75).
    const geometry::TPoint2DF left = ProjectLeft(kPoint);
    const geometry::TPoint3DF other(-0.5, 0.4, 3.0);
    const Measurement m = Make({{.pixel = left, .bits = 11}, {.pixel = left, .bits = 25}},
                               {{.pixel = ProjectRight(kPoint), .bits = 20},
                                {.pixel = ProjectRight(other), .bits = 100}});
    const StereoMatches matches = MatchStereo<TestExtractor>(m, {});
    EXPECT_EQ(matches.num_matches, 1U);
    EXPECT_EQ(matches.right_indices[0], -1);
    EXPECT_EQ(matches.right_indices[1], 0);
    EXPECT_EQ(matches.depths[0], -1.0);
    EXPECT_NEAR(matches.depths[1], kPoint.z(), 1e-6);
}

TEST(StereoMatcherTest, RejectsPointBehindRightCamera) {
    // Right camera turned 180 degrees about y, centre at (-b, 0, 0): kPoint is in
    // front of the left camera and behind the right one. The projections are
    // exact, so only the right-camera depth test can reject the pair.
    StereoRig rig = SyntheticStereoRig();
    const geometry::TMatrix33F R =
        Eigen::AngleAxis<double>(std::numbers::pi, geometry::TVector3DF::UnitY()).toRotationMatrix();
    rig.T_rl = geometry::RigidTransformation(R, geometry::TVector3DF(-rig.baseline, 0.0, 0.0));
    ASSERT_LT(rig.T_rl.Transform(kPoint).z(), 0.0);
    const Measurement m = Make({{.pixel = ProjectLeft(kPoint, rig), .bits = 0}},
                               {{.pixel = ProjectRight(kPoint, rig), .bits = 0}}, rig);
    const StereoMatches matches = MatchStereo<TestExtractor>(m, {});
    EXPECT_EQ(matches.num_matches, 0U);
    EXPECT_EQ(matches.right_indices[0], -1);
}

TEST(StereoMatcherTest, ToeInRigExactDepth) {
    // Right camera yawed ~5 degrees about y: exact projections of a few points
    // must triangulate to the true left-camera depth. Catches a wrong right
    // centre (-t instead of -R^T t) or wrong rotation use.
    StereoRig rig = SyntheticStereoRig();
    const geometry::TMatrix33F R =
        Eigen::AngleAxis<double>(5.0 * std::numbers::pi / 180.0, geometry::TVector3DF::UnitY())
            .toRotationMatrix();
    rig.T_rl = geometry::RigidTransformation(R, geometry::TVector3DF(-rig.baseline, 0.0, 0.0));
    const std::vector<geometry::TPoint3DF> points = {
        {0.3, -0.2, 4.0}, {-0.5, 0.4, 3.0}, {1.0, 0.5, 6.0}, {0.0, 0.0, 2.5}};
    std::vector<Spec> left;
    std::vector<Spec> right;
    for (size_t i = 0; i < points.size(); ++i) {
        ASSERT_GT(rig.T_rl.Transform(points[i]).z(), 0.0);
        left.push_back({.pixel = ProjectLeft(points[i], rig), .seed = static_cast<int>(i)});
        right.push_back({.pixel = ProjectRight(points[i], rig), .seed = static_cast<int>(i)});
    }
    const StereoMatches matches = MatchStereo<TestExtractor>(Make(left, right, rig), {});
    ASSERT_EQ(matches.num_matches, points.size());
    for (size_t i = 0; i < points.size(); ++i) {
        EXPECT_EQ(matches.right_indices[i], static_cast<int>(i));
        EXPECT_NEAR(matches.depths[i], points[i].z(), 1e-6);
    }

    // The right centre only enters the parallax test: put the threshold between
    // the cosines from the true centre -R^T t and the wrong one, -t.
    const geometry::TPoint3DF X = points[3];
    auto parallax_cos = [&](const geometry::TVector3DF& centre) {
        const geometry::TVector3DF ray_right = X - centre;
        return X.dot(ray_right) / (X.norm() * ray_right.norm());
    };
    const double true_cos = parallax_cos(rig.T_rl.Inverse().translation());
    const double wrong_cos = parallax_cos(-rig.T_rl.translation());
    ASSERT_GT(std::abs(true_cos - wrong_cos), 1e-9);
    const Measurement single = Make({left[3]}, {right[3]}, rig);
    StereoMatcherOptions options;
    options.max_parallax_cos = 0.5 * (true_cos + wrong_cos);
    EXPECT_EQ(MatchStereo<TestExtractor>(single, options).num_matches, true_cos < wrong_cos ? 1U : 0U);
}

TEST(StereoMatcherTest, RejectsNegativeDisparity) {
    // With the synthetic rig, a right pixel to the right of the left one makes
    // the rays meet behind both cameras.
    const geometry::TPoint2DF left = ProjectLeft(kPoint);
    const geometry::TPoint2DF right = left + geometry::TPoint2DF(10.0, 0.0);
    const Measurement m = Make({{.pixel = left, .bits = 0}}, {{.pixel = right, .bits = 0}});
    const StereoMatches matches = MatchStereo<TestExtractor>(m, {});
    EXPECT_EQ(matches.num_matches, 0U);
}

TEST(StereoMatcherTest, RejectsNearParallelRays) {
    const geometry::TPoint3DF far(1.0, 0.5, 2000.0);
    const Measurement m = Make({{.pixel = ProjectLeft(far), .bits = 0}},
                               {{.pixel = ProjectRight(far), .bits = 0}});
    const StereoMatches matches = MatchStereo<TestExtractor>(m, {});
    EXPECT_EQ(matches.num_matches, 0U);
    EXPECT_EQ(matches.right_indices[0], -1);

    // Same point accepted once the parallax test is disabled: the rejection
    // above is the parallax test's.
    StereoMatcherOptions options;
    options.max_parallax_cos = 1.0 + 1e-9;
    EXPECT_EQ(MatchStereo<TestExtractor>(m, options).num_matches, 1U);
}

TEST(StereoMatcherTest, ParallaxUsesRightCameraCentre) {
    // Off-axis point: the true right centre (+b, 0, 0) subtends a larger angle
    // than the mirrored (-b, 0, 0) would. A threshold between the two cosines
    // accepts the pair only if the centre is T_rl.Inverse().translation().
    const StereoRig rig = SyntheticStereoRig();
    const geometry::TPoint3DF X(5.0, 0.0, 5.0);
    auto parallax_cos = [&](const geometry::TVector3DF& centre) {
        const geometry::TVector3DF ray_right = X - centre;
        return X.dot(ray_right) / (X.norm() * ray_right.norm());
    };
    const double true_cos = parallax_cos(geometry::TVector3DF(rig.baseline, 0.0, 0.0));
    const double mirrored_cos = parallax_cos(geometry::TVector3DF(-rig.baseline, 0.0, 0.0));
    ASSERT_LT(true_cos, mirrored_cos);

    const Measurement m = Make({{.pixel = ProjectLeft(X), .bits = 0}}, {{.pixel = ProjectRight(X), .bits = 0}});
    StereoMatcherOptions options;
    options.max_parallax_cos = 0.5 * (true_cos + mirrored_cos);
    EXPECT_EQ(MatchStereo<TestExtractor>(m, options).num_matches, 1U);
    options.max_parallax_cos = true_cos - 0.1 * (mirrored_cos - true_cos);
    EXPECT_EQ(MatchStereo<TestExtractor>(m, options).num_matches, 0U);
}

TEST(StereoMatcherTest, RejectsGeometricallyInconsistentPair) {
    const geometry::TPoint2DF left = ProjectLeft(kPoint);
    const geometry::TPoint2DF right = ProjectRight(kPoint) + geometry::TPoint2DF(0.0, 20.0);
    const Measurement m = Make({{.pixel = left, .bits = 0}}, {{.pixel = right, .bits = 0}});
    const StereoMatches matches = MatchStereo<TestExtractor>(m, {});
    EXPECT_EQ(matches.num_matches, 0U);
    EXPECT_EQ(matches.right_indices[0], -1);
}

TEST(StereoMatcherTest, ReprojectionSigmaScalesWithLevel) {
    // A 6 px vertical offset splits into ~3 px residuals per image: chi2 ~ 9 >
    // 5.991 at level 0, ~1 at level 6 (sigma = 1.2^6 ~ 3 px).
    const geometry::TPoint2DF left = ProjectLeft(kPoint);
    const geometry::TPoint2DF right = ProjectRight(kPoint) + geometry::TPoint2DF(0.0, 6.0);
    const Measurement level0 = Make({{.pixel = left, .bits = 0}}, {{.pixel = right, .bits = 0}});
    EXPECT_EQ(MatchStereo<TestExtractor>(level0, {}).num_matches, 0U);
    const Measurement level6 = Make({{.pixel = left, .bits = 0, .level = 6}},
                                    {{.pixel = right, .bits = 0, .level = 6}});
    EXPECT_EQ(MatchStereo<TestExtractor>(level6, {}).num_matches, 1U);
}

TEST(StereoMatcherTest, EmptySideYieldsNoMatches) {
    {
        const Measurement m = Make({{.pixel = ProjectLeft(kPoint)}, {.pixel = {100.0, 100.0}}}, {});
        const StereoMatches matches = MatchStereo<TestExtractor>(m, {});
        ASSERT_EQ(matches.right_indices.size(), 2);
        ASSERT_EQ(matches.depths.size(), 2);
        EXPECT_TRUE((matches.right_indices.array() == -1).all());
        EXPECT_TRUE((matches.depths.array() == -1.0).all());
        EXPECT_EQ(matches.num_matches, 0U);
    }
    {
        const Measurement m = Make({}, {{.pixel = ProjectRight(kPoint)}});
        const StereoMatches matches = MatchStereo<TestExtractor>(m, {});
        EXPECT_EQ(matches.right_indices.size(), 0);
        EXPECT_EQ(matches.depths.size(), 0);
        EXPECT_EQ(matches.num_matches, 0U);
    }
}

TEST(StereoMatcherTest, SingleRightCandidateCanMatch) {
    const Measurement m = Make({{.pixel = ProjectLeft(kPoint), .bits = 3}},
                               {{.pixel = ProjectRight(kPoint), .bits = 7}});
    const StereoMatches matches = MatchStereo<TestExtractor>(m, {});
    ASSERT_EQ(matches.num_matches, 1U);
    EXPECT_EQ(matches.right_indices[0], 0);
    EXPECT_NEAR(matches.depths[0], kPoint.z(), 1e-6);
}

TEST(StereoMatcherTest, FisheyePeripheralPairsAccepted) {
    // TUM-VI KB cameras on a 10 cm baseline; points 60-80 degrees off the left
    // axis, raw keypoints with 0.5 px noise. In undistorted pixels that noise
    // is magnified several times; in raw pixels it is well inside the gate.
    const auto model = TumViModel();
    const StereoRig rig{
        .K_left = TumViIntrinsics(),
        .K_right = TumViIntrinsics(),
        .T_rl = geometry::RigidTransformation(geometry::TMatrix33F::Identity(), geometry::TVector3DF(-0.1, 0.0, 0.0)),
        .baseline = 0.1,
        .distortion_left = &model,
        .distortion_right = &model,
    };
    const geometry::Camera left_camera{rig.K_left, &model};
    const geometry::Camera right_camera{rig.K_right, &model};
    constexpr double kDeg = std::numbers::pi / 180.0;
    constexpr int kNumPoints = 12;
    // 0.5 px in golden-angle directions: deterministic across standard libraries.
    int num_noise = 0;
    auto noise = [&]() {
        const double a = 2.399963229728653 * ++num_noise;
        return geometry::TPoint2DF(0.5 * std::cos(a), 0.5 * std::sin(a));
    };
    std::vector<Spec> left;
    std::vector<Spec> right;
    std::vector<Spec> exact_left;
    std::vector<Spec> shifted_right;
    for (int i = 0; i < kNumPoints; ++i) {
        const double theta = (60.0 + 20.0 * i / (kNumPoints - 1)) * kDeg;
        const double phi = 2.0 * std::numbers::pi * i / kNumPoints + 0.2;
        const geometry::TPoint3DF X =
            1.5 * geometry::TPoint3DF(std::sin(theta) * std::cos(phi), std::sin(theta) * std::sin(phi),
                                      std::cos(theta));
        const geometry::TPoint3DF X_right = rig.T_rl.Transform(X);
        ASSERT_LT(std::acos(X_right.z() / X_right.norm()), 84.0 * kDeg) << "point " << i;
        left.push_back({.pixel = left_camera.Project(X) + noise(), .seed = i});
        right.push_back({.pixel = right_camera.Project(X_right) + noise(), .seed = i});
        // The exact pair with the right keypoint 5 px across the epipolar curve
        // (the image of the left ray), so no depth along the ray explains it.
        // Noise is left out here: a 5 px inconsistency splits into ~2.5 px per
        // image (chi2 ~ 6.25 against 5.991), which 0.5 px noise could undo.
        const geometry::TPoint2DF along =
            right_camera.ProjectJacobian(X_right) * rig.T_rl.rotation() * X.normalized();
        const geometry::TPoint2DF across = geometry::TPoint2DF(-along.y(), along.x()).normalized();
        exact_left.push_back({.pixel = left_camera.Project(X), .seed = i});
        shifted_right.push_back({.pixel = right_camera.Project(X_right) + 5.0 * across, .seed = i});
    }

    const StereoMatches accepted = MatchStereo<TestExtractor>(Make(left, right, rig), {});
    EXPECT_EQ(accepted.num_matches, static_cast<size_t>(kNumPoints));
    for (int i = 0; i < kNumPoints; ++i) {
        EXPECT_EQ(accepted.right_indices[i], i) << "point " << i;
    }

    const StereoMatches rejected = MatchStereo<TestExtractor>(Make(exact_left, shifted_right, rig), {});
    EXPECT_EQ(rejected.num_matches, 0U);
}

TEST(StereoMatcherTest, SyntheticNoiseFreeExact) {
    const auto frame = MakeSyntheticStereoFrame({.pixel_noise = 0.0});
    const StereoMatches matches = MatchStereo<TestExtractor>(frame.measurement, {});
    const auto n = static_cast<Eigen::Index>(frame.right_of_left.size());
    ASSERT_EQ(matches.right_indices.size(), n);
    ASSERT_EQ(matches.depths.size(), n);
    size_t num_true = 0;
    size_t num_recorded = 0;
    for (Eigen::Index i = 0; i < n; ++i) {
        num_recorded += matches.right_indices[i] >= 0 ? 1U : 0U;
        if (frame.right_of_left[i] < 0) {
            continue;
        }
        ++num_true;
        ASSERT_EQ(matches.right_indices[i], frame.right_of_left[i]) << "left feature " << i;
        EXPECT_LT(std::abs(matches.depths[i] - frame.depth_of_left[i]), 1e-6) << "left feature " << i;
        const geometry::TPoint3DF X = matches.depths[i] * frame.measurement.left().normalized_point(i);
        EXPECT_LT((X - frame.points_left.col(i)).norm(), 1e-6) << "left feature " << i;
    }
    EXPECT_GT(num_true, 400U);
    EXPECT_EQ(matches.num_matches, num_recorded);
}

TEST(StereoMatcherTest, SyntheticPrecisionAndDepth) {
    const auto frame = MakeSyntheticStereoFrame();
    const StereoMatches matches = MatchStereo<TestExtractor>(frame.measurement, {});
    const auto n = static_cast<Eigen::Index>(frame.right_of_left.size());
    size_t correct = 0;
    size_t num_true = 0;
    std::vector<double> relative_errors;
    for (Eigen::Index i = 0; i < n; ++i) {
        num_true += frame.right_of_left[i] >= 0 ? 1U : 0U;
        if (matches.right_indices[i] < 0) {
            continue;
        }
        if (matches.right_indices[i] == frame.right_of_left[i]) {
            ++correct;
            relative_errors.push_back(std::abs(matches.depths[i] - frame.depth_of_left[i]) /
                                      frame.depth_of_left[i]);
        }
    }
    ASSERT_GT(matches.num_matches, 0U);
    ASSERT_FALSE(relative_errors.empty());
    const double precision = static_cast<double>(correct) / static_cast<double>(matches.num_matches);
    const double recall = static_cast<double>(correct) / static_cast<double>(num_true);
    std::nth_element(relative_errors.begin(),
                     relative_errors.begin() + static_cast<std::ptrdiff_t>(relative_errors.size() / 2),
                     relative_errors.end());
    const double median = relative_errors[relative_errors.size() / 2];
    std::cout << "stereo matches " << matches.num_matches << ", precision " << precision << ", recall "
              << recall << ", median relative depth error " << median << '\n';
    EXPECT_GE(precision, 0.98);
    EXPECT_LT(median, 0.10);
}

}  // namespace voyahslam::features::test
