#include <gtest/gtest.h>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <numbers>
#include <random>
#include <tuple>
#include <utility>
#include <vector>
#include "support/test_measurements.h"
#include "voyahslam/geometry/camera.h"
#include "voyahslam/geometry/distortion_models/kannala_brandt_distortion_model.h"
#include "voyahslam/mapping/triangulation.h"

namespace voyahslam::mapping {
namespace {

using voyahslam::test::TestExtractor;
using voyahslam::test::TestImageFeatures;
using Measurement = features::TMonocularMeasurement<TestExtractor>;
using View = TriangulationView<TestExtractor>;
using Pairs = std::vector<std::pair<uint32_t, uint32_t>>;

// TUM-VI cam0 (equidistant fisheye, 1024x1024); duplicated from the stereo
// matcher and camera tests (known deferred fixture duplication).
geometry::TMatrix33F TumViIntrinsics() {
    geometry::TMatrix33F K = geometry::TMatrix33F::Identity();
    K(0, 0) = 380.81042871360756;
    K(1, 1) = 380.81194179427075;
    K(0, 2) = 510.29465304840727;
    K(1, 2) = 514.3304630538506;
    return K;
}

const geometry::distortion_models::KannalaBrandtDistortionModel& TumViModel() {
    static const geometry::distortion_models::KannalaBrandtDistortionModel model(
        0.010171079892421483, -0.010816440029919381, 0.005942781769412756, -0.001662284667857643);
    return model;
}

// Two views of the same world points: feature i of each view observes point i,
// with the same random descriptor, angle 0 and the given level.
struct Scene {
    std::vector<geometry::TPoint3DF> points;
    geometry::RigidTransformation T_aw;
    geometry::RigidTransformation T_bw;
    geometry::TPoints2DF pixels_a;
    geometry::TPoints2DF pixels_b;
    TestImageFeatures::TDescriptors descriptors;
    features::TLevels levels_a;
    features::TLevels levels_b;
};

Scene MakeScene(const std::vector<geometry::TPoint3DF>& points, const geometry::RigidTransformation& T_aw,
                const geometry::RigidTransformation& T_bw, const geometry::Camera& camera) {
    const auto n = static_cast<Eigen::Index>(points.size());
    Scene scene{points, T_aw, T_bw, geometry::TPoints2DF(2, n), geometry::TPoints2DF(2, n),
                TestImageFeatures::TDescriptors(TestExtractor::DescriptorSize, n),
                features::TLevels::Zero(n), features::TLevels::Zero(n)};
    std::mt19937 rng(7);
    for (Eigen::Index i = 0; i < n; ++i) {
        scene.pixels_a.col(i) = camera.Project(T_aw.Transform(points[i]));
        scene.pixels_b.col(i) = camera.Project(T_bw.Transform(points[i]));
        for (Eigen::Index k = 0; k < scene.descriptors.rows(); ++k) {
            scene.descriptors(k, i) = static_cast<uint8_t>(rng() & 0xFFU);
        }
    }
    return scene;
}

Measurement MakeMeasurement(const geometry::TPoints2DF& pixels, const TestImageFeatures::TDescriptors& descriptors,
                            const features::TLevels& levels, const geometry::Camera& camera) {
    const Eigen::Index n = pixels.cols();
    TestImageFeatures features(geometry::TPoints2DF(pixels), geometry::TRowVectorF::Ones(n),
                               geometry::TRowVectorF::Zero(n), TestImageFeatures::TDescriptors(descriptors),
                               features::TLevels(levels));
    Measurement measurement(std::move(features), camera.K, camera.distortion, 0.0);
    EXPECT_EQ(measurement.points().cols(), n) << "a feature was dropped by the 85 degree cut";
    return measurement;
}

View MakeView(const Measurement& measurement, const geometry::RigidTransformation& T_cw) {
    const auto n = static_cast<size_t>(measurement.points().cols());
    return View{&measurement, T_cw, std::vector<bool>(n, false), {}, 0.0};
}

geometry::Camera PinholeCamera() { return geometry::Camera{voyahslam::test::TestIntrinsics(), nullptr}; }

// Points in front of a camera at the origin looking down +z, within the
// central +-100 px of the pinhole test intrinsics, depths 3..8 m.
std::vector<geometry::TPoint3DF> PinholePoints(size_t count, double min_depth = 3.0, double max_depth = 8.0) {
    std::mt19937 rng(42);
    std::uniform_real_distribution<double> pixel(-100.0, 100.0);
    std::uniform_real_distribution<double> depth(min_depth, max_depth);
    std::vector<geometry::TPoint3DF> points;
    for (size_t i = 0; i < count; ++i) {
        const double z = depth(rng);
        points.emplace_back(pixel(rng) / 500.0 * z, pixel(rng) / 500.0 * z, z);
    }
    return points;
}

geometry::RigidTransformation Translation(double x, double y, double z) {
    // T_cw of a camera centred at (x, y, z) with the world's orientation.
    return geometry::RigidTransformation(geometry::TMatrix33F::Identity(), geometry::TVector3DF(-x, -y, -z));
}

std::vector<TriangulatedPoint> MatchAndTriangulate(const View& a, const View& b,
                                                   const TriangulationOptions& options = {}) {
    return TriangulateMatches(a, b, SearchForTriangulation(a, b, options), options);
}

TEST(TriangulationTest, PinholeRecoversPoints) {
    const geometry::Camera camera = PinholeCamera();
    const Scene scene = MakeScene(PinholePoints(200), Translation(0, 0, 0), Translation(0.3, 0, 0), camera);
    const Measurement ma = MakeMeasurement(scene.pixels_a, scene.descriptors, scene.levels_a, camera);
    const Measurement mb = MakeMeasurement(scene.pixels_b, scene.descriptors, scene.levels_b, camera);
    const View a = MakeView(ma, scene.T_aw);
    const View b = MakeView(mb, scene.T_bw);

    const Pairs matches = SearchForTriangulation(a, b, TriangulationOptions{});
    for (const auto& [ia, ib] : matches) {
        EXPECT_EQ(ia, ib) << "wrong pair";
    }
    const std::vector<TriangulatedPoint> points = TriangulateMatches(a, b, matches, TriangulationOptions{});
    EXPECT_GE(points.size(), 190U);
    for (const TriangulatedPoint& p : points) {
        ASSERT_EQ(p.feature_a, p.feature_b);
        EXPECT_LT((p.position - scene.points[p.feature_a]).norm(), 1e-6);
    }
}

TEST(TriangulationTest, FisheyePeripheralPointsRecovered) {
    const geometry::Camera camera{TumViIntrinsics(), &TumViModel()};
    // Rays 60..80 degrees off a's axis, at 1..2 m; b sits 0.5 m behind a so the
    // rays stay inside the 85 degree cut in b.
    std::mt19937 rng(3);
    std::uniform_real_distribution<double> off_axis(60.0, 80.0);
    std::uniform_real_distribution<double> azimuth(0.0, 2.0 * std::numbers::pi);
    std::uniform_real_distribution<double> distance(1.0, 2.0);
    std::vector<geometry::TPoint3DF> world;
    for (int i = 0; i < 200; ++i) {
        const double theta = off_axis(rng) * std::numbers::pi / 180.0;
        const double phi = azimuth(rng);
        world.push_back(distance(rng) * geometry::TPoint3DF(std::sin(theta) * std::cos(phi),
                                                            std::sin(theta) * std::sin(phi), std::cos(theta)));
    }
    Scene scene = MakeScene(world, Translation(0, 0, 0), Translation(0, 0, -0.5), camera);
    std::normal_distribution<double> noise(0.0, 0.5);
    for (Eigen::Index i = 0; i < scene.pixels_a.cols(); ++i) {
        scene.pixels_a.col(i) += geometry::TPoint2DF(noise(rng), noise(rng));
        scene.pixels_b.col(i) += geometry::TPoint2DF(noise(rng), noise(rng));
    }
    const Measurement ma = MakeMeasurement(scene.pixels_a, scene.descriptors, scene.levels_a, camera);
    const Measurement mb = MakeMeasurement(scene.pixels_b, scene.descriptors, scene.levels_b, camera);

    const std::vector<TriangulatedPoint> points = MatchAndTriangulate(MakeView(ma, scene.T_aw), MakeView(mb, scene.T_bw));
    size_t recovered = 0;
    for (const TriangulatedPoint& p : points) {
        ASSERT_EQ(p.feature_a, p.feature_b);
        const geometry::TPoint3DF& truth = world[p.feature_a];
        if ((p.position - truth).norm() <= 0.02 * truth.norm()) {
            ++recovered;
        }
    }
    EXPECT_GE(recovered, 160U);
}

TEST(TriangulationTest, SkipsFeaturesWithPoints) {
    const geometry::Camera camera = PinholeCamera();
    const Scene scene = MakeScene(PinholePoints(100), Translation(0, 0, 0), Translation(0.3, 0, 0), camera);
    const Measurement ma = MakeMeasurement(scene.pixels_a, scene.descriptors, scene.levels_a, camera);
    const Measurement mb = MakeMeasurement(scene.pixels_b, scene.descriptors, scene.levels_b, camera);
    View a = MakeView(ma, scene.T_aw);
    View b = MakeView(mb, scene.T_bw);
    for (size_t i = 0; i < 100; i += 3) {
        a.has_point[i] = true;
    }
    for (size_t i = 1; i < 100; i += 5) {
        b.has_point[i] = true;
    }

    const Pairs matches = SearchForTriangulation(a, b, TriangulationOptions{});
    EXPECT_GT(matches.size(), 40U);
    for (const auto& [ia, ib] : matches) {
        EXPECT_FALSE(a.has_point[ia]);
        EXPECT_FALSE(b.has_point[ib]);
    }
}

TEST(TriangulationTest, RejectsLowParallax) {
    const geometry::Camera camera = PinholeCamera();
    const Scene scene = MakeScene(PinholePoints(100, 5.0, 5.0), Translation(0, 0, 0), Translation(0.001, 0, 0), camera);
    const Measurement ma = MakeMeasurement(scene.pixels_a, scene.descriptors, scene.levels_a, camera);
    const Measurement mb = MakeMeasurement(scene.pixels_b, scene.descriptors, scene.levels_b, camera);
    const View a = MakeView(ma, scene.T_aw);
    const View b = MakeView(mb, scene.T_bw);

    const Pairs matches = SearchForTriangulation(a, b, TriangulationOptions{});
    EXPECT_FALSE(matches.empty());  // the rejection is the parallax gate, not the matcher
    EXPECT_TRUE(TriangulateMatches(a, b, matches, TriangulationOptions{}).empty());
}

TEST(TriangulationTest, RejectsReprojectionOutlier) {
    const geometry::Camera camera = PinholeCamera();
    Scene scene = MakeScene(PinholePoints(50), Translation(0, 0, 0), Translation(0.3, 0, 0), camera);
    // Pure x translation: epipolar lines are image rows, so a y shift is
    // perpendicular to the epipolar curve.
    constexpr uint32_t kShifted = 17;
    scene.pixels_b(1, kShifted) += 5.0;
    const Measurement ma = MakeMeasurement(scene.pixels_a, scene.descriptors, scene.levels_a, camera);
    const Measurement mb = MakeMeasurement(scene.pixels_b, scene.descriptors, scene.levels_b, camera);
    const View a = MakeView(ma, scene.T_aw);
    const View b = MakeView(mb, scene.T_bw);

    // The descriptor alone would pair them (distance 0, the rest ~128 away).
    EXPECT_EQ(TestExtractor::Distance(ma.descriptors().col(kShifted), mb.descriptors().col(kShifted)), 0);

    // 5 px is outside the 3 sigma epipolar band, so matching already drops it.
    const Pairs matches = SearchForTriangulation(a, b, TriangulationOptions{});
    for (const auto& [ia, ib] : matches) {
        EXPECT_NE(ia, kShifted);
        EXPECT_NE(ib, kShifted);
    }
    const std::vector<TriangulatedPoint> points = TriangulateMatches(a, b, matches, TriangulationOptions{});
    EXPECT_GE(points.size(), 45U);
    for (const TriangulatedPoint& p : points) {
        EXPECT_NE(p.feature_a, kShifted);
        EXPECT_NE(p.feature_b, kShifted);
    }
    // Forced through acceptance, the raw-pixel chi2 gate rejects it.
    EXPECT_TRUE(TriangulateMatches(a, b, {{kShifted, kShifted}}, TriangulationOptions{}).empty());
}

TEST(TriangulationTest, ZeroBaselineCreatesNothing) {
    const geometry::Camera camera = PinholeCamera();
    const Scene scene = MakeScene(PinholePoints(50), Translation(0.2, 0, 0), Translation(0.2, 0, 0), camera);
    const Measurement ma = MakeMeasurement(scene.pixels_a, scene.descriptors, scene.levels_a, camera);
    const Measurement mb = MakeMeasurement(scene.pixels_b, scene.descriptors, scene.levels_b, camera);
    const View a = MakeView(ma, scene.T_aw);
    const View b = MakeView(mb, scene.T_bw);

    EXPECT_TRUE(SearchForTriangulation(a, b, TriangulationOptions{}).empty());
    Pairs forced;
    for (uint32_t i = 0; i < 50; ++i) {
        forced.emplace_back(i, i);
    }
    EXPECT_TRUE(TriangulateMatches(a, b, forced, TriangulationOptions{}).empty());
}

TEST(TriangulationTest, StereoFallbackUsesDepth) {
    const geometry::Camera camera = PinholeCamera();
    const Scene scene = MakeScene(PinholePoints(20), Translation(0, 0, 0), Translation(0.002, 0, 0), camera);
    const Measurement ma = MakeMeasurement(scene.pixels_a, scene.descriptors, scene.levels_a, camera);
    const Measurement mb = MakeMeasurement(scene.pixels_b, scene.descriptors, scene.levels_b, camera);
    View a = MakeView(ma, scene.T_aw);
    const View b = MakeView(mb, scene.T_bw);
    a.baseline = 0.11;
    a.stereo_depth.assign(20, -1.0);
    // A depth 0.1 % off the truth, so the stereo placement differs from the
    // (exact) two-view one.
    constexpr uint32_t kStereo = 4;
    a.stereo_depth[kStereo] = 1.001 * scene.T_aw.Transform(scene.points[kStereo]).z();

    const std::vector<TriangulatedPoint> points = MatchAndTriangulate(a, b);
    ASSERT_EQ(points.size(), 1U);  // the 2 mm two-view parallax alone is too low
    EXPECT_EQ(points[0].feature_a, kStereo);
    EXPECT_EQ(points[0].feature_b, kStereo);
    const geometry::TPoint3DF expected =
        scene.T_aw.Inverse().Transform(
            geometry::TPoint3DF(ma.normalized_point(static_cast<int>(kStereo)) * a.stereo_depth[kStereo]));
    EXPECT_LT((points[0].position - expected).norm(), 1e-9);
    EXPECT_GT((points[0].position - scene.points[kStereo]).norm(), 1e-3);
}

TEST(TriangulationTest, ScaleConsistencyFollowsOctaveRatio) {
    // X is 1 m from a and 1.2^4 m from b. Seen at level 4 in a and 0 in b, the
    // patch looks 1.2^4 larger in a, consistent with a being closer: accepted.
    // Swapped or equal levels contradict the distances and are rejected.
    const geometry::Camera camera = PinholeCamera();
    const geometry::TPoint3DF X(0.0, 0.0, 1.0);
    const double dist_b = std::pow(1.2, 4);
    // b looks at X along its optical axis from a side position (~67 deg parallax).
    const geometry::TVector3DF centre_b(0.0, std::sqrt(dist_b * dist_b - 0.64), 0.2);
    const geometry::TVector3DF z_axis = (X - centre_b).normalized();
    const geometry::TVector3DF x_axis = geometry::TVector3DF::UnitX();
    const geometry::TVector3DF y_axis = z_axis.cross(x_axis);
    geometry::TMatrix33F R_bw;
    R_bw.row(0) = x_axis.transpose();
    R_bw.row(1) = y_axis.transpose();
    R_bw.row(2) = z_axis.transpose();
    const geometry::RigidTransformation T_bw_looking(R_bw, -(R_bw * centre_b));

    const Scene scene = MakeScene({X}, Translation(0, 0, 0), T_bw_looking, camera);
    for (const auto& [level_a, level_b, accepted] :
         {std::tuple{4, 0, true}, std::tuple{0, 4, false}, std::tuple{0, 0, false}}) {
        const Measurement ma = MakeMeasurement(scene.pixels_a, scene.descriptors,
                                               features::TLevels::Constant(1, static_cast<uint8_t>(level_a)), camera);
        const Measurement mb = MakeMeasurement(scene.pixels_b, scene.descriptors,
                                               features::TLevels::Constant(1, static_cast<uint8_t>(level_b)), camera);
        const std::vector<TriangulatedPoint> points =
            TriangulateMatches(MakeView(ma, scene.T_aw), MakeView(mb, scene.T_bw), {{0U, 0U}}, TriangulationOptions{});
        EXPECT_EQ(points.size(), accepted ? 1U : 0U) << "levels " << level_a << "/" << level_b;
    }
}

}  // namespace
}  // namespace voyahslam::mapping
