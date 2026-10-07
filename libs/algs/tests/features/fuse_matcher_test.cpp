#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <numbers>
#include <optional>
#include <random>
#include <utility>
#include <vector>

#include <Eigen/Geometry>

#include "support/test_measurements.h"
#include <voyahslam/features/fuse_matcher.h>
#include <voyahslam/geometry/camera.h>
#include <voyahslam/geometry/distortion_models/kannala_brandt_distortion_model.h>

namespace voyahslam::features {
namespace {

using voyahslam::test::TestExtractor;
using voyahslam::test::TestImageFeatures;
using Measurement = TMonocularMeasurement<TestExtractor>;
using Descriptor = TestImageFeatures::TDescriptor;
using Candidate = FuseCandidatePoint<TestExtractor>;
using FeaturePoints = std::vector<std::optional<map::MapPointId>>;

constexpr map::MapPointId kPoint{7};

// Random descriptor of `seed` with its `flips` low bits inverted: two
// descriptors of the same seed are |flips_a - flips_b| apart; different seeds
// are ~128 apart.
Descriptor MakeDescriptor(int seed, int flips = 0) {
    std::mt19937 rng(static_cast<uint32_t>(seed));
    Descriptor d;
    for (Eigen::Index i = 0; i < d.size(); ++i) d[i] = static_cast<uint8_t>(rng() & 0xFFU);
    for (int b = 0; b < flips; ++b) d[b / 8] ^= static_cast<uint8_t>(1U << (b % 8));
    return d;
}

geometry::TMatrix33F PinholeK() {
    geometry::TMatrix33F K = geometry::TMatrix33F::Identity();
    K(0, 0) = K(1, 1) = 500.0;
    K(0, 2) = 320.0;
    K(1, 2) = 240.0;
    return K;
}

// TUM-VI cam0 (equidistant fisheye, 1024x1024).
geometry::TMatrix33F FisheyeK() {
    geometry::TMatrix33F K = geometry::TMatrix33F::Identity();
    K(0, 0) = 380.81042871360756;
    K(1, 1) = 380.81194179427075;
    K(0, 2) = 510.29465304840727;
    K(1, 2) = 514.3304630538506;
    return K;
}

const geometry::distortion_models::KannalaBrandtDistortionModel kFisheyeModel(
    0.010171079892421483, -0.010816440029919381, 0.005942781769412756, -0.001662284667857643);

enum class Lens { kPinhole, kFisheye };

geometry::Camera MakeCamera(Lens lens) {
    return lens == Lens::kPinhole ? geometry::Camera{PinholeK(), nullptr}
                                  : geometry::Camera{FisheyeK(), &kFisheyeModel};
}

// Camera-frame point at ray angle `theta_deg` off the optical axis, azimuth
// `phi_deg`, at range `range`.
geometry::TPoint3DF RayPoint(double theta_deg, double phi_deg, double range) {
    const double theta = theta_deg * std::numbers::pi / 180.0;
    const double phi = phi_deg * std::numbers::pi / 180.0;
    return range * geometry::TPoint3DF(std::sin(theta) * std::cos(phi), std::sin(theta) * std::sin(phi),
                                       std::cos(theta));
}

struct Feature {
    geometry::TPoint2DF pixel;
    Descriptor descriptor;
    int level = 0;
};

// The frame: one test camera at a non-trivial pose, its features (in order),
// then stretchers spanning the raw-point bounding box.
struct Scene {
    Lens lens = Lens::kPinhole;
    geometry::RigidTransformation T_cw{
        Eigen::AngleAxis<precision_t>(0.3, geometry::TVector3DF(0.2, 1.0, -0.4).normalized()).toRotationMatrix(),
        geometry::TVector3DF(0.5, -0.2, 1.0)};
    std::vector<Feature> features;
    std::vector<geometry::TPoint2DF> stretch;

    explicit Scene(Lens l) : lens(l) {
        if (lens == Lens::kPinhole) {
            stretch = {{0.0, 0.0}, {640.0, 480.0}};
        } else {
            // Inside the 85° measurement cut, but spanning a box that also
            // holds diagonal rays slightly beyond 85°.
            const geometry::Camera camera = MakeCamera(lens);
            stretch = {camera.Project(RayPoint(84.0, 0.0, 1.0)), camera.Project(RayPoint(84.0, 90.0, 1.0)),
                       camera.Project(RayPoint(84.0, 180.0, 1.0)), camera.Project(RayPoint(84.0, 270.0, 1.0))};
        }
    }

    [[nodiscard]] geometry::Camera camera() const { return MakeCamera(lens); }

    [[nodiscard]] geometry::TPoint2DF Project(const geometry::TPoint3DF& Xc) const { return camera().Project(Xc); }

    // World point of camera point Xc.
    [[nodiscard]] geometry::TPoint3DF World(const geometry::TPoint3DF& Xc) const {
        return T_cw.Inverse().Transform(Xc);
    }

    // A candidate seen from this camera at Xc, its normal along the view ray
    // and its reference at the same distance (predicted level `level`).
    [[nodiscard]] Candidate MakeCandidate(map::MapPointId id, const geometry::TPoint3DF& Xc, const Descriptor& d,
                                          int level = 0) const {
        const geometry::TPoint3DF X = World(Xc);
        const geometry::TPoint3DF centre = T_cw.Inverse().translation();
        const double distance = (X - centre).norm();
        return Candidate{id, X, (X - centre).normalized(), d,
                         map::PointReference{map::FrameId{0}, 0, level, distance}};
    }

    [[nodiscard]] Measurement MakeMeasurement() const {
        std::vector<Feature> all = features;
        int seed = 5000;
        for (const geometry::TPoint2DF& pixel : stretch) {
            all.push_back({pixel, MakeDescriptor(seed++), 0});
        }
        const auto n = static_cast<Eigen::Index>(all.size());
        geometry::TPoints2DF points(2, n);
        TestImageFeatures::TDescriptors descriptors(TestExtractor::DescriptorSize, n);
        TLevels levels(n);
        for (Eigen::Index i = 0; i < n; ++i) {
            const Feature& f = all[static_cast<size_t>(i)];
            points.col(i) = f.pixel;
            descriptors.col(i) = f.descriptor;
            levels[i] = static_cast<uint8_t>(f.level);
        }
        const geometry::Camera cam = camera();
        Measurement measurement(TestImageFeatures(std::move(points), geometry::TRowVectorF::Ones(n),
                                                  geometry::TRowVectorF::Zero(n), std::move(descriptors),
                                                  std::move(levels)),
                                cam.K, cam.distortion, 0.0);
        EXPECT_EQ(measurement.points().cols(), n);  // nothing dropped by the 85° cut
        return measurement;
    }

    [[nodiscard]] FeaturePoints NoPoints() const {
        return FeaturePoints(features.size() + stretch.size(), std::nullopt);
    }
};

const geometry::TPoint3DF kXc(0.3, -0.2, 4.0);

// One feature 0.5 px off the projection of kXc, 10 bits off the point's
// descriptor, and a candidate at kXc.
Scene BaseScene(Lens lens, std::vector<Candidate>& out_candidates) {
    Scene scene(lens);
    scene.features.push_back({scene.Project(kXc) + geometry::TPoint2DF(0.4, -0.3), MakeDescriptor(1, 10), 0});
    out_candidates = {scene.MakeCandidate(kPoint, kXc, MakeDescriptor(1))};
    return scene;
}

std::vector<FuseDecision> Fuse(const Scene& scene, const FeaturePoints& feature_points,
                              const std::vector<Candidate>& candidates, const FuseOptions& options = {},
                              const std::optional<StereoView>& stereo = std::nullopt) {
    return FuseMatches<TestExtractor>(scene.MakeMeasurement(), scene.T_cw, stereo, feature_points, candidates,
                                      options);
}

TEST(FuseMatchesTest, AddsObservationForFreeFeature) {
    for (const Lens lens : {Lens::kPinhole, Lens::kFisheye}) {
        SCOPED_TRACE(lens == Lens::kPinhole ? "pinhole" : "fisheye");
        std::vector<Candidate> candidates;
        const Scene scene = BaseScene(lens, candidates);
        const std::vector<FuseDecision> decisions = Fuse(scene, scene.NoPoints(), candidates);
        ASSERT_EQ(decisions.size(), 1U);
        EXPECT_EQ(decisions[0].point, kPoint);
        EXPECT_EQ(decisions[0].feature, 0U);
        EXPECT_FALSE(decisions[0].existing.has_value());
    }
}

TEST(FuseMatchesTest, ReportsExistingForDuplicate) {
    for (const Lens lens : {Lens::kPinhole, Lens::kFisheye}) {
        SCOPED_TRACE(lens == Lens::kPinhole ? "pinhole" : "fisheye");
        std::vector<Candidate> candidates;
        const Scene scene = BaseScene(lens, candidates);
        FeaturePoints feature_points = scene.NoPoints();
        feature_points[0] = map::MapPointId{3};
        const std::vector<FuseDecision> decisions = Fuse(scene, feature_points, candidates);
        ASSERT_EQ(decisions.size(), 1U);
        EXPECT_EQ(decisions[0].point, kPoint);
        EXPECT_EQ(decisions[0].feature, 0U);
        ASSERT_TRUE(decisions[0].existing.has_value());
        EXPECT_EQ(*decisions[0].existing, map::MapPointId{3});
    }
}

TEST(FuseMatchesTest, RejectsBehindOutsideAngleScaleChi2Descriptor) {
    const Lens lens = Lens::kPinhole;
    std::vector<Candidate> base;
    const Scene scene = BaseScene(lens, base);
    const FeaturePoints none = scene.NoPoints();
    ASSERT_EQ(Fuse(scene, none, base).size(), 1U);  // the control matches

    {
        SCOPED_TRACE("behind");
        // Mirrored through the camera centre: the same pixel under a pinhole.
        std::vector<Candidate> candidates = {scene.MakeCandidate(kPoint, -kXc, MakeDescriptor(1))};
        candidates[0].normal = -candidates[0].normal;
        EXPECT_TRUE(Fuse(scene, none, candidates).empty());
    }
    {
        SCOPED_TRACE("outside the image");
        Scene edge(lens);
        const geometry::TPoint3DF inside(4.0 * 319.0 / 500.0, 0.0, 4.0);    // x = 639 px
        const geometry::TPoint3DF outside(4.0 * 320.5 / 500.0, 0.0, 4.0);  // x = 640.5 px
        edge.features.push_back({edge.Project(inside), MakeDescriptor(1, 10), 0});
        ASSERT_EQ(Fuse(edge, edge.NoPoints(), {edge.MakeCandidate(kPoint, inside, MakeDescriptor(1))}).size(), 1U);
        EXPECT_TRUE(Fuse(edge, edge.NoPoints(), {edge.MakeCandidate(kPoint, outside, MakeDescriptor(1))}).empty());
    }
    {
        SCOPED_TRACE("view angle");
        std::vector<Candidate> candidates = base;
        const geometry::TPoint3DF view = candidates[0].normal;
        const geometry::TPoint3DF side = view.cross(geometry::TPoint3DF::UnitX()).normalized();
        const double tilt = 61.0 * std::numbers::pi / 180.0;  // cos 61° < 0.5
        candidates[0].normal = std::cos(tilt) * view + std::sin(tilt) * side;
        EXPECT_TRUE(Fuse(scene, none, candidates).empty());
        const double ok = 59.0 * std::numbers::pi / 180.0;
        candidates[0].normal = std::cos(ok) * view + std::sin(ok) * side;
        EXPECT_EQ(Fuse(scene, none, candidates).size(), 1U);
    }
    {
        SCOPED_TRACE("scale: too far");
        std::vector<Candidate> candidates = base;
        candidates[0].reference.distance /= 1.25;  // max distance 0.8 d < d / 1.2
        EXPECT_TRUE(Fuse(scene, none, candidates).empty());
        candidates[0].reference.distance = base[0].reference.distance / 1.15;  // within +20 %
        EXPECT_EQ(Fuse(scene, none, candidates).size(), 1U);
    }
    {
        SCOPED_TRACE("scale: too close");
        // min distance = reference.distance / 1.2^7. The predicted level
        // clamps to 7, so the feature sits there.
        Scene top = scene;
        top.features[0].level = 7;
        std::vector<Candidate> candidates = base;
        candidates[0].reference.distance = base[0].reference.distance * std::pow(1.2, 7) * 1.3;  // 0.8 min > d
        EXPECT_TRUE(Fuse(top, none, candidates).empty());
        candidates[0].reference.distance = base[0].reference.distance * std::pow(1.2, 7) * 1.2;  // 0.8 min < d
        EXPECT_EQ(Fuse(top, none, candidates).size(), 1U);
    }
    {
        SCOPED_TRACE("chi2");
        Scene off = scene;
        off.features[0].pixel = scene.Project(kXc) + geometry::TPoint2DF(2.5, 0.0);  // chi2 6.25, inside radius 3
        EXPECT_TRUE(Fuse(off, none, base).empty());
        off.features[0].pixel = scene.Project(kXc) + geometry::TPoint2DF(2.4, 0.0);  // chi2 5.76
        EXPECT_EQ(Fuse(off, none, base).size(), 1U);
    }
    {
        SCOPED_TRACE("descriptor");
        Scene far = scene;
        far.features[0].descriptor = MakeDescriptor(1, 51);
        EXPECT_TRUE(Fuse(far, none, base).empty());
        far.features[0].descriptor = MakeDescriptor(1, 50);
        EXPECT_EQ(Fuse(far, none, base).size(), 1U);
    }
}

TEST(FuseMatchesTest, RejectsBeyondMaxRayAngleFisheye) {
    Scene scene(Lens::kFisheye);
    // Diagonal rays: inside the stretchers' box, either side of 85°.
    const geometry::TPoint3DF inside = RayPoint(84.95, 45.0, 3.0);
    const geometry::TPoint3DF beyond = RayPoint(85.05, 45.0, 3.0);
    scene.features.push_back({scene.Project(inside), MakeDescriptor(1, 10), 0});
    ASSERT_LT((scene.Project(beyond) - scene.Project(inside)).norm(), 1.0);
    EXPECT_EQ(Fuse(scene, scene.NoPoints(), {scene.MakeCandidate(kPoint, inside, MakeDescriptor(1))}).size(), 1U);
    EXPECT_TRUE(Fuse(scene, scene.NoPoints(), {scene.MakeCandidate(kPoint, beyond, MakeDescriptor(1))}).empty());
}

TEST(FuseMatchesTest, SkipsPointsAlreadyObserved) {
    std::vector<Candidate> candidates;
    const Scene scene = BaseScene(Lens::kPinhole, candidates);
    FeaturePoints feature_points = scene.NoPoints();
    feature_points[1] = kPoint;  // a stretcher observes the candidate
    EXPECT_TRUE(Fuse(scene, feature_points, candidates).empty());
}

TEST(FuseMatchesTest, ChainedDuplicatesResolveToOneWinner) {
    std::vector<Candidate> base;
    const Scene scene = BaseScene(Lens::kPinhole, base);
    FeaturePoints feature_points = scene.NoPoints();
    feature_points[0] = map::MapPointId{3};
    const geometry::TPoint3DF nearby = kXc + geometry::TPoint3DF(0.004, 0.0, 0.0);  // 0.5 px away
    const Candidate far = scene.MakeCandidate(map::MapPointId{8}, kXc, MakeDescriptor(1, 30));
    const Candidate close = scene.MakeCandidate(map::MapPointId{9}, nearby, MakeDescriptor(1, 5));
    for (const std::vector<Candidate>& candidates :
         {std::vector<Candidate>{far, close}, std::vector<Candidate>{close, far}}) {
        const std::vector<FuseDecision> decisions = Fuse(scene, feature_points, candidates);
        ASSERT_EQ(decisions.size(), 1U);
        EXPECT_EQ(decisions[0].point, map::MapPointId{9});
        EXPECT_EQ(decisions[0].feature, 0U);
        ASSERT_TRUE(decisions[0].existing.has_value());
        EXPECT_EQ(*decisions[0].existing, map::MapPointId{3});
    }
}

TEST(FuseMatchesTest, StereoUsesRightResidual) {
    std::vector<Candidate> candidates;
    Scene scene = BaseScene(Lens::kPinhole, candidates);
    scene.features[0].pixel = scene.Project(kXc);  // exact on the left
    const optimization::StereoCameras cameras{
        geometry::Camera{PinholeK(), nullptr},
        geometry::RigidTransformation(geometry::TMatrix33F::Identity(), geometry::TVector3DF(-0.11, 0.0, 0.0))};
    const geometry::TPoint2DF right_exact = cameras.right.Project(cameras.T_rl.Transform(kXc));
    const FeaturePoints none = scene.NoPoints();

    geometry::TPoints2DF right_points(2, 1);
    Eigen::VectorXi right_indices = Eigen::VectorXi::Constant(static_cast<Eigen::Index>(none.size()), -1);
    right_indices(0) = 0;
    const StereoView stereo{&right_points, &right_indices, cameras};

    right_points.col(0) = right_exact;
    EXPECT_EQ(Fuse(scene, none, candidates, {}, stereo).size(), 1U);

    right_points.col(0) = right_exact + geometry::TPoint2DF(5.0, 0.0);
    EXPECT_TRUE(Fuse(scene, none, candidates, {}, stereo).empty());

    // Without a right match the mono test applies: the right image is unused.
    right_indices(0) = -1;
    EXPECT_EQ(Fuse(scene, none, candidates, {}, stereo).size(), 1U);
}

}  // namespace
}  // namespace voyahslam::features
