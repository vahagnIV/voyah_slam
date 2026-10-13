#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <iostream>
#include <map>
#include <memory>
#include <numbers>
#include <optional>
#include <random>
#include <utility>
#include <variant>
#include <vector>

#include <Eigen/Geometry>

#include "support/synthetic_sequence.h"
#include "support/test_measurements.h"
#include <voyahslam/events/async_observer.h>
#include <voyahslam/events/events.h>
#include <voyahslam/events/observable.h>
#include <voyahslam/features/projection_matcher.h>
#include <voyahslam/geometry/camera.h>
#include <voyahslam/geometry/distortion_models/kannala_brandt_distortion_model.h>
#include <voyahslam/geometry/sim3.h>
#include <voyahslam/mapping/mapper.h>
#include <voyahslam/tracking/map_replica.h>

namespace voyahslam::features {
namespace {

using voyahslam::test::TestExtractor;
using voyahslam::test::TestImageFeatures;
using Measurement = TMonocularMeasurement<TestExtractor>;
using Descriptor = TestImageFeatures::TDescriptor;
using TReplica = tracking::MapReplica<TestExtractor>;
using TTransaction = events::MapTransaction<TestExtractor>;

constexpr double kFocal = 500.0;
constexpr double kCx = 320.0;
constexpr double kCy = 240.0;

geometry::TMatrix33F Intrinsics() {
    geometry::TMatrix33F K = geometry::TMatrix33F::Identity();
    K(0, 0) = K(1, 1) = kFocal;
    K(0, 2) = kCx;
    K(1, 2) = kCy;
    return K;
}

geometry::TPoint2DF Project(const Eigen::Vector3d& X) {
    return {kCx + kFocal * X.x() / X.z(), kCy + kFocal * X.y() / X.z()};
}

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

struct Feature {
    geometry::TPoint2DF pixel{kCx, kCy};
    Descriptor descriptor = MakeDescriptor(1000);
    int level = 0;
    double angle = 0.0;
};

// TUM-VI cam0 (equidistant fisheye, 1024x1024).
geometry::TMatrix33F TumViIntrinsics() {
    geometry::TMatrix33F K = geometry::TMatrix33F::Identity();
    K(0, 0) = 380.81042871360756;
    K(1, 1) = 380.81194179427075;
    K(0, 2) = 510.29465304840727;
    K(1, 2) = 514.3304630538506;
    return K;
}

geometry::distortion_models::TDistortionModelPtr TumViModel() {
    return std::make_shared<const geometry::distortion_models::KannalaBrandtDistortionModel>(
        0.010171079892421483, -0.010816440029919381, 0.005942781769412756, -0.001662284667857643);
}

// Features in order (raw pixels), then two stretchers so the point bounding
// box spans [0, 640] x [0, 480] (or `stretch` when given).
Measurement MakeMeasurement(std::vector<Feature> features,
                            const std::vector<geometry::TPoint2DF>& stretch = {{0.0, 0.0},
                                                                               {640.0, 480.0}},
                            const geometry::TMatrix33F& K = Intrinsics(),
                            const geometry::distortion_models::TDistortionModelPtr& distortion = nullptr) {
    int seed = 5000;
    for (const geometry::TPoint2DF& pixel : stretch) {
        features.push_back({pixel, MakeDescriptor(seed++), 0, 0.0});
    }
    const auto n = static_cast<Eigen::Index>(features.size());
    geometry::TPoints2DF points(2, n);
    geometry::TRowVectorF angles(n);
    TestImageFeatures::TDescriptors descriptors(TestExtractor::DescriptorSize, n);
    TLevels levels(n);
    for (Eigen::Index i = 0; i < n; ++i) {
        const Feature& f = features[static_cast<size_t>(i)];
        points.col(i) = f.pixel;
        angles[i] = f.angle;
        descriptors.col(i) = f.descriptor;
        levels[i] = static_cast<uint8_t>(f.level);
    }
    Measurement measurement(TestImageFeatures(std::move(points), geometry::TRowVectorF::Ones(n),
                                              std::move(angles), std::move(descriptors),
                                              std::move(levels)),
                            K, distortion, 0.0);
    EXPECT_EQ(measurement.points().cols(), n);  // nothing dropped by the 85° cut
    return measurement;
}

struct Point {
    Eigen::Vector3d position;
    Descriptor descriptor;
    Eigen::Vector3d normal = Eigen::Vector3d::Zero();  // zero: the view direction from the origin
};

// Adds the points through a transaction whose Frame (no features) observes
// none of them, so they get no reference.
void AddPoints(TReplica& out_replica, const std::vector<Point>& points) {
    auto event = std::make_shared<TTransaction>();
    event->map_id = {1};
    events::NewFrame<TestExtractor>& added = event->added_frames.emplace_back();
    added.id = {1000 + static_cast<uint32_t>(out_replica.num_points())};
    added.measurement = test::Share(features::TMonocularMeasurement<TestExtractor>{});
    event->new_points_start_id = {static_cast<uint32_t>(out_replica.num_points())};
    event->ResizeNewPoints(static_cast<uint32_t>(points.size()));
    for (size_t i = 0; i < points.size(); ++i) {
        const auto col = static_cast<Eigen::Index>(i);
        event->new_positions.col(col) = points[i].position;
        event->new_descriptors.col(col) = points[i].descriptor;
        event->new_normals.col(col) =
            points[i].normal.isZero() ? points[i].position.normalized() : points[i].normal;
    }
    out_replica.Apply(*event);
}

void RemovePoint(TReplica& out_replica, uint32_t id) {
    auto event = std::make_shared<TTransaction>();
    event->map_id = {1};
    event->removed_points = {{id}};
    out_replica.Apply(*event);
}

// A keyframe with centre `centre` (identity rotation) whose feature i, at
// level levels[i], observes point points[i]: gives those points their reference.
void CreateKeyFrame(TReplica& out_replica, uint32_t frame, const Eigen::Vector3d& centre,
                    const std::vector<int>& levels, const std::vector<uint32_t>& points) {
    auto event = std::make_shared<TTransaction>();
    event->map_id = {1};
    events::NewFrame<TestExtractor>& added = event->added_frames.emplace_back();
    added.id = {frame};
    added.T_cw = geometry::RigidTransformation(geometry::TMatrix33F::Identity(), -centre);
    std::vector<Feature> features;
    for (const int level : levels) features.push_back({{kCx, kCy}, MakeDescriptor(9000), level, 0.0});
    added.measurement = test::Share(MakeMeasurement(features, {}));
    for (size_t i = 0; i < points.size(); ++i) {
        added.observed_features.push_back(static_cast<uint32_t>(i));
        added.observed_points.push_back({points[i]});
    }
    out_replica.Apply(*event);
}

// Last frame whose feature i has level levels[i] and observes point i.
Measurement MakeLastFrame(const std::vector<int>& levels, const std::vector<double>& angles = {}) {
    std::vector<Feature> features;
    for (size_t i = 0; i < levels.size(); ++i) {
        features.push_back({{kCx, kCy}, MakeDescriptor(9000), levels[i],
                            angles.empty() ? 0.0 : angles[i]});
    }
    return MakeMeasurement(features);
}

std::vector<int64_t> IdentityMatches(size_t num_features, size_t num_points) {
    std::vector<int64_t> matches(num_features, -1);
    for (size_t i = 0; i < num_points; ++i) matches[i] = static_cast<int64_t>(i);
    return matches;
}

size_t CountMatches(const std::vector<int64_t>& matches) {
    size_t n = 0;
    for (const int64_t m : matches) n += m >= 0 ? 1 : 0;
    return n;
}

const geometry::RigidTransformation kIdentity = geometry::RigidTransformation::Identity();

TEST(ProjectionMatcherTest, LastFrameWindowAndLevels) {
    TReplica replica;
    const std::vector<Point> points = {
        {{-1.0, -0.5, 5.0}, MakeDescriptor(1)},  // in window, same level
        {{0.0, 0.5, 5.0}, MakeDescriptor(2)},    // candidate outside the radius
        {{1.0, -0.5, 5.0}, MakeDescriptor(3)},   // candidate 2 levels away
        {{1.0, 0.5, 5.0}, MakeDescriptor(4)},    // candidate 1 level away
    };
    AddPoints(replica, points);
    const Measurement last = MakeLastFrame({1, 0, 3, 3});
    const Measurement current = MakeMeasurement({
        {Project(points[0].position) + geometry::TPoint2DF(11.0, 0.0), MakeDescriptor(1, 5), 1},
        {Project(points[1].position) + geometry::TPoint2DF(10.5, 0.0), MakeDescriptor(2, 5), 0},
        {Project(points[2].position), MakeDescriptor(3, 5), 1},
        {Project(points[3].position), MakeDescriptor(4, 5), 2},
    });
    // radius 10 px: level 1 -> 12 px, level 0 -> 10 px.
    const std::vector<int64_t> matches =
        SearchFromLastFrame<TestExtractor>(current, kIdentity, last, IdentityMatches(6, 4), replica,
                                           10.0, ProjectionMatcherOptions{});
    ASSERT_EQ(matches.size(), 6u);
    EXPECT_EQ(matches[0], 0);
    EXPECT_EQ(matches[1], -1);
    EXPECT_EQ(matches[2], -1);
    EXPECT_EQ(matches[3], 3);
    EXPECT_EQ(CountMatches(matches), 2u);
}

TEST(ProjectionMatcherTest, LastFrameDescriptorThresholdAndProjection) {
    TReplica replica;
    const std::vector<Point> points = {
        {{0.0, 0.0, 5.0}, MakeDescriptor(1)},
        {{0.0, 0.0, -5.0}, MakeDescriptor(2)},  // behind the camera, projects onto the centre too
    };
    AddPoints(replica, points);
    const Measurement last = MakeLastFrame({0, 0});
    ProjectionMatcherOptions options;
    options.max_distance_last = 4.0;
    Measurement current = MakeMeasurement({
        {{kCx + 2.0, kCy}, MakeDescriptor(1, 5), 0},
        {{kCx - 2.0, kCy}, MakeDescriptor(2), 0},
    });
    std::vector<int64_t> matches = SearchFromLastFrame<TestExtractor>(
        current, kIdentity, last, IdentityMatches(4, 2), replica, 10.0, options);
    EXPECT_EQ(CountMatches(matches), 0u);  // distance 5 > 4; point 1 behind the camera

    options.max_distance_last = 5.0;
    matches = SearchFromLastFrame<TestExtractor>(current, kIdentity, last, IdentityMatches(4, 2),
                                                 replica, 10.0, options);
    EXPECT_EQ(matches[0], 0);
    EXPECT_EQ(CountMatches(matches), 1u);
}

TEST(ProjectionMatcherTest, OneToOneClaims) {
    TReplica replica;
    // Points 0, 1 compete for feature 0 (farther claim first); points 2, 3 for
    // feature 1 (closer claim first). The closer claim wins either way.
    const std::vector<Point> points = {
        {{-0.5, 0.0, 5.0}, MakeDescriptor(1, 20)},
        {{-0.5, 0.0, 5.0}, MakeDescriptor(1, 30)},
        {{0.5, 0.0, 5.0}, MakeDescriptor(2, 30)},
        {{0.5, 0.0, 5.0}, MakeDescriptor(2, 20)},
    };
    AddPoints(replica, points);
    const Measurement last = MakeLastFrame({0, 0, 0, 0});
    const Measurement current = MakeMeasurement({
        {Project(points[0].position), MakeDescriptor(1, 32), 0},
        {Project(points[2].position), MakeDescriptor(2, 32), 0},
    });
    const std::vector<int64_t> matches = SearchFromLastFrame<TestExtractor>(
        current, kIdentity, last, IdentityMatches(6, 4), replica, 10.0, ProjectionMatcherOptions{});
    EXPECT_EQ(matches[0], 1);
    EXPECT_EQ(matches[1], 2);
    EXPECT_EQ(CountMatches(matches), 2u);
}

TEST(ProjectionMatcherTest, SkipsDeadPoints) {
    TReplica replica;
    const std::vector<Point> points = {
        {{-0.5, 0.0, 5.0}, MakeDescriptor(1)},
        {{0.5, 0.0, 5.0}, MakeDescriptor(2)},
    };
    AddPoints(replica, points);
    const Measurement last = MakeLastFrame({0, 0});
    // The last frame matched both; point 1 is removed afterwards.
    RemovePoint(replica, 1);
    const Measurement current = MakeMeasurement({
        {Project(points[0].position), MakeDescriptor(1, 2), 0},
        {Project(points[1].position), MakeDescriptor(2, 2), 0},
    });
    std::vector<int64_t> last_matches = IdentityMatches(4, 2);
    last_matches[2] = 7;  // an id the replica never issued
    const std::vector<int64_t> matches = SearchFromLastFrame<TestExtractor>(
        current, kIdentity, last, last_matches, replica, 10.0, ProjectionMatcherOptions{});
    EXPECT_EQ(matches[0], 0);
    EXPECT_EQ(matches[1], -1);
    EXPECT_EQ(CountMatches(matches), 1u);
}

TEST(ProjectionMatcherTest, RotationHistogramDropsInconsistent) {
    TReplica replica;
    std::vector<Point> points;
    std::vector<Feature> features;
    std::vector<double> last_angles;
    constexpr int kConsistent = 12;  // the odd one is below 0.1 x the fullest bin
    for (int i = 0; i <= kConsistent; ++i) {
        points.push_back({{-1.0 + 0.2 * i, 0.0, 5.0}, MakeDescriptor(i + 1)});
        // Current angle = last angle - 0.3 rad, except the last point (rotated by pi).
        last_angles.push_back(0.1 * i);
        const double rotation = i < kConsistent ? 0.3 : std::numbers::pi;
        features.push_back({Project(points.back().position), MakeDescriptor(i + 1, 2), 0,
                            0.1 * i - rotation});
    }
    AddPoints(replica, points);
    const Measurement last = MakeLastFrame(std::vector<int>(kConsistent + 1, 0), last_angles);
    const Measurement current = MakeMeasurement(features);
    const std::vector<int64_t> last_matches = IdentityMatches(kConsistent + 3, kConsistent + 1);

    std::vector<int64_t> matches = SearchFromLastFrame<TestExtractor>(
        current, kIdentity, last, last_matches, replica, 10.0, ProjectionMatcherOptions{});
    for (int i = 0; i < kConsistent; ++i) EXPECT_EQ(matches[i], i);
    EXPECT_EQ(matches[kConsistent], -1);
    EXPECT_EQ(CountMatches(matches), static_cast<size_t>(kConsistent));

    ProjectionMatcherOptions no_check;
    no_check.check_orientation = false;
    matches = SearchFromLastFrame<TestExtractor>(current, kIdentity, last, last_matches, replica,
                                                 10.0, no_check);
    EXPECT_EQ(matches[kConsistent], kConsistent);
    EXPECT_EQ(CountMatches(matches), static_cast<size_t>(kConsistent + 1));
}

TEST(ProjectionMatcherTest, LocalMapVisibility) {
    TReplica replica;
    const double tan_inside = 8.0;    // ray at 84.95°
    const double tan_outside = 8.2;   // ray at 85.07°
    const std::vector<Point> points = {
        {{0.0, 0.0, 5.0}, MakeDescriptor(1)},    // visible
        {{0.0, 0.0, -5.0}, MakeDescriptor(2)},   // behind the camera
        {{tan_outside, tan_outside, 1.0}, MakeDescriptor(3)},  // beyond 85°
        // cos(view, normal) = 0.45
        {{1.0, 0.0, 5.0}, MakeDescriptor(4),
         Eigen::AngleAxisd(std::acos(0.45), Eigen::Vector3d::UnitY()) *
             Eigen::Vector3d(1.0, 0.0, 5.0).normalized()},
        {{-1.0, 0.0, 5.0}, MakeDescriptor(5)},   // visible, but already matched
    };
    AddPoints(replica, points);
    CreateKeyFrame(replica, 0, Eigen::Vector3d::Zero(), {0, 0, 0, 0, 0}, {0, 1, 2, 3, 4});

    // The bounding box spans the beyond-85° projection; every candidate is an exact copy.
    const Measurement current = MakeMeasurement(
        {
            {Project(points[0].position), MakeDescriptor(1), 0},
            {{kCx + 1.0, kCy}, MakeDescriptor(2), 0},
            {{kCx + kFocal * tan_inside, kCy + kFocal * tan_inside}, MakeDescriptor(3), 0},
            {Project(points[3].position), MakeDescriptor(4), 0},
            {Project(points[4].position), MakeDescriptor(5), 0},
            {Project(points[4].position) + geometry::TPoint2DF(1.0, 0.0), MakeDescriptor(5), 0},
        },
        {{0.0, 0.0}, {kCx + kFocal * 10.0, kCy}, {kCx, kCy + kFocal * 10.0}});
    std::vector<int64_t> matches(static_cast<size_t>(current.points().cols()), -1);
    matches[4] = 4;
    // th = 50: the beyond-85° candidate (100 px from the projection) is inside the window.
    const size_t added = SearchFromLocalMap<TestExtractor>(current, kIdentity, replica, matches,
                                                           50.0, ProjectionMatcherOptions{});
    EXPECT_EQ(added, 1u);
    EXPECT_EQ(matches[0], 0);
    EXPECT_EQ(matches[1], -1);
    EXPECT_EQ(matches[2], -1);
    EXPECT_EQ(matches[3], -1);
    EXPECT_EQ(matches[4], 4);
    EXPECT_EQ(matches[5], -1);
    EXPECT_EQ(CountMatches(matches), 2u);
}

TEST(ProjectionMatcherTest, ReportsVisiblePoints) {
    TReplica replica;
    const std::vector<Point> points = {
        {{0.0, 0.0, 5.0}, MakeDescriptor(1)},    // visible, matched now
        {{0.0, 0.0, -5.0}, MakeDescriptor(2)},   // behind the camera
        {{1.0, 0.0, 5.0}, MakeDescriptor(3),     // cos(view, normal) = 0.45
         Eigen::AngleAxisd(std::acos(0.45), Eigen::Vector3d::UnitY()) *
             Eigen::Vector3d(1.0, 0.0, 5.0).normalized()},
        {{-1.0, 0.0, 5.0}, MakeDescriptor(4)},   // already matched
        {{0.0, 1.0, 5.0}, MakeDescriptor(5)},    // visible, no candidate
        {{0.0, -1.0, 5.0}, MakeDescriptor(6)},   // visible, but removed
        {{0.0, -0.5, 5.0}, MakeDescriptor(7)},   // in view, no reference
    };
    AddPoints(replica, points);
    CreateKeyFrame(replica, 0, Eigen::Vector3d::Zero(), {0, 0, 0, 0, 0, 0}, {0, 1, 2, 3, 4, 5});
    RemovePoint(replica, 5);

    const Measurement current = MakeMeasurement({
        {Project(points[0].position), MakeDescriptor(1), 0},
        {Project(points[2].position), MakeDescriptor(3), 0},
        {Project(points[3].position), MakeDescriptor(4), 0},
    });
    std::vector<int64_t> matches(static_cast<size_t>(current.points().cols()), -1);
    matches[2] = 3;
    std::vector<map::MapPointId> visible = {{42}};  // appended to
    const size_t added = SearchFromLocalMap<TestExtractor>(
        current, kIdentity, replica, matches, 1.0, ProjectionMatcherOptions{}, &visible);
    EXPECT_EQ(added, 1u);
    EXPECT_EQ(matches[0], 0);
    EXPECT_EQ(visible, (std::vector<map::MapPointId>{{42}, {0}, {3}, {4}}));
}

TEST(ProjectionMatcherTest, LocalMapPredictedLevelAndRatio) {
    TReplica replica;
    const std::vector<Point> points = {
        {{0.0, 0.0, 4.0}, MakeDescriptor(1)},   // A: predicted level 2
        {{1.0, 0.0, 4.0}, MakeDescriptor(2)},   // B: level 0, ambiguous same-level pair
        {{-1.0, 0.0, 4.0}, MakeDescriptor(3)},  // C: level 1, pair on different levels
    };
    AddPoints(replica, points);
    // A's reference is 4 * 1.2^2 away: two levels finer than seen now.
    CreateKeyFrame(replica, 0, Eigen::Vector3d(0.0, 0.0, 4.0 - 4.0 * 1.44), {0}, {0});
    CreateKeyFrame(replica, 1, Eigen::Vector3d::Zero(), {0, 1}, {1, 2});

    const auto a = Project(points[0].position);
    const auto b = Project(points[1].position);
    const auto c = Project(points[2].position);
    const Measurement current = MakeMeasurement({
        {a, MakeDescriptor(1), 0},                                        // exact, wrong level
        {a + geometry::TPoint2DF(1.0, 0.0), MakeDescriptor(1, 10), 2},    // predicted level
        {b, MakeDescriptor(2, 10), 0},
        {b + geometry::TPoint2DF(1.0, 0.0), MakeDescriptor(2, 11), 0},
        {c, MakeDescriptor(3, 10), 1},
        {c + geometry::TPoint2DF(1.0, 0.0), MakeDescriptor(3, 11), 0},
    });
    std::vector<int64_t> matches(static_cast<size_t>(current.points().cols()), -1);
    const size_t added = SearchFromLocalMap<TestExtractor>(current, kIdentity, replica, matches,
                                                           1.0, ProjectionMatcherOptions{});
    EXPECT_EQ(added, 2u);
    EXPECT_EQ(matches[0], -1);
    EXPECT_EQ(matches[1], 0);
    EXPECT_EQ(matches[2], -1);
    EXPECT_EQ(matches[3], -1);
    EXPECT_EQ(matches[4], 2);
    EXPECT_EQ(matches[5], -1);
}

// SearchFromFrame (relocalization's passes): one Frame observing the point at
// (0, 0, 5) with its feature 0, at level 0 from the origin, so the predicted
// level is 0 and the radius th pixels.
class SearchFromFrameTest : public ::testing::Test {
protected:
    SearchFromFrameTest() {
        AddPoints(replica_, {{{0.0, 0.0, 5.0}, MakeDescriptor(1)}});
        CreateKeyFrame(replica_, 0, Eigen::Vector3d::Zero(), {0}, {0});
    }

    // Matches `current` with th = 10 and max_distance = 100 (or the given).
    std::vector<int64_t> Search(const Measurement& current, double max_distance = 100.0) const {
        std::vector<int64_t> matches(static_cast<size_t>(current.points().cols()), -1);
        const std::vector<std::pair<uint32_t, map::MapPointId>> observations = {{0, {0}}};
        SearchFromFrame<TestExtractor>(current, kIdentity, frame_, observations, replica_, matches, 10.0,
                                       max_distance, ProjectionMatcherOptions{});
        return matches;
    }

    TReplica replica_;
    const Measurement frame_ = MakeLastFrame({0});
    const geometry::TPoint2DF projection_ = Project({0.0, 0.0, 5.0});
};

TEST_F(SearchFromFrameTest, RadiusIsThPixelsAtLevelZero) {
    // 9 px away: inside th = 10 (SearchFromLocalMap's window would be 25 px).
    std::vector<int64_t> matches =
        Search(MakeMeasurement({{projection_ + geometry::TPoint2DF(9.0, 0.0), MakeDescriptor(1), 0}}));
    EXPECT_EQ(matches[0], 0);
    EXPECT_EQ(CountMatches(matches), 1u);

    matches = Search(MakeMeasurement({{projection_ + geometry::TPoint2DF(11.0, 0.0), MakeDescriptor(1), 0}}));
    EXPECT_EQ(CountMatches(matches), 0u);
}

TEST_F(SearchFromFrameTest, NoRatioTest) {
    // Best 10, second 11 on the same level: SearchFromLocalMap's ratio test
    // (10 < 0.8 x 11 fails) would reject it; SearchFromFrame takes the best.
    const Measurement current = MakeMeasurement({
        {projection_ + geometry::TPoint2DF(1.0, 0.0), MakeDescriptor(1, 11), 0},
        {projection_ + geometry::TPoint2DF(-1.0, 0.0), MakeDescriptor(1, 10), 0},
    });
    std::vector<int64_t> matches = Search(current);
    EXPECT_EQ(matches[0], -1);
    EXPECT_EQ(matches[1], 0);

    std::vector<int64_t> local(static_cast<size_t>(current.points().cols()), -1);
    EXPECT_EQ(SearchFromLocalMap<TestExtractor>(current, kIdentity, replica_, local, 4.0,
                                                ProjectionMatcherOptions{}),
              0u);

    // The best must be within max_distance.
    EXPECT_EQ(CountMatches(Search(current, 10.0)), 1u);
    EXPECT_EQ(CountMatches(Search(current, 9.0)), 0u);
}

TEST_F(SearchFromFrameTest, SkipsAlreadyMatchedPointsAndFeatures) {
    const Measurement current = MakeMeasurement({
        {projection_, MakeDescriptor(1), 0},
        {projection_ + geometry::TPoint2DF(1.0, 0.0), MakeDescriptor(1, 1), 0},
    });
    std::vector<int64_t> matches(static_cast<size_t>(current.points().cols()), -1);
    const std::vector<std::pair<uint32_t, map::MapPointId>> observations = {{0, {0}}};
    // The exact feature is taken by another point: the point goes to the next.
    matches[0] = 7;
    EXPECT_EQ(SearchFromFrame<TestExtractor>(current, kIdentity, frame_, observations, replica_, matches, 10.0,
                                             100.0, ProjectionMatcherOptions{}),
              1u);
    EXPECT_EQ(matches[1], 0);
    // Point 0 already matched: nothing added.
    EXPECT_EQ(SearchFromFrame<TestExtractor>(current, kIdentity, frame_, observations, replica_, matches, 10.0,
                                             100.0, ProjectionMatcherOptions{}),
              0u);
}

TEST(ProjectionMatcherTest, FisheyePeripheralPointsMatch) {
    // TUM-VI KB camera, points 60-80 degrees off axis, keypoints 1.8 raw px
    // from the true projection: inside the 2.5 px window in raw pixels, but
    // the undistortion magnifies the offset beyond it.
    const auto model = TumViModel();
    const geometry::Camera camera{TumViIntrinsics(), model};
    constexpr double kDeg = std::numbers::pi / 180.0;
    constexpr int kNumPoints = 8;
    TReplica replica;
    std::vector<Point> points;
    std::vector<Feature> features;
    for (int i = 0; i < kNumPoints; ++i) {
        const double theta = (60.0 + 20.0 * i / (kNumPoints - 1)) * kDeg;
        const double phi = 2.0 * std::numbers::pi * i / kNumPoints + 0.3;
        const Eigen::Vector3d ray(std::sin(theta) * std::cos(phi), std::sin(theta) * std::sin(phi),
                                  std::cos(theta));
        points.push_back({3.0 * ray, MakeDescriptor(i + 1)});
        const double alpha = 1.1 * i;
        const geometry::TPoint2DF offset(1.8 * std::cos(alpha), 1.8 * std::sin(alpha));
        features.push_back({camera.Project(points.back().position) + offset, MakeDescriptor(i + 1, 3), 0});
    }
    AddPoints(replica, points);
    std::vector<uint32_t> ids(kNumPoints);
    for (int i = 0; i < kNumPoints; ++i) ids[static_cast<size_t>(i)] = static_cast<uint32_t>(i);
    CreateKeyFrame(replica, 0, Eigen::Vector3d::Zero(), std::vector<int>(kNumPoints, 0), ids);

    // Stretch the raw bounding box with 84 degree rays along both axes.
    const double sin84 = std::sin(84.0 * kDeg);
    const double cos84 = std::cos(84.0 * kDeg);
    const std::vector<geometry::TPoint2DF> stretch = {
        camera.Project({sin84, 0.0, cos84}), camera.Project({-sin84, 0.0, cos84}),
        camera.Project({0.0, sin84, cos84}), camera.Project({0.0, -sin84, cos84})};
    const Measurement current = MakeMeasurement(features, stretch, TumViIntrinsics(), model);

    // Scene check: in undistorted px every keypoint is outside the 2.5 px window.
    constexpr double kWindow = 2.5;  // th = 1, cos > 0.998, level 0; radius_px below
    for (int i = 0; i < kNumPoints; ++i) {
        const geometry::TPoint2DF pinhole =
            (TumViIntrinsics() * points[static_cast<size_t>(i)].position).hnormalized();
        ASSERT_GT((current.points().col(i) - pinhole).cwiseAbs().maxCoeff(), kWindow) << "point " << i;
        ASSERT_LE((current.distorted_points().col(i) - camera.Project(points[static_cast<size_t>(i)].position))
                      .norm(),
                  2.0);
    }

    std::vector<int64_t> from_map(static_cast<size_t>(current.points().cols()), -1);
    const size_t added = SearchFromLocalMap<TestExtractor>(current, kIdentity, replica, from_map, 1.0,
                                                           ProjectionMatcherOptions{});
    EXPECT_EQ(added, static_cast<size_t>(kNumPoints));
    for (int i = 0; i < kNumPoints; ++i) EXPECT_EQ(from_map[static_cast<size_t>(i)], i) << "point " << i;

    const Measurement last = MakeLastFrame(std::vector<int>(kNumPoints, 0));
    const std::vector<int64_t> from_last = SearchFromLastFrame<TestExtractor>(
        current, kIdentity, last, IdentityMatches(kNumPoints + 2, kNumPoints), replica, kWindow,
        ProjectionMatcherOptions{});
    EXPECT_EQ(CountMatches(from_last), static_cast<size_t>(kNumPoints));
    for (int i = 0; i < kNumPoints; ++i) EXPECT_EQ(from_last[static_cast<size_t>(i)], i) << "point " << i;
}

// ---- Synthetic sequence through a real Mapper ----

using TMonocular = features::TMonocularMeasurement<TestExtractor>;

struct SyntheticMap {
    test::SyntheticSequence sequence = test::MakeSyntheticSequence();
    TReplica replica;
    // Sequence frame index of each keyframe, in creation order.
    std::vector<size_t> keyframe_frames;
    std::vector<map::FrameId> keyframe_ids;
    // Map point id -> scene point index (-1 unknown).
    std::vector<Eigen::Index> scene_of_point;
    // Estimated / true translation of the second keyframe relative to the first.
    double scale = 1.0;

    // True T_cw of sequence frame k in the map's world frame (the first
    // keyframe's camera, scaled).
    geometry::RigidTransformation PoseInMap(size_t k) const {
        const geometry::RigidTransformation relative =
            sequence.frames[k].T_cw * sequence.frames[keyframe_frames[0]].T_cw.Inverse();
        return geometry::RigidTransformation(relative.rotation(), scale * relative.translation());
    }
};

SyntheticMap BuildSyntheticMap() {
    SyntheticMap result;
    // Declaration order as in MapperTest: the subscriptions are declared before
    // the components they connect.
    std::vector<events::Subscription> subscriptions;
    events::Observable tracker;
    mapping::Mapper<TestExtractor> mapper;
    events::AsyncObserver capture;
    events::Listener listener = capture.GetListener();
    subscriptions.push_back(tracker.ConnectAsync(mapper.inbox()));
    subscriptions.push_back(mapper.AddObserver(capture));

    const auto& frames = result.sequence.frames;
    for (size_t i = 0; i < frames.size() && mapper.state() == mapping::MapperState::Initializing;
         ++i) {
        auto event = std::make_shared<events::InitializationMeasurement<TestExtractor>>();
        event->map_id = map::MapId{0};
        event->measurement = frames[i].measurement;
        tracker.Notify(std::move(event));
        while (mapper.ProcessPending()) {
        }
    }
    EXPECT_EQ(mapper.state(), mapping::MapperState::Mapping);

    std::map<map::FrameId, geometry::RigidTransformation> poses;
    while (events::EventPtr event = listener.GetNextEvent()) {
        result.replica.Apply(*event);
        if (event->type != events::EventType::MapTransaction) continue;
        result.scene_of_point.resize(result.replica.num_points(), -1);
        for (const auto& created : static_cast<const TTransaction&>(*event).added_frames) {
            const auto& measurement = std::get<TMonocular>(*created.measurement);
            const auto k = static_cast<size_t>(std::lround(
                measurement.timestamp() / test::SyntheticSequenceOptions{}.frame_interval));
            EXPECT_EQ(measurement.points().cols(), frames[k].measurement.points().cols());
            result.keyframe_frames.push_back(k);
            result.keyframe_ids.push_back(created.id);
            poses[created.id] = created.T_cw;
            for (size_t o = 0; o < created.observed_features.size(); ++o) {
                const Eigen::Index scene = frames[k].point_of_feature[created.observed_features[o]];
                Eigen::Index& known = result.scene_of_point[created.observed_points[o].value];
                if (known < 0) known = scene;
            }
        }
    }
    EXPECT_EQ(result.keyframe_frames.size(), 2u);
    if (result.keyframe_frames.size() >= 2) {
        result.scale = 1.0;
        const double true_norm = result.PoseInMap(result.keyframe_frames[1]).translation().norm();
        result.scale = poses[result.keyframe_ids[1]].translation().norm() / true_norm;
    }
    return result;
}

// Fraction of matches whose map point is the scene point of the feature.
std::pair<size_t, size_t> CountCorrect(const SyntheticMap& map, size_t frame,
                                       const std::vector<int64_t>& matches) {
    size_t correct = 0;
    size_t total = 0;
    for (size_t i = 0; i < matches.size(); ++i) {
        if (matches[i] < 0) continue;
        ++total;
        const Eigen::Index scene = map.scene_of_point[static_cast<size_t>(matches[i])];
        if (scene >= 0 && scene == map.sequence.frames[frame].point_of_feature[i]) ++correct;
    }
    return {correct, total};
}

TEST(ProjectionMatcherTest, SyntheticPrecision) {
    const SyntheticMap map = BuildSyntheticMap();
    ASSERT_EQ(map.keyframe_frames.size(), 2u);
    const size_t second = map.keyframe_frames[1];
    ASSERT_LT(second + 6, map.sequence.frames.size());

    // Frame to frame: the second keyframe's observations are the last frame's matches.
    const TReplica& replica = map.replica;
    const auto& last_keyframe = *replica.keyframe(map.keyframe_ids[1]);
    const TMonocular& last = map.sequence.frames[second].measurement;
    std::vector<int64_t> last_matches(static_cast<size_t>(last.points().cols()), -1);
    for (const auto& [feature, point] : last_keyframe.observations) last_matches[feature] = point.value;
    const size_t num_last = CountMatches(last_matches);

    const size_t next = second + 1;
    const std::vector<int64_t> from_last = SearchFromLastFrame<TestExtractor>(
        map.sequence.frames[next].measurement, map.PoseInMap(next), last, last_matches, replica,
        15.0, ProjectionMatcherOptions{});
    const auto [correct_last, total_last] = CountCorrect(map, next, from_last);
    std::cout << "SearchFromLastFrame: " << correct_last << " / " << total_last
              << " correct (last frame had " << num_last << " matches)\n";
    EXPECT_GE(total_last, num_last * 8 / 10);
    EXPECT_GE(static_cast<double>(correct_last), 0.98 * static_cast<double>(total_last));

    // Local map on a later frame, from scratch.
    const size_t later = second + 5;
    const TMonocular& current = map.sequence.frames[later].measurement;
    std::vector<int64_t> from_map(static_cast<size_t>(current.points().cols()), -1);
    const size_t added = SearchFromLocalMap<TestExtractor>(current, map.PoseInMap(later), replica,
                                                           from_map, 1.0, ProjectionMatcherOptions{});
    const auto [correct_map, total_map] = CountCorrect(map, later, from_map);
    std::cout << "SearchFromLocalMap: " << correct_map << " / " << total_map << " correct ("
              << replica.num_points() << " map points)\n";
    EXPECT_EQ(added, total_map);
    EXPECT_GE(total_map, replica.num_points() * 7 / 10);
    EXPECT_GE(static_cast<double>(correct_map), 0.95 * static_cast<double>(total_map));
}

// SearchBySim3Projection: Frame 2's camera coordinates map to Frame 1's by S_12 (a 0.05 rad
// yaw, a 0.3 m shift, scale 1); Frame 1 is at the world origin, Frame 2 1 m along z. Feature 0
// of each Frame observes a point about 5 m away (predicted level 0, so the radius is th =
// 10 px): Frame 1's point projects d_12 px right of Frame 2's feature, Frame 2's point d_21 px
// right of Frame 1's feature. With `rival`, Frame 1's feature 1 (2 px right of feature 0, 20
// bits further from Frame 2's descriptor) observes a point projecting onto the same spot.
std::vector<std::pair<uint32_t, uint32_t>> SearchBySim3(double d_12, double d_21, bool rival = false) {
    const geometry::Sim3 S_12{1.0, geometry::TMatrix33F(Eigen::AngleAxisd(0.05, Eigen::Vector3d::UnitY())),
                              geometry::TVector3DF(0.3, 0.0, 0.0)};
    const geometry::RigidTransformation T2w(geometry::TMatrix33F::Identity(), geometry::TVector3DF(0.0, 0.0, -1.0));
    const geometry::Sim3 S_21 = geometry::Inverse(S_12);
    const Eigen::Vector3d P1(0.2, 0.1, 5.0);  // Frame 1 camera = world
    const geometry::TPoint2DF pixel1 = Project(P1);
    const geometry::TPoint2DF pixel2 = Project(geometry::Apply(S_21, P1)) - geometry::TPoint2DF(d_12, 0.0);
    const geometry::TPoint2DF target1 = pixel1 + geometry::TPoint2DF(d_21, 0.0);
    const Eigen::Vector3d X2 = geometry::Apply(
        S_21, Eigen::Vector3d((target1.x() - kCx) / kFocal * 5.0, (target1.y() - kCy) / kFocal * 5.0, 5.0));

    std::vector<Feature> features1 = {{pixel1, MakeDescriptor(1), 0}};
    std::vector<std::optional<map::MapPointId>> points1 = {map::MapPointId{10}};
    std::vector<geometry::TPoint3DF> positions1 = {P1};
    if (rival) {
        features1.push_back({pixel1 + geometry::TPoint2DF(2.0, 0.0), MakeDescriptor(1, 30), 0});
        points1.push_back(map::MapPointId{11});
        positions1.push_back(P1 + Eigen::Vector3d(0.002, 0.0, 0.0));
    }
    const Measurement frame1 = MakeMeasurement(features1);
    const Measurement frame2 = MakeMeasurement({{pixel2, MakeDescriptor(1, 10), 0}});
    points1.resize(static_cast<size_t>(frame1.points().cols()));  // the stretchers observe nothing
    positions1.resize(points1.size(), geometry::TPoint3DF::Zero());
    std::vector<std::optional<map::MapPointId>> points2(static_cast<size_t>(frame2.points().cols()));
    std::vector<geometry::TPoint3DF> positions2(points2.size(), geometry::TPoint3DF::Zero());
    points2[0] = map::MapPointId{20};
    positions2[0] = T2w.Inverse().Transform(X2);
    return SearchBySim3Projection<TestExtractor>(frame1, kIdentity, points1, positions1, frame2, T2w, points2,
                                                 positions2, S_12, 10.0, 100);
}

TEST(ProjectionMatcherTest, SearchBySim3MutualMatches) {
    using Pairs = std::vector<std::pair<uint32_t, uint32_t>>;
    EXPECT_EQ(SearchBySim3(9.0, 9.0), (Pairs{{0, 0}}));
    // 11 px in either direction: the other direction alone is not a match.
    EXPECT_EQ(SearchBySim3(11.0, 9.0), Pairs{});
    EXPECT_EQ(SearchBySim3(9.0, 11.0), Pairs{});
    // Both Frame-1 features pick Frame 2's feature, which picks feature 0 back: only (0, 0).
    EXPECT_EQ(SearchBySim3(1.0, 1.0, true), (Pairs{{0, 0}}));
}


// SearchPointsBySim3 (loop detection's ORB-SLAM3 SearchByProjection(KF, Scw)):
// the point at (1, 0, 5) observed from the origin at level 0 (reference
// distance ~5.1, so camera distances up to 1.2 x 5.1 pass the scale gate).
class SearchPointsBySim3Test : public ::testing::Test {
protected:
    SearchPointsBySim3Test() {
        AddPoints(replica_, {{kPoint, MakeDescriptor(1)}});
        CreateKeyFrame(replica_, 0, Eigen::Vector3d::Zero(), {0}, {0});
    }

    std::vector<int64_t> Search(const Measurement& current, const geometry::Sim3& S_cw, double th = 8.0,
                                std::vector<int64_t> matches = {}) const {
        if (matches.empty()) matches.assign(static_cast<size_t>(current.points().cols()), -1);
        const std::vector<map::MapPointId> points = {{0}};
        SearchPointsBySim3<TestExtractor>(current, S_cw, points, replica_, matches, th, 75.0,
                                          ProjectionMatcherOptions{});
        return matches;
    }

    const Eigen::Vector3d kPoint{1.0, 0.0, 5.0};
    TReplica replica_;
};

TEST_F(SearchPointsBySim3Test, RadiusIsThPixelsAtLevelZero) {
    const geometry::Sim3 identity;
    const geometry::TPoint2DF uv = Project(kPoint);
    EXPECT_EQ(Search(MakeMeasurement({{uv + geometry::TPoint2DF(7.0, 0.0), MakeDescriptor(1), 0}}), identity)[0], 0);
    EXPECT_EQ(CountMatches(Search(MakeMeasurement({{uv + geometry::TPoint2DF(9.0, 0.0), MakeDescriptor(1), 0}}),
                                  identity)),
              0u);
}

TEST_F(SearchPointsBySim3Test, ScaleIsFoldedIntoTheCamera) {
    // S_cw = 2 [I | (0.4, 0, 0)]: the rigid [I | (0.2, 0, 0)] projects alike,
    // and the camera is ~5.2 from the point in the map's units (in range);
    // unfolded, s R X + t is ~10.4 away, beyond the scale gate.
    const geometry::Sim3 S_cw{2.0, geometry::TMatrix33F::Identity(), geometry::TVector3DF(0.4, 0.0, 0.0)};
    const geometry::TPoint2DF uv = Project(kPoint + Eigen::Vector3d(0.2, 0.0, 0.0));
    EXPECT_EQ(Search(MakeMeasurement({{uv, MakeDescriptor(1), 0}}), S_cw)[0], 0);
    // The same camera at scale 1 sees the point from 2x farther: rejected.
    const geometry::Sim3 far{1.0, geometry::TMatrix33F::Identity(), geometry::TVector3DF(0.2, 0.0, 5.0)};
    const geometry::TPoint2DF far_uv = Project(kPoint + Eigen::Vector3d(0.2, 0.0, 5.0));
    EXPECT_EQ(CountMatches(Search(MakeMeasurement({{far_uv, MakeDescriptor(1), 0}}), far)), 0u);
}

TEST_F(SearchPointsBySim3Test, SkipsMatchedPointsAndFeatures) {
    const geometry::Sim3 identity;
    const geometry::TPoint2DF uv = Project(kPoint);
    const Measurement current = MakeMeasurement({
        {uv, MakeDescriptor(1), 0},
        {uv + geometry::TPoint2DF(1.0, 0.0), MakeDescriptor(1, 1), 0},
    });
    std::vector<int64_t> taken(static_cast<size_t>(current.points().cols()), -1);
    taken[0] = 7;  // the exact feature is another point's: the next one is taken
    EXPECT_EQ(Search(current, identity, 8.0, taken)[1], 0);
    taken[0] = 0;  // the point is already matched: nothing added
    EXPECT_EQ(CountMatches(Search(current, identity, 8.0, taken)), 1u);
}

}  // namespace
}  // namespace voyahslam::features
