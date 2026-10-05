#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <iostream>
#include <map>
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
#include <voyahslam/mapping/mapper.h>
#include <voyahslam/memory/object_pool.h>
#include <voyahslam/tracking/map_replica.h>

namespace voyahslam::features {
namespace {

using voyahslam::test::TestExtractor;
using voyahslam::test::TestImageFeatures;
using Measurement = TMonocularMeasurement<TestExtractor>;
using Descriptor = TestImageFeatures::TDescriptor;
using TReplica = tracking::MapReplica<TestExtractor>;
using TAdded = events::MapPointsAdded<TestExtractor>;
using TKeyFrameCreated = events::KeyFrameCreated<TestExtractor>;

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

// Features in order, then two stretchers so the point bounding box spans
// [0, 640] x [0, 480] (or `stretch` when given).
Measurement MakeMeasurement(std::vector<Feature> features,
                            const std::vector<geometry::TPoint2DF>& stretch = {{0.0, 0.0},
                                                                               {640.0, 480.0}}) {
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
                            Intrinsics(), nullptr, 0.0);
    EXPECT_EQ(measurement.points().cols(), n);  // nothing dropped by the 85° cut
    return measurement;
}

struct Point {
    Eigen::Vector3d position;
    Descriptor descriptor;
    Eigen::Vector3d normal = Eigen::Vector3d::Zero();  // zero: the view direction from the origin
};

void AddPoints(TReplica& out_replica, const std::vector<Point>& points) {
    memory::ObjectPool<TAdded, memory::ThreadSafePolicy> pool(1);
    TAdded* event = pool.Create();
    event->map_id = {1};
    event->start_id = {static_cast<uint32_t>(out_replica.num_points())};
    event->Resize(static_cast<uint32_t>(points.size()));
    for (size_t i = 0; i < points.size(); ++i) {
        const auto col = static_cast<Eigen::Index>(i);
        event->positions.col(col) = points[i].position;
        event->descriptors.col(col) = points[i].descriptor;
        event->normals.col(col) =
            points[i].normal.isZero() ? points[i].position.normalized() : points[i].normal;
    }
    out_replica.Apply(*event);
    event->Recycle();
}

void RemovePoint(TReplica& out_replica, uint32_t id) {
    memory::ObjectPool<events::MapPointsRemoved, memory::ThreadSafePolicy> pool(1);
    events::MapPointsRemoved* event = pool.Create();
    event->map_id = {1};
    event->ids = {{id}};
    out_replica.Apply(*event);
    event->Recycle();
}

// A keyframe with centre `centre` (identity rotation) whose feature i, at
// level levels[i], observes point points[i]: gives those points their reference.
void CreateKeyFrame(TReplica& out_replica, uint32_t frame, const Eigen::Vector3d& centre,
                    const std::vector<int>& levels, const std::vector<uint32_t>& points) {
    memory::ObjectPool<TKeyFrameCreated, memory::ThreadSafePolicy> pool(1);
    TKeyFrameCreated* event = pool.Create();
    event->Reset();
    event->map_id = {1};
    event->frame_id = {frame};
    event->pose = geometry::RigidTransformation(geometry::TMatrix33F::Identity(), -centre);
    std::vector<Feature> features;
    for (const int level : levels) features.push_back({{kCx, kCy}, MakeDescriptor(9000), level, 0.0});
    event->measurement = MakeMeasurement(features, {});
    for (size_t i = 0; i < points.size(); ++i) {
        event->observed_features.push_back(static_cast<uint32_t>(i));
        event->observed_points.push_back({points[i]});
    }
    out_replica.Apply(*event);
    event->Recycle();
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
    // Declaration order as in MapperTest: the pool outlives the Mapper.
    memory::ObjectPool<events::InitializationMeasurement<TestExtractor>, memory::ThreadSafePolicy>
        pool(256);
    events::Observable tracker;
    mapping::Mapper<TestExtractor> mapper;
    events::AsyncObserver capture;
    events::Listener listener = capture.GetListener();
    tracker.ConnectAsync(mapper.inbox());
    mapper.AddObserver(capture);

    const auto& frames = result.sequence.frames;
    for (size_t i = 0; i < frames.size() && mapper.state() == mapping::MapperState::Initializing;
         ++i) {
        auto* event = pool.Create();
        event->map_id = map::MapId{0};
        event->measurement = frames[i].measurement;
        tracker.Notify(event);
        while (mapper.ProcessPending()) {
        }
    }
    EXPECT_EQ(mapper.state(), mapping::MapperState::Mapping);

    std::map<map::FrameId, geometry::RigidTransformation> poses;
    while (events::Event* event = listener.GetNextEvent()) {
        result.replica.Apply(*event);
        if (event->type != events::EventType::KeyFrameCreated) continue;
        const auto* created = static_cast<const TKeyFrameCreated*>(event);
        const auto& measurement = std::get<TMonocular>(created->measurement);
        const auto k = static_cast<size_t>(
            std::lround(measurement.timestamp() / test::SyntheticSequenceOptions{}.frame_interval));
        EXPECT_EQ(measurement.points().cols(), frames[k].measurement.points().cols());
        result.keyframe_frames.push_back(k);
        result.keyframe_ids.push_back(created->frame_id);
        poses[created->frame_id] = created->pose;
        result.scene_of_point.resize(result.replica.num_points(), -1);
        for (size_t o = 0; o < created->observed_features.size(); ++o) {
            const Eigen::Index scene = frames[k].point_of_feature[created->observed_features[o]];
            Eigen::Index& known = result.scene_of_point[created->observed_points[o].value];
            if (known < 0) known = scene;
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

}  // namespace
}  // namespace voyahslam::features
