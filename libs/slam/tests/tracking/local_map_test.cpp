#include <gtest/gtest.h>

#include <cstdint>
#include <memory>
#include <optional>
#include <utility>
#include <vector>
#include <voyahslam/events/events.h>
#include <voyahslam/features/orb_feature_extractor.h>
#include <voyahslam/memory/object_pool.h>
#include <voyahslam/tracking/local_map.h>
#include <voyahslam/tracking/map_replica.h>

namespace voyahslam::tracking {
namespace {

using Extractor = features::OrbFeatureExtractor;
using TReplica = MapReplica<Extractor>;
using TTransaction = events::MapTransaction<Extractor>;
using TMono = features::TMonocularMeasurement<Extractor>;

constexpr uint32_t kMap = 1;

// Mono measurement with `n` level-0 features.
TMono MakeMeasurement(int n) {
    geometry::TPoints2DF points = geometry::TPoints2DF::Zero(2, n);
    for (int i = 0; i < n; ++i) points.col(i) = Eigen::Vector2d(300 + i, 240).cast<precision_t>();
    using TFeatures =
        features::ImageFeatures<Extractor::TDescriptorScalarType, Extractor::DescriptorSize>;
    TFeatures features(std::move(points), geometry::TRowVectorF::Zero(n),
                       geometry::TRowVectorF::Zero(n),
                       TFeatures::TDescriptors::Zero(Extractor::DescriptorSize, n),
                       features::TLevels::Zero(n));
    geometry::TMatrix33F K = geometry::TMatrix33F::Identity();
    K(0, 0) = K(1, 1) = 500;
    K(0, 2) = 320;
    K(1, 2) = 240;
    return TMono(std::move(features), K, nullptr, 1.0);
}

// Applies a transaction that adds Frame `frame`, whose feature i observes
// points[i], and `num_new` points from id `start`.
void CreateFrame(TReplica& out_replica, uint32_t frame, const std::vector<uint32_t>& points,
                 uint32_t num_new = 0, uint32_t start = 0) {
    memory::ObjectPool<TTransaction, memory::ThreadSafePolicy> pool(1);
    TTransaction* event = pool.Create();
    event->map_id = {kMap};
    events::NewFrame<Extractor>& added = event->added_frames.emplace_back();
    added.id = {frame};
    added.T_cw = geometry::RigidTransformation::Identity();
    added.measurement =
        std::make_shared<const features::TMeasurement<Extractor>>(MakeMeasurement(static_cast<int>(points.size())));
    for (size_t i = 0; i < points.size(); ++i) {
        added.observed_features.push_back(static_cast<uint32_t>(i));
        added.observed_points.push_back({points[i]});
    }
    event->new_points_start_id = {start};
    event->ResizeNewPoints(num_new);
    for (uint32_t i = 0; i < num_new; ++i) {
        event->new_positions.col(i) = Eigen::Vector3d(i, 0, 5).cast<precision_t>();
        event->new_descriptors.col(i).setConstant(static_cast<uint8_t>(i));
        event->new_normals.col(i) = Eigen::Vector3d(0, 0, 1).cast<precision_t>();
    }
    out_replica.Apply(*event);
    event->Recycle();
}

void RemovePoint(TReplica& out_replica, uint32_t id) {
    memory::ObjectPool<TTransaction, memory::ThreadSafePolicy> pool(1);
    TTransaction* event = pool.Create();
    event->map_id = {kMap};
    event->removed_points = {{id}};
    out_replica.Apply(*event);
    event->Recycle();
}

std::vector<uint32_t> Range(uint32_t begin, uint32_t end) {
    std::vector<uint32_t> ids;
    for (uint32_t i = begin; i < end; ++i) ids.push_back(i);
    return ids;
}

std::vector<map::FrameId> Frames(const std::vector<uint32_t>& ids) {
    std::vector<map::FrameId> frames;
    for (const uint32_t id : ids) frames.push_back({id});
    return frames;
}

// Per feature, the given point ids; one feature per id.
std::vector<int64_t> Matches(const std::vector<uint32_t>& ids) {
    std::vector<int64_t> matches = {-1};
    for (const uint32_t id : ids) matches.push_back(id);
    return matches;
}

TEST(LocalMapTest, K1AndK2) {
    // F1: p0-p9. F2: p0-p4 (5 shared with F1), p10. F3: p5, p6 (2 shared
    // with F1), p11, p12. F4: p12 (shared with F3 only), p13. F5: p14.
    TReplica replica;
    CreateFrame(replica, 1, Range(0, 10), 15, 0);
    CreateFrame(replica, 2, {0, 1, 2, 3, 4, 10});
    CreateFrame(replica, 3, {5, 6, 11, 12});
    CreateFrame(replica, 4, {12, 13});
    CreateFrame(replica, 5, {14});
    RemovePoint(replica, 3);
    // Matched points only F1 observes, plus a removed one and a stray -1.
    const std::vector<int64_t> matches = Matches({7, 8, 9, 3});

    const LocalMap local = BuildLocalMap(replica, matches, 1, 80);
    EXPECT_FALSE(local.all_points);
    EXPECT_EQ(local.frames, Frames({1, 2}));
    ASSERT_EQ(local.point_mask.size(), replica.num_points());
    for (uint32_t p = 0; p < replica.num_points(); ++p) {
        EXPECT_EQ(local.point_mask[p], p <= 10 && p != 3) << "point " << p;
    }
    const LocalPointSource<Extractor> source(replica, local);
    EXPECT_EQ(source.num_points(), replica.num_points());
    EXPECT_TRUE(source.IsAlive({10}));
    EXPECT_FALSE(source.IsAlive({3}));
    EXPECT_FALSE(source.IsAlive({11}));
    EXPECT_FALSE(source.IsAlive({99}));
    EXPECT_TRUE(source.Position({10}).isApprox(replica.Position({10})));

    // Ten neighbours: F3 joins; F4 is only F3's neighbour.
    EXPECT_EQ(BuildLocalMap(replica, matches, 10, 80).frames, Frames({1, 2, 3}));
}

TEST(LocalMapTest, K1OrderedByMatchCount) {
    // F1: p0. F2: p1, p2. F3: p3, p4 (tie with F2, larger id).
    TReplica replica;
    CreateFrame(replica, 1, {0}, 5, 0);
    CreateFrame(replica, 2, {1, 2});
    CreateFrame(replica, 3, {3, 4});
    EXPECT_EQ(BuildLocalMap(replica, Matches({0, 1, 2, 3, 4}), 10, 80).frames,
              Frames({2, 3, 1}));
    EXPECT_EQ(BuildLocalMap(replica, Matches({0, 1, 2, 3, 4}), 10, 2).frames, Frames({2, 3}));
}

TEST(LocalMapTest, CapLimitsFrames) {
    TReplica replica;
    CreateFrame(replica, 1, {0}, 1, 0);
    for (uint32_t frame = 2; frame <= 100; ++frame) CreateFrame(replica, frame, {0});

    const LocalMap local = BuildLocalMap(replica, Matches({0}), 10, 80);
    EXPECT_EQ(local.frames, Frames(Range(1, 81)));
    ASSERT_EQ(local.point_mask.size(), 1u);
    EXPECT_TRUE(local.point_mask[0]);
}

TEST(LocalMapTest, UnknownFramesFallBackToAllPoints) {
    // The points come from a Frame that observes none of them: no matched
    // point has a known observing Frame.
    TReplica replica;
    CreateFrame(replica, 1, {}, 3, 0);
    RemovePoint(replica, 2);
    const std::vector<int64_t> matches = Matches({0, 1});

    const LocalMap local = BuildLocalMap(replica, matches, 10, 80);
    EXPECT_TRUE(local.all_points);
    EXPECT_TRUE(local.frames.empty());
    const LocalPointSource<Extractor> source(replica, local);
    EXPECT_TRUE(source.IsAlive({0}));
    EXPECT_TRUE(source.IsAlive({1}));
    EXPECT_FALSE(source.IsAlive({2}));
    EXPECT_FALSE(ReferenceFrame(replica, matches).has_value());

    // No matches at all: the same fallback.
    EXPECT_TRUE(BuildLocalMap(replica, {}, 10, 80).all_points);
}

TEST(LocalMapTest, ReferenceFrameHasMostInliers) {
    // F1: p0, p1. F2: p1, p2, p3. F3: p4, p5, p6, p7 (no inlier).
    TReplica replica;
    CreateFrame(replica, 1, {0, 1}, 8, 0);
    CreateFrame(replica, 2, {1, 2, 3});
    CreateFrame(replica, 3, {4, 5, 6, 7});

    EXPECT_EQ(ReferenceFrame(replica, Matches({0, 1, 2, 3})), std::optional(map::FrameId{2}));
    // A tie goes to the smaller id.
    EXPECT_EQ(ReferenceFrame(replica, Matches({0, 1, 2})), std::optional(map::FrameId{1}));
    // Removed points are not inliers.
    RemovePoint(replica, 2);
    RemovePoint(replica, 3);
    EXPECT_EQ(ReferenceFrame(replica, Matches({0, 1, 2, 3})), std::optional(map::FrameId{1}));
}

} // namespace
} // namespace voyahslam::tracking
