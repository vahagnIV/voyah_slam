#include <gtest/gtest.h>

#include <Eigen/Geometry>
#include <algorithm>
#include <utility>
#include <vector>
#include <voyahslam/events/events.h>
#include <voyahslam/features/orb_feature_extractor.h>
#include <voyahslam/memory/object_pool.h>
#include <voyahslam/tracking/map_replica.h>

namespace voyahslam::tracking {
namespace {

using Extractor = features::OrbFeatureExtractor;
using TReplica = MapReplica<Extractor>;
using TUpdated = events::MapPointsUpdated<Extractor>;
using TKeyFrameCreated = events::KeyFrameCreated<Extractor>;
using TMono = features::TMonocularMeasurement<Extractor>;

geometry::RigidTransformation MakePose(double angle, double tx, double ty, double tz) {
    const Eigen::Matrix3d rotation =
        Eigen::AngleAxisd(angle, Eigen::Vector3d::UnitY()).toRotationMatrix();
    return geometry::RigidTransformation(rotation.cast<precision_t>(),
                                         Eigen::Vector3d(tx, ty, tz).cast<precision_t>());
}

// Sets the new-points part of `out_event`: ids [start, start + points.size()),
// point i with position points[i], a descriptor filled with i + 1 and normal (0, 0, i + 1).
void SetNewPoints(TKeyFrameCreated& out_event, uint32_t start,
                  const std::vector<Eigen::Vector3d>& points) {
    out_event.new_points_start_id = {start};
    out_event.ResizeNewPoints(static_cast<uint32_t>(points.size()));
    for (size_t i = 0; i < points.size(); ++i) {
        const auto col = static_cast<Eigen::Index>(i);
        out_event.new_positions.col(col) = points[i].cast<precision_t>();
        out_event.new_descriptors.col(col).setConstant(static_cast<uint8_t>(i + 1));
        out_event.new_normals.col(col) =
            Eigen::Vector3d(0, 0, static_cast<double>(i + 1)).cast<precision_t>();
    }
}

// Mono measurement with `levels.size()` features.
TMono MakeMeasurement(const std::vector<uint8_t>& levels) {
    const int n = static_cast<int>(levels.size());
    geometry::TPoints2DF points = geometry::TPoints2DF::Zero(2, n);
    for (int i = 0; i < n; ++i) points.col(i) = Eigen::Vector2d(300 + i, 240).cast<precision_t>();
    features::TLevels level_row(n);
    for (int i = 0; i < n; ++i) level_row(i) = levels[static_cast<size_t>(i)];
    features::ImageFeatures<Extractor::TDescriptorScalarType, Extractor::DescriptorSize> features(
        std::move(points), geometry::TRowVectorF::Zero(n), geometry::TRowVectorF::Zero(n),
        features::ImageFeatures<Extractor::TDescriptorScalarType,
                                Extractor::DescriptorSize>::TDescriptors::Zero(
            Extractor::DescriptorSize, n),
        std::move(level_row));
    geometry::TMatrix33F K = geometry::TMatrix33F::Identity();
    K(0, 0) = K(1, 1) = 500;
    K(0, 2) = 320;
    K(1, 2) = 240;
    return TMono(std::move(features), K, nullptr, 1.0);
}

// Sets the Frame part of `out_event`: feature i at level levels[i], and
// (feature, point) observations.
void SetFrame(TKeyFrameCreated& out_event, uint32_t map, uint32_t frame,
              const geometry::RigidTransformation& pose, const std::vector<uint8_t>& levels,
              const std::vector<std::pair<uint32_t, uint32_t>>& observations) {
    out_event.map_id = {map};
    out_event.frame_id = {frame};
    out_event.pose = pose;
    out_event.measurement = MakeMeasurement(levels);
    for (const auto& [feature, point] : observations) {
        out_event.observed_features.push_back(feature);
        out_event.observed_points.push_back({point});
    }
}

// Applies a KeyFrameCreated of Frame `frame` that also adds `new_points` from id `start`.
void CreateKeyFrame(TReplica& out_replica, uint32_t map, uint32_t frame,
                    const geometry::RigidTransformation& pose, const std::vector<uint8_t>& levels,
                    const std::vector<std::pair<uint32_t, uint32_t>>& observations,
                    const std::vector<Eigen::Vector3d>& new_points = {}, uint32_t start = 0) {
    memory::ObjectPool<TKeyFrameCreated, memory::ThreadSafePolicy> pool(1);
    TKeyFrameCreated* event = pool.Create();
    SetFrame(*event, map, frame, pose, levels, observations);
    SetNewPoints(*event, start, new_points);
    out_replica.Apply(*event);
    event->Recycle();
}

// Adds points through a KeyFrameCreated whose Frame (`frame`, no features)
// observes none of them, so they get no reference.
void AddPoints(TReplica& out_replica, uint32_t map, uint32_t start,
               const std::vector<Eigen::Vector3d>& points, uint32_t frame = 100) {
    CreateKeyFrame(out_replica, map, frame, geometry::RigidTransformation::Identity(), {}, {},
                   points, start);
}

const std::vector<Eigen::Vector3d> kThreePoints = {{1, 2, 3}, {4, 5, 6}, {7, 8, 9}};

TEST(MapReplicaTest, PointsAddedUpdatedRemoved) {
    TReplica replica;
    AddPoints(replica, 1, 0, kThreePoints);
    ASSERT_EQ(replica.num_points(), 3u);
    EXPECT_EQ(replica.map_id().value, 1u);
    EXPECT_TRUE(replica.IsAlive({2}));
    EXPECT_TRUE(replica.Position({2}).isApprox(kThreePoints[2].cast<precision_t>()));
    EXPECT_EQ(replica.Descriptor({1})(5), 2);
    EXPECT_NEAR(replica.Normal({1}).z(), 2.0, 1e-6);

    memory::ObjectPool<TUpdated, memory::ThreadSafePolicy> updated(1);
    TUpdated* up = updated.Create();
    up->map_id = {1};
    up->Resize(2);
    up->ids[0] = {1};
    up->ids[1] = {7};
    up->positions.col(0) = Eigen::Vector3d(10, 20, 30).cast<precision_t>();
    up->positions.col(1) = Eigen::Vector3d(-1, -1, -1).cast<precision_t>();
    up->descriptors.col(0).setConstant(77);
    up->descriptors.col(1).setConstant(88);
    up->normals.col(0) = Eigen::Vector3d(0, 1, 0).cast<precision_t>();
    up->normals.col(1) = Eigen::Vector3d(0, 1, 0).cast<precision_t>();
    replica.Apply(*up);
    up->Recycle();
    EXPECT_TRUE(replica.Position({1}).isApprox(Eigen::Vector3d(10, 20, 30).cast<precision_t>()));
    EXPECT_EQ(replica.Descriptor({1})(0), 77);
    EXPECT_NEAR(replica.Normal({1}).y(), 1.0, 1e-6);
    EXPECT_EQ(replica.num_points(), 3u);
    EXPECT_FALSE(replica.IsAlive({7}));

    memory::ObjectPool<events::MapPointsRemoved, memory::ThreadSafePolicy> removed(1);
    events::MapPointsRemoved* rm = removed.Create();
    rm->map_id = {1};
    rm->ids = {{0}, {9}};
    replica.Apply(*rm);
    rm->Recycle();
    EXPECT_FALSE(replica.IsAlive({0}));
    EXPECT_TRUE(replica.IsAlive({1}));
    EXPECT_FALSE(replica.IsAlive({9}));
}

TEST(MapReplicaTest, KeyFrameStoresObservationsAndReference) {
    TReplica replica;
    const auto pose = MakePose(0.3, 1, -2, 0.5);
    // The points arrive with the Frame and are applied first: the references see their positions.
    CreateKeyFrame(replica, 1, 4, pose, {2, 5, 3}, {{0, 0}, {2, 2}}, kThreePoints);

    const auto* keyframe = replica.keyframe({4});
    ASSERT_NE(keyframe, nullptr);
    EXPECT_EQ(keyframe->id.value, 4u);
    EXPECT_TRUE(keyframe->T_cw.ToMatrix34().isApprox(pose.ToMatrix34()));
    ASSERT_EQ(keyframe->observations.size(), 2u);
    EXPECT_EQ(keyframe->observations[1].first, 2u);
    EXPECT_EQ(keyframe->observations[1].second.value, 2u);
    EXPECT_EQ(replica.keyframe_ids().size(), 1u);
    EXPECT_EQ(replica.keyframe({5}), nullptr);

    const Eigen::Vector3d centre = pose.Inverse().translation().cast<double>();
    const auto& ref0 = replica.Reference({0});
    ASSERT_TRUE(ref0.has_value());
    EXPECT_EQ(ref0->keyframe.value, 4u);
    EXPECT_EQ(ref0->feature, 0);
    EXPECT_EQ(ref0->level, 2);
    EXPECT_NEAR(ref0->distance, (kThreePoints[0] - centre).norm(), 1e-12);
    const auto& ref2 = replica.Reference({2});
    ASSERT_TRUE(ref2.has_value());
    EXPECT_EQ(ref2->feature, 2);
    EXPECT_EQ(ref2->level, 3);
    EXPECT_FALSE(replica.Reference({1}).has_value());
}

TEST(MapReplicaTest, ReferenceIsFirstObservingKeyFrame) {
    TReplica replica;
    CreateKeyFrame(replica, 1, 4, MakePose(0.0, 0, 0, 0), {2, 5, 3}, {{0, 0}}, kThreePoints);
    CreateKeyFrame(replica, 1, 9, MakePose(0.1, 1, 0, 0), {1, 1, 1}, {{1, 0}, {2, 1}});
    const auto& ref0 = replica.Reference({0});
    ASSERT_TRUE(ref0.has_value());
    EXPECT_EQ(ref0->keyframe.value, 4u);
    EXPECT_EQ(ref0->feature, 0);
    ASSERT_TRUE(replica.Reference({1}).has_value());
    EXPECT_EQ(replica.Reference({1})->keyframe.value, 9u);
    EXPECT_EQ(replica.keyframe_ids().size(), 2u);
}

TEST(MapReplicaTest, KeyFramePoseUpdated) {
    TReplica replica;
    AddPoints(replica, 1, 0, kThreePoints);
    CreateKeyFrame(replica, 1, 3, MakePose(0.1, 1, 2, 3), {0, 0, 0}, {{0, 0}});
    const auto updated_pose = MakePose(0.4, 5, 6, 7);
    memory::ObjectPool<events::KeyFramePoseUpdated, memory::ThreadSafePolicy> pool(1);
    events::KeyFramePoseUpdated* event = pool.Create();
    event->map_id = {1};
    event->frame_id = {3};
    event->pose = updated_pose;
    replica.Apply(*event);
    event->frame_id = {99};  // unknown keyframe: ignored
    replica.Apply(*event);
    event->Recycle();
    EXPECT_TRUE(replica.keyframe({3})->T_cw.ToMatrix34().isApprox(updated_pose.ToMatrix34()));
    EXPECT_EQ(replica.keyframe({99}), nullptr);
}

TEST(MapReplicaTest, NewerMapClears) {
    TReplica replica;
    AddPoints(replica, 1, 0, kThreePoints);
    CreateKeyFrame(replica, 1, 3, MakePose(0.1, 1, 2, 3), {0, 0, 0}, {{0, 0}});
    ASSERT_EQ(replica.num_points(), 3u);

    memory::ObjectPool<events::MapCreated, memory::ThreadSafePolicy> pool(1);
    events::MapCreated* created = pool.Create();
    created->map_id = {2};
    created->frame_id = {10};
    replica.Apply(*created);
    created->Recycle();
    EXPECT_EQ(replica.map_id().value, 2u);
    EXPECT_EQ(replica.num_points(), 0u);
    EXPECT_TRUE(replica.keyframe_ids().empty());

    AddPoints(replica, 2, 0, {{1, 1, 1}});
    EXPECT_EQ(replica.num_points(), 1u);
}

TEST(MapReplicaTest, OlderAndZeroMapIgnored) {
    TReplica replica;
    AddPoints(replica, 2, 0, kThreePoints);
    AddPoints(replica, 1, 3, {{0, 0, 0}});
    AddPoints(replica, 0, 3, {{0, 0, 0}});
    CreateKeyFrame(replica, 1, 3, MakePose(0.1, 1, 2, 3), {0, 0, 0}, {{0, 0}});
    CreateKeyFrame(replica, 0, 4, MakePose(0.1, 1, 2, 3), {0, 0, 0}, {{0, 0}});
    EXPECT_EQ(replica.map_id().value, 2u);
    EXPECT_EQ(replica.num_points(), 3u);
    // Only map 2's Frame, the one that carried its points.
    EXPECT_EQ(replica.keyframe_ids(), std::vector<map::FrameId>{map::FrameId{100}});

    TReplica fresh;
    AddPoints(fresh, 0, 0, kThreePoints);
    EXPECT_EQ(fresh.map_id().value, 0u);
    EXPECT_EQ(fresh.num_points(), 0u);
}

TEST(MapReplicaTest, AppliesCombinedKeyFrameCreatedAtomically) {
    TReplica replica;
    CreateKeyFrame(replica, 1, 2, MakePose(0.0, 0, 0, 0), {0, 0, 0, 0}, {{0, 0}, {1, 1}, {2, 2}},
                   kThreePoints);

    // Frame 5 adds points 3-5, updates point 1, adds an observation of new
    // point 4 to existing Frame 2, and observes points 1, 3 and 5 itself.
    memory::ObjectPool<TKeyFrameCreated, memory::ThreadSafePolicy> pool(1);
    TKeyFrameCreated* event = pool.Create();
    SetFrame(*event, 1, 5, MakePose(0.1, 1, 0, 0), {0, 1, 2}, {{0, 1}, {1, 3}, {2, 5}});
    SetNewPoints(*event, 3, {{1, 0, 4}, {2, 0, 4}, {3, 0, 4}});
    event->ResizeUpdatedPoints(1);
    event->updated_ids[0] = {1};
    event->updated_positions.col(0) = Eigen::Vector3d(4, 5, 7).cast<precision_t>();
    event->updated_descriptors.col(0).setConstant(42);
    event->updated_normals.col(0) = Eigen::Vector3d(1, 0, 0).cast<precision_t>();
    event->added_observations.push_back({map::FrameId{2}, 3, map::MapPointId{4}});
    replica.Apply(*event);
    event->Recycle();

    ASSERT_EQ(replica.num_points(), 6u);
    EXPECT_TRUE(replica.Position({4}).isApprox(Eigen::Vector3d(2, 0, 4).cast<precision_t>()));
    EXPECT_TRUE(replica.Position({1}).isApprox(Eigen::Vector3d(4, 5, 7).cast<precision_t>()));
    // Point 3's reference is Frame 5, at the new point's position.
    ASSERT_TRUE(replica.Reference({3}).has_value());
    EXPECT_EQ(replica.Reference({3})->keyframe.value, 5u);
    EXPECT_EQ(replica.Reference({3})->level, 1);
    // Point 1 keeps its reference to Frame 2.
    EXPECT_EQ(replica.Reference({1})->keyframe.value, 2u);
    // The observation on the existing Frame.
    const auto& frame2 = replica.keyframe({2})->observations;
    ASSERT_EQ(frame2.size(), 4u);
    EXPECT_EQ(frame2.back(), (std::pair<uint32_t, map::MapPointId>{3, map::MapPointId{4}}));

    // Every observation refers to a live point and an existing Frame, and the
    // per-Frame lists and the per-point index agree both ways.
    const map::CovisibilityIndex& covisibility = replica.covisibility();
    size_t num_observations = 0;
    for (const map::FrameId frame : replica.keyframe_ids()) {
        const auto& observations = replica.keyframe(frame)->observations;
        num_observations += observations.size();
        EXPECT_EQ(covisibility.Points(frame).size(), observations.size()) << "frame " << frame.value;
        for (const auto& [feature, point] : observations) {
            EXPECT_TRUE(replica.IsAlive(point)) << "point " << point.value;
            EXPECT_TRUE(covisibility.Contains(frame, point));
        }
    }
    size_t num_indexed = 0;
    for (uint32_t p = 0; p < replica.num_points(); ++p) {
        for (const map::PointObservation& observation : covisibility.Observations({p})) {
            ++num_indexed;
            const auto* keyframe = replica.keyframe(observation.frame);
            ASSERT_NE(keyframe, nullptr);
            EXPECT_NE(std::find(keyframe->observations.begin(), keyframe->observations.end(),
                                std::pair<uint32_t, map::MapPointId>{observation.feature, {p}}),
                      keyframe->observations.end())
                << "point " << p << " frame " << observation.frame.value;
        }
    }
    EXPECT_EQ(num_indexed, num_observations);
    EXPECT_EQ(covisibility.NumObservations({1}), 2u);
    EXPECT_EQ(covisibility.NumObservations({4}), 1u);
    EXPECT_EQ(covisibility.Covisible({5}, 10),
              (std::vector<std::pair<map::FrameId, uint32_t>>{{map::FrameId{2}, 1}}));
}

TEST(MapReplicaTest, UpdatedPointsChangeNormalAndDescriptor) {
    TReplica replica;
    CreateKeyFrame(replica, 1, 0, MakePose(0.0, 0, 0, 0), {0}, {{0, 0}}, kThreePoints);

    memory::ObjectPool<TKeyFrameCreated, memory::ThreadSafePolicy> pool(1);
    TKeyFrameCreated* event = pool.Create();
    SetFrame(*event, 1, 1, MakePose(0.1, 1, 0, 0), {0}, {{0, 2}});
    event->ResizeUpdatedPoints(2);
    event->updated_ids[0] = {2};
    event->updated_ids[1] = {0};
    for (Eigen::Index i = 0; i < 2; ++i) {
        event->updated_positions.col(i) = kThreePoints[static_cast<size_t>(2 - 2 * i)].cast<precision_t>();
        event->updated_descriptors.col(i).setConstant(static_cast<uint8_t>(50 + i));
        event->updated_normals.col(i) = Eigen::Vector3d(0, 1 + i, 0).cast<precision_t>();
    }
    replica.Apply(*event);
    event->Recycle();

    EXPECT_EQ(replica.Descriptor({2})(0), 50);
    EXPECT_NEAR(replica.Normal({2}).y(), 1.0, 1e-6);
    EXPECT_EQ(replica.Descriptor({0})(31), 51);
    EXPECT_NEAR(replica.Normal({0}).y(), 2.0, 1e-6);
    EXPECT_EQ(replica.Descriptor({1})(0), 2);  // untouched
    EXPECT_EQ(replica.num_points(), 3u);
}

} // namespace
} // namespace voyahslam::tracking
