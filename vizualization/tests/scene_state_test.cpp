#include <voyahviz/scene_state.h>

#include <gtest/gtest.h>

#include <voyahslam/events/events.h>
#include <voyahslam/features/orb_feature_extractor.h>
#include <voyahslam/memory/object_pool.h>

namespace voyahviz {
namespace {

using namespace voyahslam;
using events::EventType;
using TOrbScene = SceneState<features::OrbFeatureExtractor>;
using TOrbMapPointsAdded = events::MapPointsAdded<features::OrbFeatureExtractor>;

template <typename TEvent>
void ApplyOne(TOrbScene& out_scene, memory::ObjectPool<TEvent, memory::ThreadSafePolicy>& pool) {
    TEvent* event = pool.Create();
    out_scene.Apply(*event);
    event->Recycle();
}

TEST(SceneStateTest, StartsEmpty) {
    TOrbScene scene;
    EXPECT_EQ(scene.total_applied(), 0u);
    EXPECT_EQ(scene.applied_count(EventType::PositionUpdated), 0u);
}

TEST(SceneStateTest, CountsAppliedEventsPerType) {
    memory::ObjectPool<events::PositionUpdated, memory::ThreadSafePolicy> positions(2);
    memory::ObjectPool<TOrbMapPointsAdded, memory::ThreadSafePolicy> added(1);
    memory::ObjectPool<events::MapPointsRemoved, memory::ThreadSafePolicy> removed(1);

    TOrbScene scene;
    ApplyOne(scene, positions);
    ApplyOne(scene, positions);
    ApplyOne(scene, added);
    ApplyOne(scene, removed);

    EXPECT_EQ(scene.applied_count(EventType::PositionUpdated), 2u);
    EXPECT_EQ(scene.applied_count(EventType::MapPointsAdded), 1u);
    EXPECT_EQ(scene.applied_count(EventType::MapPointsRemoved), 1u);
    EXPECT_EQ(scene.applied_count(EventType::KeyFrameCreated), 0u);
    EXPECT_EQ(scene.total_applied(), 4u);
}

TEST(SceneStateTest, CountsNewEventTypes) {
    memory::ObjectPool<events::MapCreated, memory::ThreadSafePolicy> created(1);
    memory::ObjectPool<events::TrackingLost, memory::ThreadSafePolicy> lost(1);
    memory::ObjectPool<events::InitializationMeasurement<features::OrbFeatureExtractor>,
                       memory::ThreadSafePolicy> init(1);

    TOrbScene scene;
    ApplyOne(scene, created);
    ApplyOne(scene, lost);
    ApplyOne(scene, init);

    EXPECT_EQ(scene.applied_count(EventType::MapCreated), 1u);
    EXPECT_EQ(scene.applied_count(EventType::TrackingLost), 1u);
    EXPECT_EQ(scene.applied_count(EventType::InitializationMeasurement), 1u);
}

using TOrbKeyFrameCreated = events::KeyFrameCreated<features::OrbFeatureExtractor>;
using TOrbMapPointsUpdated = events::MapPointsUpdated<features::OrbFeatureExtractor>;
using TOrbInit = events::InitializationMeasurement<features::OrbFeatureExtractor>;

geometry::RigidTransformation MakePose(double angle, double tx, double ty, double tz) {
    const Eigen::Matrix3d rotation = Eigen::AngleAxisd(angle, Eigen::Vector3d::UnitY()).toRotationMatrix();
    return geometry::RigidTransformation(rotation.cast<precision_t>(),
                                         Eigen::Vector3d(tx, ty, tz).cast<precision_t>());
}

// Applies a MapPointsAdded for `map` with the given points starting at `start`.
void AddPoints(TOrbScene& out_scene, uint32_t map, uint32_t start,
               const std::vector<Eigen::Vector3d>& points) {
    memory::ObjectPool<TOrbMapPointsAdded, memory::ThreadSafePolicy> pool(1);
    TOrbMapPointsAdded* event = pool.Create();
    event->map_id = {map};
    event->start_id = {start};
    event->Resize(static_cast<uint32_t>(points.size()));
    for (size_t i = 0; i < points.size(); ++i) {
        event->positions.col(static_cast<Eigen::Index>(i)) = points[i].cast<precision_t>();
    }
    out_scene.Apply(*event);
    event->Recycle();
}

void CreateKeyFrame(TOrbScene& out_scene, uint32_t map, uint32_t frame,
                    const geometry::RigidTransformation& pose) {
    memory::ObjectPool<TOrbKeyFrameCreated, memory::ThreadSafePolicy> pool(1);
    TOrbKeyFrameCreated* event = pool.Create();
    event->map_id = {map};
    event->frame_id = {frame};
    event->pose = pose;
    out_scene.Apply(*event);
    event->Recycle();
}

void UpdatePosition(TOrbScene& out_scene, uint32_t map, const geometry::RigidTransformation& pose) {
    memory::ObjectPool<events::PositionUpdated, memory::ThreadSafePolicy> pool(1);
    events::PositionUpdated* event = pool.Create();
    event->map_id = {map};
    event->frame_id = {0};
    event->pose = pose;
    out_scene.Apply(*event);
    event->Recycle();
}

void LoseTracking(TOrbScene& out_scene, uint32_t map) {
    memory::ObjectPool<events::TrackingLost, memory::ThreadSafePolicy> pool(1);
    events::TrackingLost* event = pool.Create();
    event->map_id = {map};
    out_scene.Apply(*event);
    event->Recycle();
}

const std::vector<Eigen::Vector3d> kThreePoints = {{1, 2, 3}, {4, 5, 6}, {7, 8, 9}};

TEST(SceneStateTest, PointsAddedUpdatedRemoved) {
    TOrbScene scene;
    AddPoints(scene, 1, 0, kThreePoints);
    ASSERT_EQ(scene.point_positions().size(), 3u);
    EXPECT_EQ(scene.num_alive_points(), 3u);
    EXPECT_TRUE(scene.point_positions()[2].isApprox(kThreePoints[2].cast<precision_t>()));

    memory::ObjectPool<TOrbMapPointsUpdated, memory::ThreadSafePolicy> updated(1);
    TOrbMapPointsUpdated* up = updated.Create();
    up->map_id = {1};
    up->Resize(2);
    up->ids[0] = {1};
    up->ids[1] = {7};
    up->positions.col(0) = Eigen::Vector3d(10, 20, 30).cast<precision_t>();
    up->positions.col(1) = Eigen::Vector3d(-1, -1, -1).cast<precision_t>();
    scene.Apply(*up);
    up->Recycle();
    EXPECT_TRUE(scene.point_positions()[1].isApprox(Eigen::Vector3d(10, 20, 30).cast<precision_t>()));
    EXPECT_EQ(scene.point_positions().size(), 3u);

    memory::ObjectPool<events::MapPointsRemoved, memory::ThreadSafePolicy> removed(1);
    events::MapPointsRemoved* rm = removed.Create();
    rm->map_id = {1};
    rm->ids = {{0}, {9}};
    scene.Apply(*rm);
    rm->Recycle();
    EXPECT_FALSE(scene.point_alive()[0]);
    EXPECT_TRUE(scene.point_alive()[1]);
    EXPECT_EQ(scene.num_alive_points(), 2u);
}

TEST(SceneStateTest, AddedWithGapLeavesGapDead) {
    TOrbScene scene;
    AddPoints(scene, 1, 5, {{1, 1, 1}, {2, 2, 2}});
    ASSERT_EQ(scene.point_positions().size(), 7u);
    for (size_t i = 0; i < 5; ++i) EXPECT_FALSE(scene.point_alive()[i]) << i;
    EXPECT_TRUE(scene.point_alive()[5]);
    EXPECT_TRUE(scene.point_alive()[6]);
    EXPECT_EQ(scene.num_alive_points(), 2u);
}

TEST(SceneStateTest, KeyFramesInsertedAndUpdated) {
    TOrbScene scene;
    CreateKeyFrame(scene, 1, 3, MakePose(0.1, 1, 2, 3));
    ASSERT_EQ(scene.keyframes().size(), 1u);

    const auto updated_pose = MakePose(0.4, 5, 6, 7);
    memory::ObjectPool<events::KeyFramePoseUpdated, memory::ThreadSafePolicy> pool(1);
    events::KeyFramePoseUpdated* event = pool.Create();
    event->map_id = {1};
    event->frame_id = {3};
    event->pose = updated_pose;
    scene.Apply(*event);
    event->Recycle();

    ASSERT_EQ(scene.keyframes().size(), 1u);
    EXPECT_TRUE(scene.keyframes().at({3}).ToMatrix34().isApprox(updated_pose.ToMatrix34()));
}

TEST(SceneStateTest, PositionUpdatedSetsPoseAndTrajectory) {
    TOrbScene scene;
    const auto pose = MakePose(0.5, 1, 2, 3);
    UpdatePosition(scene, 1, pose);
    ASSERT_TRUE(scene.current_pose().has_value());
    EXPECT_TRUE(scene.current_pose()->ToMatrix34().isApprox(pose.ToMatrix34()));
    EXPECT_EQ(scene.status(), TrackingStatus::Tracking);
    ASSERT_EQ(scene.trajectory().size(), 1u);
    EXPECT_TRUE(scene.trajectory().back().isApprox(pose.Inverse().translation()));

    UpdatePosition(scene, 1, MakePose(0.6, 2, 2, 3));
    EXPECT_EQ(scene.trajectory().size(), 2u);
}

TEST(SceneStateTest, MapCreatedSetsCurrentPose) {
    TOrbScene scene;
    memory::ObjectPool<events::MapCreated, memory::ThreadSafePolicy> pool(1);
    events::MapCreated* event = pool.Create();
    event->map_id = {1};
    event->frame_id = {0};
    event->pose = MakePose(0.2, 1, 0, 0);
    scene.Apply(*event);
    event->Recycle();
    ASSERT_TRUE(scene.current_pose().has_value());
    EXPECT_TRUE(scene.current_pose()->ToMatrix34().isApprox(MakePose(0.2, 1, 0, 0).ToMatrix34()));
    EXPECT_EQ(scene.status(), TrackingStatus::Tracking);
    EXPECT_TRUE(scene.trajectory().empty());
}

TEST(SceneStateTest, TrackingLostKeepsScene) {
    TOrbScene scene;
    AddPoints(scene, 1, 0, kThreePoints);
    CreateKeyFrame(scene, 1, 0, MakePose(0, 0, 0, 0));
    UpdatePosition(scene, 1, MakePose(0.1, 1, 1, 1));
    LoseTracking(scene, 1);
    EXPECT_EQ(scene.status(), TrackingStatus::Lost);
    EXPECT_EQ(scene.num_alive_points(), 3u);
    EXPECT_EQ(scene.keyframes().size(), 1u);
    EXPECT_TRUE(scene.current_pose().has_value());
}

TEST(SceneStateTest, OlderMapIgnored) {
    TOrbScene scene;
    AddPoints(scene, 2, 0, kThreePoints);
    AddPoints(scene, 1, 0, {{9, 9, 9}});
    EXPECT_EQ(scene.current_map_id().value, 2u);
    EXPECT_EQ(scene.point_positions().size(), 3u);
    EXPECT_EQ(scene.applied_count(EventType::MapPointsAdded), 2u);
}

TEST(SceneStateTest, NewerMapClearsAndAdopts) {
    TOrbScene scene;
    AddPoints(scene, 1, 0, kThreePoints);
    CreateKeyFrame(scene, 1, 0, MakePose(0, 0, 0, 0));
    UpdatePosition(scene, 1, MakePose(0.1, 1, 1, 1));
    AddPoints(scene, 2, 0, {{1, 1, 1}});
    EXPECT_EQ(scene.current_map_id().value, 2u);
    EXPECT_EQ(scene.point_positions().size(), 1u);
    EXPECT_EQ(scene.num_alive_points(), 1u);
    EXPECT_TRUE(scene.keyframes().empty());
    EXPECT_FALSE(scene.current_pose().has_value());
    EXPECT_TRUE(scene.trajectory().empty());
    EXPECT_EQ(scene.status(), TrackingStatus::None);
}

TEST(SceneStateTest, NewerMapKeyFrameSwitchesScene) {
    TOrbScene scene;
    AddPoints(scene, 1, 0, kThreePoints);
    CreateKeyFrame(scene, 2, 0, MakePose(0, 0, 0, 0));
    EXPECT_EQ(scene.current_map_id().value, 2u);
    EXPECT_TRUE(scene.point_positions().empty());
    EXPECT_EQ(scene.keyframes().size(), 1u);
}

TEST(SceneStateTest, MapZeroCountedOnly) {
    TOrbScene scene;
    AddPoints(scene, 1, 0, kThreePoints);
    memory::ObjectPool<TOrbInit, memory::ThreadSafePolicy> pool(1);
    TOrbInit* event = pool.Create();
    event->map_id = {0};
    scene.Apply(*event);
    event->Recycle();
    EXPECT_EQ(scene.current_map_id().value, 1u);
    EXPECT_EQ(scene.num_alive_points(), 3u);
    EXPECT_EQ(scene.applied_count(EventType::InitializationMeasurement), 1u);
}

} // namespace
} // namespace voyahviz
