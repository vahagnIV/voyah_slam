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
using TOrbKeyFrameCreated = events::KeyFrameCreated<features::OrbFeatureExtractor>;
using TOrbFrameTracked = events::FrameTracked<features::OrbFeatureExtractor>;

template <typename TEvent>
void ApplyOne(TOrbScene& out_scene, memory::ObjectPool<TEvent, memory::ThreadSafePolicy>& pool) {
    TEvent* event = pool.Create();
    out_scene.Apply(*event);
    event->Recycle();
}

TEST(SceneStateTest, StartsEmpty) {
    TOrbScene scene;
    EXPECT_EQ(scene.total_applied(), 0u);
    EXPECT_EQ(scene.applied_count(EventType::FrameTracked), 0u);
}

TEST(SceneStateTest, CountsAppliedEventsPerType) {
    memory::ObjectPool<TOrbFrameTracked, memory::ThreadSafePolicy> positions(2);
    memory::ObjectPool<TOrbKeyFrameCreated, memory::ThreadSafePolicy> added(1);
    memory::ObjectPool<events::MapPointsRemoved, memory::ThreadSafePolicy> removed(1);

    TOrbScene scene;
    ApplyOne(scene, positions);
    ApplyOne(scene, positions);
    ApplyOne(scene, added);
    ApplyOne(scene, removed);

    EXPECT_EQ(scene.applied_count(EventType::FrameTracked), 2u);
    EXPECT_EQ(scene.applied_count(EventType::KeyFrameCreated), 1u);
    EXPECT_EQ(scene.applied_count(EventType::MapPointsRemoved), 1u);
    EXPECT_EQ(scene.applied_count(EventType::MapPointsUpdated), 0u);
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

using TOrbMapPointsUpdated = events::MapPointsUpdated<features::OrbFeatureExtractor>;
using TOrbInit = events::InitializationMeasurement<features::OrbFeatureExtractor>;

geometry::RigidTransformation MakePose(double angle, double tx, double ty, double tz) {
    const Eigen::Matrix3d rotation = Eigen::AngleAxisd(angle, Eigen::Vector3d::UnitY()).toRotationMatrix();
    return geometry::RigidTransformation(rotation.cast<precision_t>(),
                                         Eigen::Vector3d(tx, ty, tz).cast<precision_t>());
}

// Applies a KeyFrameCreated of Frame `frame` that adds `new_points` from id `start`.
void CreateKeyFrame(TOrbScene& out_scene, uint32_t map, uint32_t frame,
                    const geometry::RigidTransformation& pose,
                    const std::vector<Eigen::Vector3d>& new_points = {}, uint32_t start = 0) {
    memory::ObjectPool<TOrbKeyFrameCreated, memory::ThreadSafePolicy> pool(1);
    TOrbKeyFrameCreated* event = pool.Create();
    event->map_id = {map};
    event->frame_id = {frame};
    event->pose = pose;
    event->new_points_start_id = {start};
    event->ResizeNewPoints(static_cast<uint32_t>(new_points.size()));
    for (size_t i = 0; i < new_points.size(); ++i) {
        event->new_positions.col(static_cast<Eigen::Index>(i)) = new_points[i].cast<precision_t>();
    }
    out_scene.Apply(*event);
    event->Recycle();
}

// Adds points for `map` from id `start`, carried by Frame `frame`.
void AddPoints(TOrbScene& out_scene, uint32_t map, uint32_t start,
               const std::vector<Eigen::Vector3d>& points, uint32_t frame = 100) {
    CreateKeyFrame(out_scene, map, frame, geometry::RigidTransformation::Identity(), points, start);
}

void TrackFrame(TOrbScene& out_scene, uint32_t map, const geometry::RigidTransformation& pose) {
    memory::ObjectPool<TOrbFrameTracked, memory::ThreadSafePolicy> pool(1);
    TOrbFrameTracked* event = pool.Create();
    event->map_id = {map};
    event->reference_frame_id = map::FrameId{0};
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

TEST(SceneStateTest, KeyFrameCreatedUpdatesPoints) {
    TOrbScene scene;
    CreateKeyFrame(scene, 1, 0, MakePose(0, 0, 0, 0), kThreePoints);

    memory::ObjectPool<TOrbKeyFrameCreated, memory::ThreadSafePolicy> pool(1);
    TOrbKeyFrameCreated* event = pool.Create();
    event->map_id = {1};
    event->frame_id = {1};
    event->pose = MakePose(0.1, 1, 0, 0);
    event->new_points_start_id = {3};
    event->ResizeNewPoints(1);
    event->new_positions.col(0) = Eigen::Vector3d(0, 0, 5).cast<precision_t>();
    event->ResizeUpdatedPoints(2);
    event->updated_ids[0] = {2};
    event->updated_ids[1] = {8};  // unknown: ignored
    event->updated_positions.col(0) = Eigen::Vector3d(10, 20, 30).cast<precision_t>();
    event->updated_positions.col(1) = Eigen::Vector3d(-1, -1, -1).cast<precision_t>();
    scene.Apply(*event);
    event->Recycle();

    ASSERT_EQ(scene.point_positions().size(), 4u);
    EXPECT_EQ(scene.num_alive_points(), 4u);
    EXPECT_TRUE(scene.point_positions()[3].isApprox(Eigen::Vector3d(0, 0, 5).cast<precision_t>()));
    EXPECT_TRUE(scene.point_positions()[2].isApprox(Eigen::Vector3d(10, 20, 30).cast<precision_t>()));
    EXPECT_EQ(scene.keyframes().size(), 2u);
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

TEST(SceneStateTest, FrameTrackedSetsPoseAndTrajectory) {
    TOrbScene scene;
    const auto pose = MakePose(0.5, 1, 2, 3);
    TrackFrame(scene, 1, pose);
    ASSERT_TRUE(scene.current_pose().has_value());
    EXPECT_TRUE(scene.current_pose()->ToMatrix34().isApprox(pose.ToMatrix34()));
    EXPECT_EQ(scene.status(), TrackingStatus::Tracking);
    ASSERT_EQ(scene.trajectory().size(), 1u);
    EXPECT_TRUE(scene.trajectory().back().isApprox(pose.Inverse().translation()));

    TrackFrame(scene, 1, MakePose(0.6, 2, 2, 3));
    EXPECT_EQ(scene.trajectory().size(), 2u);
}

TEST(SceneStateTest, FrameTrackedSetsTrackedAndVisible) {
    TOrbScene scene;
    memory::ObjectPool<TOrbFrameTracked, memory::ThreadSafePolicy> pool(1);
    TOrbFrameTracked* event = pool.Create();
    event->map_id = {1};
    event->pose = MakePose(0.1, 1, 0, 0);
    event->observed_points = {{0}, {2}};
    event->visible_points = {{0}, {1}, {2}};
    event->local_frames = {{4}, {5}};
    scene.Apply(*event);
    event->Recycle();

    EXPECT_EQ(scene.tracked_points(), (std::vector<map::MapPointId>{{0}, {2}}));
    EXPECT_EQ(scene.visible_points(), (std::vector<map::MapPointId>{{0}, {1}, {2}}));
    EXPECT_EQ(scene.local_frames(), (std::vector<map::FrameId>{{4}, {5}}));

    // The next FrameTracked replaces them.
    TrackFrame(scene, 1, MakePose(0.2, 1, 0, 0));
    EXPECT_TRUE(scene.tracked_points().empty());
    EXPECT_TRUE(scene.visible_points().empty());
    EXPECT_TRUE(scene.local_frames().empty());
}

TEST(SceneStateTest, TrackingLostClearsTrackedSets) {
    TOrbScene scene;
    memory::ObjectPool<TOrbFrameTracked, memory::ThreadSafePolicy> pool(1);
    TOrbFrameTracked* event = pool.Create();
    event->map_id = {1};
    event->observed_points = {{0}};
    event->visible_points = {{0}, {1}};
    event->local_frames = {{4}};
    scene.Apply(*event);
    event->Recycle();
    ASSERT_FALSE(scene.tracked_points().empty());

    LoseTracking(scene, 1);
    EXPECT_TRUE(scene.tracked_points().empty());
    EXPECT_TRUE(scene.visible_points().empty());
    EXPECT_TRUE(scene.local_frames().empty());
}

TEST(SceneStateTest, NewerMapClearsTrackedSets) {
    TOrbScene scene;
    memory::ObjectPool<TOrbFrameTracked, memory::ThreadSafePolicy> pool(1);
    TOrbFrameTracked* event = pool.Create();
    event->map_id = {1};
    event->observed_points = {{0}};
    event->visible_points = {{1}};
    event->local_frames = {{4}};
    scene.Apply(*event);
    event->Recycle();
    AddPoints(scene, 2, 0, {{1, 1, 1}});
    EXPECT_TRUE(scene.tracked_points().empty());
    EXPECT_TRUE(scene.visible_points().empty());
    EXPECT_TRUE(scene.local_frames().empty());
}

TEST(SceneStateTest, KeyFrameCreatedAppliesUpdatedPoints) {
    TOrbScene scene;
    CreateKeyFrame(scene, 1, 0, MakePose(0, 0, 0, 0), kThreePoints);

    memory::ObjectPool<TOrbKeyFrameCreated, memory::ThreadSafePolicy> pool(1);
    TOrbKeyFrameCreated* event = pool.Create();
    event->map_id = {1};
    event->frame_id = {1};
    event->new_points_start_id = {3};
    event->ResizeUpdatedPoints(1);
    event->updated_ids[0] = {1};
    event->updated_positions.col(0) = Eigen::Vector3d(7, 7, 7).cast<precision_t>();
    scene.Apply(*event);
    event->Recycle();
    EXPECT_TRUE(scene.point_positions()[1].isApprox(Eigen::Vector3d(7, 7, 7).cast<precision_t>()));
    EXPECT_EQ(scene.num_alive_points(), 3u);
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
    CreateKeyFrame(scene, 1, 0, MakePose(0, 0, 0, 0), kThreePoints);
    TrackFrame(scene, 1, MakePose(0.1, 1, 1, 1));
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
    EXPECT_EQ(scene.applied_count(EventType::KeyFrameCreated), 2u);
}

TEST(SceneStateTest, NewerMapClearsAndAdopts) {
    TOrbScene scene;
    CreateKeyFrame(scene, 1, 0, MakePose(0, 0, 0, 0), kThreePoints);
    TrackFrame(scene, 1, MakePose(0.1, 1, 1, 1));
    AddPoints(scene, 2, 0, {{1, 1, 1}}, /*frame=*/7);
    EXPECT_EQ(scene.current_map_id().value, 2u);
    EXPECT_EQ(scene.point_positions().size(), 1u);
    EXPECT_EQ(scene.num_alive_points(), 1u);
    // Only map 2's Frame.
    ASSERT_EQ(scene.keyframes().size(), 1u);
    EXPECT_EQ(scene.keyframes().begin()->first, map::FrameId{7});
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
