#include <voyahviz/scene_state.h>

#include <gtest/gtest.h>

#include <memory>
#include <vector>
#include <voyahslam/events/events.h>
#include <voyahslam/features/orb_feature_extractor.h>

namespace voyahviz {
namespace {

using namespace voyahslam;
using events::EventType;
using TOrbScene = SceneState<features::OrbFeatureExtractor>;
using TOrbTransaction = events::MapTransaction<features::OrbFeatureExtractor>;
using TOrbFrameTracked = events::FrameTracked<features::OrbFeatureExtractor>;

template <typename TEvent>
void ApplyOne(TOrbScene& out_scene) {
    auto event = std::make_shared<TEvent>();
    out_scene.Apply(*event);
}

TEST(SceneStateTest, StartsEmpty) {
    TOrbScene scene;
    EXPECT_EQ(scene.total_applied(), 0u);
    EXPECT_EQ(scene.applied_count(EventType::FrameTracked), 0u);
}

TEST(SceneStateTest, CountsAppliedEventsPerType) {
    TOrbScene scene;
    ApplyOne<TOrbFrameTracked>(scene);
    ApplyOne<TOrbFrameTracked>(scene);
    ApplyOne<TOrbTransaction>(scene);
    ApplyOne<TOrbTransaction>(scene);
    ApplyOne<events::LoopDetected>(scene);

    EXPECT_EQ(scene.applied_count(EventType::FrameTracked), 2u);
    EXPECT_EQ(scene.applied_count(EventType::MapTransaction), 2u);
    EXPECT_EQ(scene.applied_count(EventType::LoopDetected), 1u);
    EXPECT_EQ(scene.applied_count(EventType::TrackingLost), 0u);
    EXPECT_EQ(scene.total_applied(), 5u);
}

TEST(SceneStateTest, CountsNewEventTypes) {
    TOrbScene scene;
    ApplyOne<TOrbTransaction>(scene);
    ApplyOne<events::TrackingLost>(scene);
    ApplyOne<events::InitializationMeasurement<features::OrbFeatureExtractor>>(scene);

    EXPECT_EQ(scene.applied_count(EventType::MapTransaction), 1u);
    EXPECT_EQ(scene.applied_count(EventType::TrackingLost), 1u);
    EXPECT_EQ(scene.applied_count(EventType::InitializationMeasurement), 1u);
}

using TOrbInit = events::InitializationMeasurement<features::OrbFeatureExtractor>;

geometry::RigidTransformation MakePose(double angle, double tx, double ty, double tz) {
    const Eigen::Matrix3d rotation = Eigen::AngleAxisd(angle, Eigen::Vector3d::UnitY()).toRotationMatrix();
    return geometry::RigidTransformation(rotation.cast<precision_t>(),
                                         Eigen::Vector3d(tx, ty, tz).cast<precision_t>());
}

// Applies one transaction of map `map`, filled by `fill`; as the Mapper's, it
// names its map as the active one.
template <typename TFill>
void ApplyTransaction(TOrbScene& out_scene, uint32_t map, TFill fill) {
    auto event = std::make_shared<TOrbTransaction>();
    event->map_id = {map};
    event->active_map = map::MapId{map};
    fill(*event);
    out_scene.Apply(*event);
}

void AddFrame(TOrbTransaction& out_event, uint32_t frame, const geometry::RigidTransformation& pose) {
    auto& added = out_event.added_frames.emplace_back();
    added.id = {frame};
    added.T_cw = pose;
}

// Creates map `map` (active), without Frames or points.
void NewMap(TOrbScene& out_scene, uint32_t map) {
    ApplyTransaction(out_scene, map, [](TOrbTransaction& event) { event.new_map = true; });
}

// The Mapper's answer to a final loss of map `map`: removed, none active.
void RemoveMap(TOrbScene& out_scene, uint32_t map) {
    ApplyTransaction(out_scene, map, [&](TOrbTransaction& event) {
        event.removed_maps.push_back(map::MapId{map});
        event.active_map.reset();
    });
}

// Applies a transaction that adds Frame `frame` and `new_points` from id `start`.
void CreateKeyFrame(TOrbScene& out_scene, uint32_t map, uint32_t frame,
                    const geometry::RigidTransformation& pose,
                    const std::vector<Eigen::Vector3d>& new_points = {}, uint32_t start = 0) {
    auto event = std::make_shared<TOrbTransaction>();
    event->map_id = {map};
    event->active_map = map::MapId{map};
    AddFrame(*event, frame, pose);
    event->new_points_start_id = {start};
    event->ResizeNewPoints(static_cast<uint32_t>(new_points.size()));
    for (size_t i = 0; i < new_points.size(); ++i) {
        event->new_positions.col(static_cast<Eigen::Index>(i)) = new_points[i].cast<precision_t>();
    }
    out_scene.Apply(*event);
}

// Adds points for `map` from id `start`, carried by Frame `frame`.
void AddPoints(TOrbScene& out_scene, uint32_t map, uint32_t start,
               const std::vector<Eigen::Vector3d>& points, uint32_t frame = 100) {
    CreateKeyFrame(out_scene, map, frame, geometry::RigidTransformation::Identity(), points, start);
}

void TrackFrame(TOrbScene& out_scene, uint32_t map, const geometry::RigidTransformation& pose) {
    auto event = std::make_shared<TOrbFrameTracked>();
    event->map_id = {map};
    event->reference_frame_id = map::FrameId{0};
    event->pose = pose;
    out_scene.Apply(*event);
}

void LoseTracking(TOrbScene& out_scene, uint32_t map, bool final = true) {
    auto event = std::make_shared<events::TrackingLost>();
    event->map_id = {map};
    event->final = final;
    out_scene.Apply(*event);
}

const std::vector<Eigen::Vector3d> kThreePoints = {{1, 2, 3}, {4, 5, 6}, {7, 8, 9}};

TEST(SceneStateTest, PointsAddedUpdatedRemoved) {
    TOrbScene scene;
    NewMap(scene, 1);
    AddPoints(scene, 1, 0, kThreePoints);
    ASSERT_EQ(scene.point_positions().size(), 3u);
    EXPECT_EQ(scene.num_alive_points(), 3u);
    EXPECT_TRUE(scene.point_positions()[2].isApprox(kThreePoints[2].cast<precision_t>()));

    ApplyTransaction(scene, 1, [](TOrbTransaction& up) {
        up.ResizeUpdatedPoints(2);
        up.updated_ids[0] = {1};
        up.updated_ids[1] = {7};
        up.updated_positions.col(0) = Eigen::Vector3d(10, 20, 30).cast<precision_t>();
        up.updated_positions.col(1) = Eigen::Vector3d(-1, -1, -1).cast<precision_t>();
    });
    EXPECT_TRUE(scene.point_positions()[1].isApprox(Eigen::Vector3d(10, 20, 30).cast<precision_t>()));
    EXPECT_EQ(scene.point_positions().size(), 3u);

    ApplyTransaction(scene, 1, [](TOrbTransaction& rm) { rm.removed_points = {{0}, {9}}; });
    EXPECT_FALSE(scene.point_alive()[0]);
    EXPECT_TRUE(scene.point_alive()[1]);
    EXPECT_EQ(scene.num_alive_points(), 2u);
}

TEST(SceneStateTest, TransactionAddsFrameAndUpdatesPoints) {
    TOrbScene scene;
    NewMap(scene, 1);
    CreateKeyFrame(scene, 1, 0, MakePose(0, 0, 0, 0), kThreePoints);

    auto event = std::make_shared<TOrbTransaction>();
    event->map_id = {1};
    event->active_map = map::MapId{1};
    AddFrame(*event, 1, MakePose(0.1, 1, 0, 0));
    event->new_points_start_id = {3};
    event->ResizeNewPoints(1);
    event->new_positions.col(0) = Eigen::Vector3d(0, 0, 5).cast<precision_t>();
    event->ResizeUpdatedPoints(2);
    event->updated_ids[0] = {2};
    event->updated_ids[1] = {8};  // unknown: ignored
    event->updated_positions.col(0) = Eigen::Vector3d(10, 20, 30).cast<precision_t>();
    event->updated_positions.col(1) = Eigen::Vector3d(-1, -1, -1).cast<precision_t>();
    scene.Apply(*event);

    ASSERT_EQ(scene.point_positions().size(), 4u);
    EXPECT_EQ(scene.num_alive_points(), 4u);
    EXPECT_TRUE(scene.point_positions()[3].isApprox(Eigen::Vector3d(0, 0, 5).cast<precision_t>()));
    EXPECT_TRUE(scene.point_positions()[2].isApprox(Eigen::Vector3d(10, 20, 30).cast<precision_t>()));
    EXPECT_EQ(scene.keyframes().size(), 2u);
}

TEST(SceneStateTest, AddedWithGapLeavesGapDead) {
    TOrbScene scene;
    NewMap(scene, 1);
    AddPoints(scene, 1, 5, {{1, 1, 1}, {2, 2, 2}});
    ASSERT_EQ(scene.point_positions().size(), 7u);
    for (size_t i = 0; i < 5; ++i) EXPECT_FALSE(scene.point_alive()[i]) << i;
    EXPECT_TRUE(scene.point_alive()[5]);
    EXPECT_TRUE(scene.point_alive()[6]);
    EXPECT_EQ(scene.num_alive_points(), 2u);
}

TEST(SceneStateTest, KeyFramesInsertedAndUpdated) {
    TOrbScene scene;
    NewMap(scene, 1);
    CreateKeyFrame(scene, 1, 3, MakePose(0.1, 1, 2, 3));
    ASSERT_EQ(scene.keyframes().size(), 1u);

    const auto updated_pose = MakePose(0.4, 5, 6, 7);
    ApplyTransaction(scene, 1, [&](TOrbTransaction& event) {
        event.updated_poses = {{map::FrameId{3}, updated_pose},
                               {map::FrameId{8}, updated_pose}};  // unknown: ignored
    });

    ASSERT_EQ(scene.keyframes().size(), 1u);
    EXPECT_TRUE(scene.keyframes().at({3}).ToMatrix34().isApprox(updated_pose.ToMatrix34()));
}

TEST(SceneStateTest, FrameTrackedSetsPoseAndTrajectory) {
    TOrbScene scene;
    NewMap(scene, 1);
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
    NewMap(scene, 1);
    auto event = std::make_shared<TOrbFrameTracked>();
    event->map_id = {1};
    event->pose = MakePose(0.1, 1, 0, 0);
    event->observed_points = {{0}, {2}};
    event->visible_points = {{0}, {1}, {2}};
    event->local_frames = {{4}, {5}};
    scene.Apply(*event);

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
    NewMap(scene, 1);
    auto event = std::make_shared<TOrbFrameTracked>();
    event->map_id = {1};
    event->observed_points = {{0}};
    event->visible_points = {{0}, {1}};
    event->local_frames = {{4}};
    scene.Apply(*event);
    ASSERT_FALSE(scene.tracked_points().empty());

    LoseTracking(scene, 1);
    EXPECT_TRUE(scene.tracked_points().empty());
    EXPECT_TRUE(scene.visible_points().empty());
    EXPECT_TRUE(scene.local_frames().empty());
}

// A new active map clears the tracked sets.
TEST(SceneStateTest, NewActiveMapClearsTrackedSets) {
    TOrbScene scene;
    NewMap(scene, 1);
    auto event = std::make_shared<TOrbFrameTracked>();
    event->map_id = {1};
    event->observed_points = {{0}};
    event->visible_points = {{1}};
    event->local_frames = {{4}};
    scene.Apply(*event);
    NewMap(scene, 2);
    AddPoints(scene, 2, 0, {{1, 1, 1}});
    EXPECT_TRUE(scene.tracked_points().empty());
    EXPECT_TRUE(scene.visible_points().empty());
    EXPECT_TRUE(scene.local_frames().empty());
}

TEST(SceneStateTest, TransactionAppliesUpdatedPoints) {
    TOrbScene scene;
    NewMap(scene, 1);
    CreateKeyFrame(scene, 1, 0, MakePose(0, 0, 0, 0), kThreePoints);

    ApplyTransaction(scene, 1, [](TOrbTransaction& event) {
        event.new_points_start_id = {3};
        event.ResizeUpdatedPoints(1);
        event.updated_ids[0] = {1};
        event.updated_positions.col(0) = Eigen::Vector3d(7, 7, 7).cast<precision_t>();
    });
    EXPECT_TRUE(scene.point_positions()[1].isApprox(Eigen::Vector3d(7, 7, 7).cast<precision_t>()));
    EXPECT_EQ(scene.num_alive_points(), 3u);
}

TEST(SceneStateTest, NewMapSetsCurrentPose) {
    TOrbScene scene;
    ApplyTransaction(scene, 1, [](TOrbTransaction& event) {
        event.new_map = true;
        event.start_frame = map::FrameId{1};
        AddFrame(event, 0, MakePose(0.0, 0, 0, 0));
        AddFrame(event, 1, MakePose(0.2, 1, 0, 0));
    });
    EXPECT_EQ(scene.keyframes().size(), 2u);
    ASSERT_TRUE(scene.current_pose().has_value());
    EXPECT_TRUE(scene.current_pose()->ToMatrix34().isApprox(MakePose(0.2, 1, 0, 0).ToMatrix34()));
    EXPECT_EQ(scene.status(), TrackingStatus::Tracking);
    EXPECT_TRUE(scene.trajectory().empty());
}

TEST(SceneStateTest, TrackingLostKeepsScene) {
    TOrbScene scene;
    NewMap(scene, 1);
    CreateKeyFrame(scene, 1, 0, MakePose(0, 0, 0, 0), kThreePoints);
    TrackFrame(scene, 1, MakePose(0.1, 1, 1, 1));
    LoseTracking(scene, 1);
    EXPECT_EQ(scene.status(), TrackingStatus::Lost);
    EXPECT_EQ(scene.num_alive_points(), 3u);
    EXPECT_EQ(scene.keyframes().size(), 1u);
    EXPECT_TRUE(scene.current_pose().has_value());
}

TEST(SceneStateTest, NonFinalLostKeepsScene) {
    TOrbScene scene;
    NewMap(scene, 1);
    CreateKeyFrame(scene, 1, 0, MakePose(0, 0, 0, 0), kThreePoints);
    TrackFrame(scene, 1, MakePose(0.1, 1, 1, 1));
    LoseTracking(scene, 1, /*final=*/false);
    EXPECT_EQ(scene.status(), TrackingStatus::RecentlyLost);
    EXPECT_EQ(scene.current_map_id().value, 1u);
    EXPECT_EQ(scene.num_alive_points(), 3u);
    EXPECT_EQ(scene.keyframes().size(), 1u);
    EXPECT_TRUE(scene.current_pose().has_value());
    EXPECT_EQ(scene.trajectory().size(), 1u);
    EXPECT_TRUE(scene.tracked_points().empty());

    // Relocalized: tracking again in the same scene.
    TrackFrame(scene, 1, MakePose(0.2, 1, 1, 1));
    EXPECT_EQ(scene.status(), TrackingStatus::Tracking);
    EXPECT_EQ(scene.num_alive_points(), 3u);
    EXPECT_EQ(scene.trajectory().size(), 2u);

    // A final loss after a non-final one: Lost, scene still kept.
    LoseTracking(scene, 1, /*final=*/false);
    LoseTracking(scene, 1);
    EXPECT_EQ(scene.status(), TrackingStatus::Lost);
    EXPECT_EQ(scene.num_alive_points(), 3u);
}

// After the Mapper drops map 1 (a final loss), nothing is shown and the
// status stays Lost; map 1's late events are ignored, a new map is shown.
TEST(SceneStateTest, RemovedMapIgnored) {
    TOrbScene scene;
    NewMap(scene, 1);
    CreateKeyFrame(scene, 1, 0, MakePose(0, 0, 0, 0), kThreePoints);
    TrackFrame(scene, 1, MakePose(0.1, 1, 1, 1));
    LoseTracking(scene, 1);
    RemoveMap(scene, 1);
    EXPECT_EQ(scene.current_map_id().value, 0u);
    EXPECT_TRUE(scene.map_ids().empty());
    EXPECT_EQ(scene.num_alive_points(), 0u);
    EXPECT_TRUE(scene.keyframes().empty());
    EXPECT_FALSE(scene.current_pose().has_value());
    EXPECT_TRUE(scene.trajectory().empty());
    EXPECT_EQ(scene.status(), TrackingStatus::Lost);

    AddPoints(scene, 1, 3, {{9, 9, 9}});
    NewMap(scene, 1);
    TrackFrame(scene, 1, MakePose(0.2, 1, 1, 1));
    EXPECT_TRUE(scene.map_ids().empty());
    EXPECT_FALSE(scene.current_pose().has_value());

    NewMap(scene, 2);
    AddPoints(scene, 2, 0, kThreePoints);
    AddPoints(scene, 1, 0, {{9, 9, 9}});
    EXPECT_EQ(scene.current_map_id().value, 2u);
    EXPECT_EQ(scene.map_ids(), std::vector<map::MapId>{map::MapId{2}});
    EXPECT_EQ(scene.point_positions().size(), 3u);
    EXPECT_EQ(scene.status(), TrackingStatus::None);
    EXPECT_EQ(scene.applied_count(EventType::MapTransaction), 8u);
}

// A new active map is shown alone; the previous one is kept, inactive.
TEST(SceneStateTest, NewActiveMapShownAlone) {
    TOrbScene scene;
    NewMap(scene, 1);
    CreateKeyFrame(scene, 1, 0, MakePose(0, 0, 0, 0), kThreePoints);
    TrackFrame(scene, 1, MakePose(0.1, 1, 1, 1));
    NewMap(scene, 2);
    AddPoints(scene, 2, 0, {{1, 1, 1}}, /*frame=*/7);
    EXPECT_EQ(scene.current_map_id().value, 2u);
    EXPECT_EQ(scene.map_ids(), (std::vector<map::MapId>{map::MapId{1}, map::MapId{2}}));
    EXPECT_EQ(scene.point_positions().size(), 1u);
    EXPECT_EQ(scene.num_alive_points(), 1u);
    // Only map 2's Frame.
    ASSERT_EQ(scene.keyframes().size(), 1u);
    EXPECT_EQ(scene.keyframes().begin()->first, map::FrameId{7});
    EXPECT_FALSE(scene.current_pose().has_value());
    EXPECT_TRUE(scene.trajectory().empty());
    EXPECT_EQ(scene.status(), TrackingStatus::None);

    // The inactive map's camera events are ignored; its transactions are kept.
    TrackFrame(scene, 1, MakePose(0.2, 1, 1, 1));
    EXPECT_FALSE(scene.current_pose().has_value());
    ApplyTransaction(scene, 1, [](TOrbTransaction& event) {
        event.active_map = map::MapId{2};
        event.removed_points = {map::MapPointId{0}};
    });
    EXPECT_EQ(scene.current_map_id().value, 2u);
    EXPECT_EQ(scene.num_alive_points(), 1u);

    // Re-activated (only active_map activates): map 1 shown, with its change.
    ApplyTransaction(scene, 2, [](TOrbTransaction& event) { event.active_map = map::MapId{1}; });
    EXPECT_EQ(scene.current_map_id().value, 1u);
    EXPECT_EQ(scene.num_alive_points(), 2u);
    EXPECT_EQ(scene.keyframes().size(), 1u);
}

// A merge (map merging Task 5): map 2's Frames and alive points move into map
// 1 by the Mapper's id rule (alive old ids ascending take map 1's next ids),
// then the transaction applies to map 1 under the new ids, map 2 is dropped
// and map 1 is shown.
TEST(SceneStateTest, MergeAbsorbsMapByIdRule) {
    TOrbScene scene;
    NewMap(scene, 1);
    AddPoints(scene, 1, 0, {{1, 0, 0}, {2, 0, 0}}, /*frame=*/0);
    NewMap(scene, 2);
    AddPoints(scene, 2, 0, kThreePoints, /*frame=*/5);
    ApplyTransaction(scene, 2, [](TOrbTransaction& event) { event.removed_points = {map::MapPointId{1}}; });
    ApplyTransaction(scene, 1, [](TOrbTransaction& event) {
        event.absorbed_map = map::MapId{2};
        event.removed_maps = {map::MapId{2}};
        event.ResizeUpdatedPoints(1);
        event.updated_ids[0] = map::MapPointId{3};  // map 2's old point 2
        event.updated_positions.col(0) = Eigen::Vector3d(0, 0, 9).cast<precision_t>();
        event.updated_poses.emplace_back(map::FrameId{5}, MakePose(0, 1, 2, 3));
    });
    EXPECT_EQ(scene.map_ids(), std::vector<map::MapId>{map::MapId{1}});
    EXPECT_EQ(scene.current_map_id().value, 1u);
    ASSERT_EQ(scene.point_positions().size(), 4u);
    EXPECT_EQ(scene.num_alive_points(), 4u);
    EXPECT_TRUE(scene.point_positions()[2].isApprox(kThreePoints[0].cast<precision_t>()));
    EXPECT_TRUE(scene.point_positions()[3].isApprox(Eigen::Vector3d(0, 0, 9).cast<precision_t>()));
    ASSERT_EQ(scene.keyframes().size(), 2u);
    EXPECT_TRUE(scene.keyframes().at(map::FrameId{5}).translation().isApprox(
        Eigen::Vector3d(1, 2, 3).cast<precision_t>()));
}

// Without new_map an unknown map is not created, nor its active_map taken.
TEST(SceneStateTest, UnknownMapIgnored) {
    TOrbScene scene;
    AddPoints(scene, 1, 0, kThreePoints);
    EXPECT_TRUE(scene.map_ids().empty());
    EXPECT_EQ(scene.current_map_id().value, 0u);
    EXPECT_EQ(scene.num_alive_points(), 0u);
}

TEST(SceneStateTest, MapZeroCountedOnly) {
    TOrbScene scene;
    NewMap(scene, 1);
    AddPoints(scene, 1, 0, kThreePoints);
    auto event = std::make_shared<TOrbInit>();
    event->map_id = {0};
    scene.Apply(*event);
    EXPECT_EQ(scene.current_map_id().value, 1u);
    EXPECT_EQ(scene.num_alive_points(), 3u);
    EXPECT_EQ(scene.applied_count(EventType::InitializationMeasurement), 1u);
}

TEST(SceneStateTest, AppliesTransactionRemovals) {
    TOrbScene scene;
    ApplyTransaction(scene, 1, [](TOrbTransaction& event) {
        event.new_map = true;
        AddFrame(event, 0, MakePose(0, 0, 0, 0));
        AddFrame(event, 1, MakePose(0.1, 1, 0, 0));
        AddFrame(event, 2, MakePose(0.2, 2, 0, 0));
        event.ResizeNewPoints(4);
        for (Eigen::Index i = 0; i < 4; ++i) {
            event.new_positions.col(i) = Eigen::Vector3d(static_cast<double>(i), 0, 5).cast<precision_t>();
        }
    });
    ASSERT_EQ(scene.num_alive_points(), 4u);

    const auto moved = MakePose(0.3, 0, 1, 0);
    ApplyTransaction(scene, 1, [&](TOrbTransaction& event) {
        event.ResizeUpdatedPoints(1);
        event.updated_ids[0] = {3};
        event.updated_positions.col(0) = Eigen::Vector3d(9, 9, 9).cast<precision_t>();
        event.updated_poses = {{map::FrameId{1}, moved}};
        event.replaced_points = {{map::MapPointId{0}, map::MapPointId{3}}};
        event.removed_points = {map::MapPointId{1}, map::MapPointId{7}};  // 7 unknown
        event.removed_frames = {map::FrameId{2}, map::FrameId{5}};       // 5 unknown
    });

    EXPECT_EQ(scene.num_alive_points(), 2u);
    EXPECT_FALSE(scene.point_alive()[0]);  // the loser of a replacement
    EXPECT_FALSE(scene.point_alive()[1]);
    EXPECT_TRUE(scene.point_alive()[2]);
    EXPECT_TRUE(scene.point_alive()[3]);
    EXPECT_TRUE(scene.point_positions()[3].isApprox(Eigen::Vector3d(9, 9, 9).cast<precision_t>()));
    ASSERT_EQ(scene.keyframes().size(), 2u);
    EXPECT_EQ(scene.keyframes().count(map::FrameId{2}), 0u);
    EXPECT_TRUE(scene.keyframes().at(map::FrameId{1}).ToMatrix34().isApprox(moved.ToMatrix34()));

    // Removing again changes nothing.
    ApplyTransaction(scene, 1, [](TOrbTransaction& event) {
        event.removed_points = {map::MapPointId{1}};
        event.removed_frames = {map::FrameId{2}};
    });
    EXPECT_EQ(scene.num_alive_points(), 2u);
    EXPECT_EQ(scene.keyframes().size(), 2u);
}

} // namespace
} // namespace voyahviz
