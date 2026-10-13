#include <gtest/gtest.h>

#include <Eigen/Geometry>
#include <algorithm>
#include <memory>
#include <set>
#include <utility>
#include <vector>
#include <voyahslam/events/events.h>
#include <voyahslam/features/orb_feature_extractor.h>
#include <voyahslam/tracking/map_replica.h>

namespace voyahslam::tracking {
namespace {

using Extractor = features::OrbFeatureExtractor;
using TReplica = MapReplica<Extractor>;
using TTransaction = events::MapTransaction<Extractor>;
using TMono = features::TMonocularMeasurement<Extractor>;

geometry::RigidTransformation MakePose(double angle, double tx, double ty, double tz) {
    const Eigen::Matrix3d rotation =
        Eigen::AngleAxisd(angle, Eigen::Vector3d::UnitY()).toRotationMatrix();
    return geometry::RigidTransformation(rotation.cast<precision_t>(),
                                         Eigen::Vector3d(tx, ty, tz).cast<precision_t>());
}

// Applies one transaction of map `map`, filled by `fill`.
template <typename TFill>
void ApplyTransaction(TReplica& out_replica, uint32_t map, TFill fill) {
    auto event = std::make_shared<TTransaction>();
    event->map_id = {map};
    fill(*event);
    out_replica.Apply(*event);
}

// Sets the new-points part of `out_event`: ids [start, start + points.size()),
// point i with position points[i], a descriptor filled with i + 1 and normal (0, 0, i + 1).
void SetNewPoints(TTransaction& out_event, uint32_t start,
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

// Adds Frame `frame` to `out_event`: feature i at level levels[i], and its own
// (feature, point) observations.
void AddFrame(TTransaction& out_event, uint32_t frame, const geometry::RigidTransformation& pose,
              const std::vector<uint8_t>& levels,
              const std::vector<std::pair<uint32_t, uint32_t>>& observations) {
    events::NewFrame<Extractor>& added = out_event.added_frames.emplace_back();
    added.id = {frame};
    added.T_cw = pose;
    added.measurement = std::make_shared<const features::TMeasurement<Extractor>>(MakeMeasurement(levels));
    for (const auto& [feature, point] : observations) {
        added.observed_features.push_back(feature);
        added.observed_points.push_back({point});
    }
}

// Applies a transaction that adds Frame `frame` and `new_points` from id `start`.
void CreateKeyFrame(TReplica& out_replica, uint32_t map, uint32_t frame,
                    const geometry::RigidTransformation& pose, const std::vector<uint8_t>& levels,
                    const std::vector<std::pair<uint32_t, uint32_t>>& observations,
                    const std::vector<Eigen::Vector3d>& new_points = {}, uint32_t start = 0) {
    ApplyTransaction(out_replica, map, [&](TTransaction& event) {
        AddFrame(event, frame, pose, levels, observations);
        SetNewPoints(event, start, new_points);
    });
}

// Adds points through a transaction whose Frame (`frame`, no features)
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

    ApplyTransaction(replica, 1, [](TTransaction& up) {
        up.ResizeUpdatedPoints(2);
        up.updated_ids[0] = {1};
        up.updated_ids[1] = {7};
        up.updated_positions.col(0) = Eigen::Vector3d(10, 20, 30).cast<precision_t>();
        up.updated_positions.col(1) = Eigen::Vector3d(-1, -1, -1).cast<precision_t>();
        up.updated_descriptors.col(0).setConstant(77);
        up.updated_descriptors.col(1).setConstant(88);
        up.updated_normals.col(0) = Eigen::Vector3d(0, 1, 0).cast<precision_t>();
        up.updated_normals.col(1) = Eigen::Vector3d(0, 1, 0).cast<precision_t>();
    });
    EXPECT_TRUE(replica.Position({1}).isApprox(Eigen::Vector3d(10, 20, 30).cast<precision_t>()));
    EXPECT_EQ(replica.Descriptor({1})(0), 77);
    EXPECT_NEAR(replica.Normal({1}).y(), 1.0, 1e-6);
    EXPECT_EQ(replica.num_points(), 3u);
    EXPECT_FALSE(replica.IsAlive({7}));

    ApplyTransaction(replica, 1, [](TTransaction& rm) { rm.removed_points = {{0}, {9}}; });
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

TEST(MapReplicaTest, StoresFrameBow) {
    TReplica replica;
    ApplyTransaction(replica, 1, [&](TTransaction& event) {
        AddFrame(event, 4, MakePose(0.1, 0, 0, 0), {0, 1}, {});
        AddFrame(event, 5, MakePose(0.2, 0, 0, 0), {0}, {});
        events::NewFrame<Extractor>& first = event.added_frames[0];
        first.bow.entries = {{3, 0.25}, {7, 0.75}};
        first.features.nodes = {2, 9};
        first.features.offsets = {0, 1, 2};
        first.features.features = {0, 1};
    });
    const auto* with = replica.keyframe({4});
    ASSERT_NE(with, nullptr);
    EXPECT_EQ(with->bow.entries, (std::vector<std::pair<bow::WordId, double>>{{3, 0.25}, {7, 0.75}}));
    EXPECT_EQ(with->features.nodes, (std::vector<bow::NodeId>{2, 9}));
    EXPECT_EQ(with->features.offsets, (std::vector<uint32_t>{0, 1, 2}));
    EXPECT_EQ(with->features.features, (std::vector<uint32_t>{0, 1}));
    const auto* without = replica.keyframe({5});
    ASSERT_NE(without, nullptr);
    EXPECT_TRUE(without->bow.entries.empty());
    EXPECT_TRUE(without->features.nodes.empty());
}

TEST(MapReplicaTest, DatabaseFollowsTransactions) {
    TReplica replica;
    ApplyTransaction(replica, 1, [&](TTransaction& event) {
        AddFrame(event, 4, MakePose(0.1, 0, 0, 0), {0}, {});
        AddFrame(event, 5, MakePose(0.2, 0, 0, 0), {0}, {});
        AddFrame(event, 6, MakePose(0.3, 0, 0, 0), {0}, {});  // no vocabulary
        event.added_frames[0].bow.entries = {{3, 0.5}, {7, 0.5}};
        event.added_frames[1].bow.entries = {{3, 0.5}, {8, 0.5}};
    });
    EXPECT_EQ(replica.database().size(), 2u);
    EXPECT_FALSE(replica.database().Contains({6}));
    bow::BowVector query;
    query.entries = {{3, 1.0}};
    EXPECT_EQ(replica.database().Query(query).size(), 2u);

    ApplyTransaction(replica, 1, [&](TTransaction& event) { event.removed_frames.push_back({4}); });
    const std::vector<bow::DatabaseHit> hits = replica.database().Query(query);
    ASSERT_EQ(hits.size(), 1u);
    EXPECT_EQ(hits[0].frame, map::FrameId{5});

    ApplyTransaction(replica, 1, [&](TTransaction& event) {
        event.new_map = true;
        AddFrame(event, 9, MakePose(0.0, 0, 0, 0), {0}, {});
        event.added_frames[0].bow.entries = {{3, 1.0}};
    });
    EXPECT_EQ(replica.database().size(), 1u);
    EXPECT_TRUE(replica.database().Contains({9}));
    EXPECT_FALSE(replica.database().Contains({5}));

    ApplyTransaction(replica, 2, [&](TTransaction& event) { event.new_map = true; });  // a new map clears
    EXPECT_EQ(replica.database().size(), 0u);
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

TEST(MapReplicaTest, UpdatedPoses) {
    TReplica replica;
    AddPoints(replica, 1, 0, kThreePoints);
    CreateKeyFrame(replica, 1, 3, MakePose(0.1, 1, 2, 3), {0, 0, 0}, {{0, 0}});
    const auto updated_pose = MakePose(0.4, 5, 6, 7);
    ApplyTransaction(replica, 1, [&](TTransaction& event) {
        event.updated_poses = {{map::FrameId{3}, updated_pose},
                               {map::FrameId{99}, updated_pose}};  // unknown keyframe: ignored
    });
    EXPECT_TRUE(replica.keyframe({3})->T_cw.ToMatrix34().isApprox(updated_pose.ToMatrix34()));
    EXPECT_EQ(replica.keyframe({99}), nullptr);
}

// A new map clears the replica and gives it its id (map ids: AtlasReplicaTest).
TEST(MapReplicaTest, NewMapClears) {
    TReplica replica;
    AddPoints(replica, 1, 0, kThreePoints);
    CreateKeyFrame(replica, 1, 3, MakePose(0.1, 1, 2, 3), {0, 0, 0}, {{0, 0}});
    ASSERT_EQ(replica.num_points(), 3u);

    ApplyTransaction(replica, 2, [](TTransaction& created) {
        created.new_map = true;
        created.start_frame = map::FrameId{10};
    });
    EXPECT_EQ(replica.map_id().value, 2u);
    EXPECT_EQ(replica.num_points(), 0u);
    EXPECT_TRUE(replica.keyframe_ids().empty());

    AddPoints(replica, 2, 0, {{1, 1, 1}});
    EXPECT_EQ(replica.num_points(), 1u);
}

TEST(MapReplicaTest, AppliesFrameInsertionAtomically) {
    TReplica replica;
    CreateKeyFrame(replica, 1, 2, MakePose(0.0, 0, 0, 0), {0, 0, 0, 0}, {{0, 0}, {1, 1}, {2, 2}},
                   kThreePoints);

    // Frame 5 adds points 3-5, updates point 1, adds an observation of new
    // point 4 to existing Frame 2, and observes points 1, 3 and 5 itself.
    ApplyTransaction(replica, 1, [](TTransaction& event) {
        AddFrame(event, 5, MakePose(0.1, 1, 0, 0), {0, 1, 2}, {{0, 1}, {1, 3}, {2, 5}});
        SetNewPoints(event, 3, {{1, 0, 4}, {2, 0, 4}, {3, 0, 4}});
        event.ResizeUpdatedPoints(1);
        event.updated_ids[0] = {1};
        event.updated_positions.col(0) = Eigen::Vector3d(4, 5, 7).cast<precision_t>();
        event.updated_descriptors.col(0).setConstant(42);
        event.updated_normals.col(0) = Eigen::Vector3d(1, 0, 0).cast<precision_t>();
        event.added_observations.push_back({map::FrameId{2}, 3, map::MapPointId{4}});
    });
    EXPECT_TRUE(replica.CheckConsistency());

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

    ApplyTransaction(replica, 1, [](TTransaction& event) {
        AddFrame(event, 1, MakePose(0.1, 1, 0, 0), {0}, {{0, 2}});
        event.ResizeUpdatedPoints(2);
        event.updated_ids[0] = {2};
        event.updated_ids[1] = {0};
        for (Eigen::Index i = 0; i < 2; ++i) {
            event.updated_positions.col(i) =
                kThreePoints[static_cast<size_t>(2 - 2 * i)].cast<precision_t>();
            event.updated_descriptors.col(i).setConstant(static_cast<uint8_t>(50 + i));
            event.updated_normals.col(i) = Eigen::Vector3d(0, 1 + i, 0).cast<precision_t>();
        }
    });

    EXPECT_EQ(replica.Descriptor({2})(0), 50);
    EXPECT_NEAR(replica.Normal({2}).y(), 1.0, 1e-6);
    EXPECT_EQ(replica.Descriptor({0})(31), 51);
    EXPECT_NEAR(replica.Normal({0}).y(), 2.0, 1e-6);
    EXPECT_EQ(replica.Descriptor({1})(0), 2);  // untouched
    EXPECT_EQ(replica.num_points(), 3u);
}

using TObservations = std::vector<std::pair<uint32_t, map::MapPointId>>;

// The reference of `point` must be (frame, feature) at that feature's level,
// at the point's distance from the Frame's current centre.
void ExpectReference(const TReplica& replica, uint32_t point, uint32_t frame, uint32_t feature,
                     int level) {
    const auto& reference = replica.Reference({point});
    ASSERT_TRUE(reference.has_value()) << "point " << point;
    EXPECT_EQ(reference->keyframe.value, frame) << "point " << point;
    EXPECT_EQ(reference->feature, static_cast<Eigen::Index>(feature)) << "point " << point;
    EXPECT_EQ(reference->level, level) << "point " << point;
    const auto* keyframe = replica.keyframe({frame});
    ASSERT_NE(keyframe, nullptr);
    const geometry::TPoint3DF centre = keyframe->T_cw.Inverse().translation();
    EXPECT_NEAR(reference->distance, (replica.Position({point}) - centre).norm(), 1e-9)
        << "point " << point;
}

TEST(MapReplicaTest, AppliesEveryTransactionPart) {
    TReplica replica;
    // A new map: Frames 0-2 and points 0-4.
    ApplyTransaction(replica, 1, [](TTransaction& event) {
        event.new_map = true;
        event.start_frame = map::FrameId{1};
        AddFrame(event, 0, MakePose(0.0, 0, 0, 0), {0, 1, 2, 3}, {{0, 0}, {1, 1}, {2, 2}, {3, 3}});
        AddFrame(event, 1, MakePose(0.1, 1, 0, 0), {0, 1, 2, 3}, {{0, 0}, {1, 1}, {2, 2}, {3, 4}});
        AddFrame(event, 2, MakePose(0.2, 2, 0, 0), {0, 1, 2, 3}, {{0, 3}, {1, 4}});
        SetNewPoints(event, 0, {{0, 0, 5}, {1, 0, 5}, {2, 0, 5}, {3, 0, 5}, {4, 0, 5}});
    });
    ASSERT_TRUE(replica.CheckConsistency());
    ExpectReference(replica, 2, 0, 2, 2);

    // Every part at once.
    const auto pose_1 = MakePose(0.3, 1, 1, 0);
    ApplyTransaction(replica, 1, [&](TTransaction& event) {
        AddFrame(event, 3, MakePose(0.0, 3, 0, 0), {0, 1, 2}, {{0, 1}, {1, 5}});
        SetNewPoints(event, 5, {{5, 0, 5}, {6, 0, 5}});
        // Point 6 is first observed by Frame 2, which this transaction removes.
        event.added_observations = {{map::FrameId{2}, 2, map::MapPointId{5}},
                                    {map::FrameId{2}, 3, map::MapPointId{6}},
                                    {map::FrameId{3}, 2, map::MapPointId{6}}};
        event.ResizeUpdatedPoints(1);
        event.updated_ids[0] = {0};
        event.updated_positions.col(0) = Eigen::Vector3d(0, 1, 6).cast<precision_t>();
        event.updated_descriptors.col(0).setConstant(99);
        event.updated_normals.col(0) = Eigen::Vector3d(0, 1, 0).cast<precision_t>();
        event.updated_poses = {{map::FrameId{1}, pose_1}};
        event.removed_observations = {{map::FrameId{0}, map::MapPointId{2}}};
        event.replaced_points = {{map::MapPointId{4}, map::MapPointId{3}}};
        event.removed_points = {map::MapPointId{1}};
        event.removed_frames = {map::FrameId{2}};
    });

    EXPECT_TRUE(replica.CheckConsistency());
    EXPECT_EQ(replica.num_points(), 7u);
    for (const uint32_t alive : {0u, 2u, 3u, 5u, 6u}) EXPECT_TRUE(replica.IsAlive({alive})) << alive;
    EXPECT_FALSE(replica.IsAlive({1}));  // removed
    EXPECT_FALSE(replica.IsAlive({4}));  // the loser of a replacement
    EXPECT_EQ(replica.keyframe_ids(),
              (std::vector<map::FrameId>{map::FrameId{0}, map::FrameId{1}, map::FrameId{3}}));
    EXPECT_EQ(replica.keyframe({2}), nullptr);
    EXPECT_TRUE(replica.Position({0}).isApprox(Eigen::Vector3d(0, 1, 6).cast<precision_t>()));
    EXPECT_EQ(replica.Descriptor({0})(0), 99);
    EXPECT_NEAR(replica.Normal({0}).y(), 1.0, 1e-9);
    EXPECT_TRUE(replica.keyframe({1})->T_cw.ToMatrix34().isApprox(pose_1.ToMatrix34()));

    EXPECT_EQ(replica.keyframe({0})->observations, (TObservations{{0, {0}}, {3, {3}}}));
    EXPECT_EQ(replica.keyframe({1})->observations, (TObservations{{0, {0}}, {2, {2}}}));
    EXPECT_EQ(replica.keyframe({3})->observations, (TObservations{{1, {5}}, {2, {6}}}));
    EXPECT_EQ(replica.covisibility().Covisible({0}, 10),
              (std::vector<std::pair<map::FrameId, uint32_t>>{{map::FrameId{1}, 1}}));
    EXPECT_TRUE(replica.covisibility().Points({2}).empty());
    EXPECT_TRUE(replica.covisibility().Observations({4}).empty());

    // Point 2 lost its reference observation (Frame 0): Frame 1 takes over.
    // Point 5's reference is the new Frame's own observation; point 6's
    // (Frame 2) is gone with its Frame.
    ExpectReference(replica, 2, 1, 2, 2);
    ExpectReference(replica, 3, 0, 3, 3);
    ExpectReference(replica, 5, 3, 1, 1);
    ExpectReference(replica, 6, 3, 2, 2);
    EXPECT_FALSE(replica.Reference({1}).has_value());
    EXPECT_FALSE(replica.Reference({4}).has_value());
}

TEST(MapReplicaTest, ReferenceReassignedOnFrameRemoval) {
    TReplica replica;
    // Point 0 is referenced in Frame 3 and also observed by Frames 7 and 5
    // (added in that order).
    ApplyTransaction(replica, 1, [](TTransaction& event) {
        event.new_map = true;
        AddFrame(event, 3, MakePose(0.0, 0, 0, 0), {1}, {{0, 0}});
        AddFrame(event, 7, MakePose(0.1, 2, 0, 0), {0, 2}, {{1, 0}});
        AddFrame(event, 5, MakePose(0.1, 1, 0, 0), {0, 0, 4}, {{2, 0}});
        SetNewPoints(event, 0, {{1, 2, 8}});
    });
    ExpectReference(replica, 0, 3, 0, 1);

    // Frame 5 moves in the same transaction: the distance is from its new centre.
    const auto moved = MakePose(0.2, -3, 1, 2);
    ApplyTransaction(replica, 1, [&](TTransaction& event) {
        event.updated_poses = {{map::FrameId{5}, moved}};
        event.removed_frames = {map::FrameId{3}};
    });

    EXPECT_TRUE(replica.CheckConsistency());
    ASSERT_TRUE(replica.IsAlive({0}));
    ExpectReference(replica, 0, 5, 2, 4);
    const Eigen::Vector3d centre = moved.Inverse().translation().cast<double>();
    EXPECT_NEAR(replica.Reference({0})->distance, (Eigen::Vector3d(1, 2, 8) - centre).norm(), 1e-9);
}

TEST(MapReplicaTest, PointWithoutObservationsIsRemoved) {
    TReplica replica;
    ApplyTransaction(replica, 1, [](TTransaction& event) {
        event.new_map = true;
        AddFrame(event, 1, MakePose(0.0, 0, 0, 0), {0, 0}, {{0, 0}, {1, 1}});
        AddFrame(event, 2, MakePose(0.1, 1, 0, 0), {0}, {{0, 0}});
        SetNewPoints(event, 0, {{0, 0, 5}, {1, 0, 5}});
    });
    ASSERT_TRUE(replica.CheckConsistency());

    // Point 1's only observation goes.
    ApplyTransaction(replica, 1, [](TTransaction& event) {
        event.removed_observations = {{map::FrameId{1}, map::MapPointId{1}}};
    });
    EXPECT_TRUE(replica.CheckConsistency());
    EXPECT_FALSE(replica.IsAlive({1}));
    EXPECT_FALSE(replica.Reference({1}).has_value());
    EXPECT_TRUE(replica.IsAlive({0}));

    // Point 0's two Frames go.
    ApplyTransaction(replica, 1, [](TTransaction& event) {
        event.removed_frames = {map::FrameId{2}, map::FrameId{1}};
    });
    EXPECT_TRUE(replica.CheckConsistency());
    EXPECT_FALSE(replica.IsAlive({0}));
    EXPECT_TRUE(replica.keyframe_ids().empty());
    EXPECT_TRUE(replica.covisibility().Observations({0}).empty());
}

TEST(MapReplicaTest, StoresParentsAndLoopEdges) {
    using map::FrameId;
    using Edges = std::set<FrameId>;
    TReplica replica;
    // Tree 0 <- 1 <- {2, 3}, 0 <- 4; loop edge 3 - 0. Frame 5's parent and
    // the loop edge to Frame 9 are unknown: skipped.
    ApplyTransaction(replica, 1, [](TTransaction& event) {
        event.new_map = true;
        for (uint32_t frame = 0; frame < 6; ++frame) {
            AddFrame(event, frame, MakePose(0.0, frame, 0, 0), {0}, {});
        }
        event.added_frames[1].parent = FrameId{0};
        event.added_frames[2].parent = FrameId{1};
        event.added_frames[3].parent = FrameId{1};
        event.added_frames[4].parent = FrameId{0};
        event.added_frames[5].parent = FrameId{7};
        event.added_loop_edges = {{FrameId{3}, FrameId{0}}, {FrameId{2}, FrameId{9}}};
    });
    EXPECT_TRUE(replica.CheckConsistency());
    EXPECT_EQ(replica.map_epoch(), 0u);
    EXPECT_FALSE(replica.parent(FrameId{0}).has_value());
    EXPECT_EQ(replica.parent(FrameId{1}), FrameId{0});
    EXPECT_EQ(replica.parent(FrameId{3}), FrameId{1});
    EXPECT_FALSE(replica.parent(FrameId{5}).has_value());
    EXPECT_EQ(replica.children(FrameId{0}), (Edges{FrameId{1}, FrameId{4}}));
    EXPECT_EQ(replica.children(FrameId{1}), (Edges{FrameId{2}, FrameId{3}}));
    EXPECT_EQ(replica.loop_edges(FrameId{0}), Edges{FrameId{3}});
    EXPECT_EQ(replica.loop_edges(FrameId{3}), Edges{FrameId{0}});
    EXPECT_TRUE(replica.loop_edges(FrameId{2}).empty());

    // Frame 1 culled: its children re-parented, 3 to 0 and then 2 to 3,
    // before it is removed; a loop edge 2 - 4 and a new epoch.
    ApplyTransaction(replica, 1, [](TTransaction& event) {
        event.map_epoch = 2;
        event.parent_changes = {{FrameId{3}, FrameId{0}}, {FrameId{2}, FrameId{3}}};
        event.added_loop_edges = {{FrameId{2}, FrameId{4}}};
        event.removed_frames = {FrameId{1}};
    });
    EXPECT_TRUE(replica.CheckConsistency());
    EXPECT_EQ(replica.map_epoch(), 2u);
    EXPECT_EQ(replica.parent(FrameId{3}), FrameId{0});
    EXPECT_EQ(replica.parent(FrameId{2}), FrameId{3});
    EXPECT_EQ(replica.children(FrameId{0}), (Edges{FrameId{3}, FrameId{4}}));
    EXPECT_EQ(replica.children(FrameId{3}), Edges{FrameId{2}});
    EXPECT_FALSE(replica.parent(FrameId{1}).has_value());
    EXPECT_TRUE(replica.children(FrameId{1}).empty());
    EXPECT_EQ(replica.loop_edges(FrameId{2}), Edges{FrameId{4}});
    EXPECT_EQ(replica.loop_edges(FrameId{4}), Edges{FrameId{2}});

    // Frame 3 removed with no parent change: its loop edge goes from Frame 0,
    // it leaves its parent's children, and its child is left without a parent.
    ApplyTransaction(replica, 1, [](TTransaction& event) { event.removed_frames = {FrameId{3}}; });
    EXPECT_TRUE(replica.CheckConsistency());
    EXPECT_TRUE(replica.loop_edges(FrameId{0}).empty());
    EXPECT_EQ(replica.children(FrameId{0}), Edges{FrameId{4}});
    EXPECT_FALSE(replica.parent(FrameId{2}).has_value());

    // A new map clears the graph and the epoch.
    ApplyTransaction(replica, 1, [](TTransaction& event) { event.map_epoch = 3; });
    EXPECT_EQ(replica.map_epoch(), 3u);
    ApplyTransaction(replica, 2, [](TTransaction& event) {
        event.new_map = true;
        AddFrame(event, 10, MakePose(0.0, 0, 0, 0), {0}, {});
    });
    EXPECT_TRUE(replica.CheckConsistency());
    EXPECT_EQ(replica.map_epoch(), 0u);
    EXPECT_TRUE(replica.children(FrameId{0}).empty());
    EXPECT_TRUE(replica.loop_edges(FrameId{2}).empty());
    EXPECT_FALSE(replica.parent(FrameId{10}).has_value());
}

TEST(MapReplicaTest, CheckConsistencyDetectsAnUnobservedPoint) {
    TReplica replica;
    CreateKeyFrame(replica, 1, 0, MakePose(0.0, 0, 0, 0), {0}, {{0, 0}}, kThreePoints);
    EXPECT_FALSE(replica.CheckConsistency());  // points 1 and 2 have no observation
    ApplyTransaction(replica, 1, [](TTransaction& event) {
        event.removed_points = {map::MapPointId{1}, map::MapPointId{2}};
    });
    EXPECT_TRUE(replica.CheckConsistency());
}

} // namespace
} // namespace voyahslam::tracking
