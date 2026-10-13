#include <gtest/gtest.h>

#include <atomic>
#include <cstdint>
#include <memory>
#include <optional>
#include <thread>
#include <utility>
#include <vector>
#include <voyahslam/events/events.h>
#include <voyahslam/events/observable.h>
#include <voyahslam/events/subscription.h>
#include <voyahslam/features/orb_feature_extractor.h>
#include <voyahslam/mapping/map_follower.h>
#include <voyahslam/mapping/map_snapshot.h>

namespace voyahslam::mapping {
namespace {

using Extractor = features::OrbFeatureExtractor;
using TFollower = MapFollower<Extractor>;
using TTransaction = events::MapTransaction<Extractor>;
using TMono = features::TMonocularMeasurement<Extractor>;

// A mono measurement without features.
std::shared_ptr<const features::TMeasurement<Extractor>> EmptyMeasurement() {
    using TFeatures = features::ImageFeatures<Extractor::TDescriptorScalarType, Extractor::DescriptorSize>;
    TFeatures image_features(geometry::TPoints2DF::Zero(2, 0), geometry::TRowVectorF::Zero(0),
                             geometry::TRowVectorF::Zero(0), TFeatures::TDescriptors::Zero(Extractor::DescriptorSize, 0),
                             features::TLevels(0));
    return std::make_shared<const features::TMeasurement<Extractor>>(
        TMono(std::move(image_features), geometry::TMatrix33F::Identity(), nullptr, 1.0));
}

geometry::RigidTransformation Pose(double tx) {
    return geometry::RigidTransformation(geometry::TMatrix33F::Identity(),
                                         Eigen::Vector3d(tx, 0, 0).cast<precision_t>());
}

// Publishes MapTransactions the way the Mapper does, through an Observable.
class Publisher {
public:
    [[nodiscard]] events::Subscription Connect(TFollower& follower) {
        return observable_.ConnectAsync(follower.inbox());
    }

    // Map `map` gets Frame `frame` at x = `tx` and `num_points` points starting
    // at id `first_point`, point i at (first_point + i, 0, 5); `fill` sets the rest.
    template <typename TFill>
    void Publish(uint32_t map, uint32_t frame, double tx, uint32_t first_point, uint32_t num_points,
                 TFill fill) {
        auto event = std::make_shared<TTransaction>();
        event->map_id = map::MapId{map};
        events::NewFrame<Extractor>& added = event->added_frames.emplace_back();
        added.id = map::FrameId{frame};
        added.T_cw = Pose(tx);
        added.measurement = EmptyMeasurement();
        event->new_points_start_id = map::MapPointId{first_point};
        event->ResizeNewPoints(num_points);
        for (uint32_t i = 0; i < num_points; ++i) {
            event->new_positions.col(i) = Eigen::Vector3d(first_point + i, 0, 5).cast<precision_t>();
            event->new_descriptors.col(i).setZero();
            event->new_normals.col(i) = Eigen::Vector3d(0, 0, 1).cast<precision_t>();
        }
        event->active_map = map::MapId{map};
        fill(*event);
        observable_.Notify(std::move(event));
    }

    void NewMap(uint32_t map, uint32_t frame, uint32_t num_points) {
        Publish(map, frame, 0.0, 0, num_points, [&](TTransaction& event) {
            event.new_map = true;
            event.start_frame = map::FrameId{frame};
        });
    }

    void AddFrame(uint32_t map, uint32_t frame, double tx, uint32_t first_point, uint32_t num_points) {
        Publish(map, frame, tx, first_point, num_points, [](TTransaction&) {});
    }

    template <typename TEvent>
    void PublishOther(uint32_t map) {
        auto event = std::make_shared<TEvent>();
        event->map_id = map::MapId{map};
        observable_.Notify(std::move(event));
    }

private:
    events::Observable observable_;
};

TEST(MapFollowerTest, UpdateWithNothingPendingReturnsZero) {
    TFollower follower;
    EXPECT_EQ(follower.Update(), 0u);
    EXPECT_EQ(follower.num_transactions_applied(), 0u);
}

TEST(MapFollowerTest, SnapshotOfEmptyFollowerHasNoMaps) {
    TFollower follower;
    follower.Update();
    const std::shared_ptr<const MapSnapshot> snapshot = follower.Snapshot();
    ASSERT_NE(snapshot, nullptr);
    EXPECT_TRUE(snapshot->maps.empty());
    EXPECT_FALSE(snapshot->active_map.has_value());
    EXPECT_EQ(snapshot->num_transactions_applied, 0u);
    EXPECT_EQ(snapshot->Find(map::MapId{1}), nullptr);
}

TEST(MapFollowerTest, AppliesTransactionsOnlyOnUpdate) {
    TFollower follower;
    Publisher publisher;
    const events::Subscription subscription = publisher.Connect(follower);
    publisher.NewMap(1, 0, 3);
    publisher.AddFrame(1, 1, 1.0, 3, 2);

    EXPECT_TRUE(follower.atlas().map_ids().empty());
    EXPECT_EQ(follower.num_transactions_applied(), 0u);

    EXPECT_EQ(follower.Update(), 2u);
    EXPECT_EQ(follower.num_transactions_applied(), 2u);
    ASSERT_EQ(follower.atlas().map_ids(), std::vector<map::MapId>{map::MapId{1}});
    EXPECT_EQ(follower.atlas().active_map(), map::MapId{1});
    EXPECT_EQ(follower.atlas().map(map::MapId{1})->num_keyframes(), 2u);
    EXPECT_EQ(follower.Update(), 0u);
}

TEST(MapFollowerTest, IgnoresOtherEventTypes) {
    TFollower follower;
    Publisher publisher;
    const events::Subscription subscription = publisher.Connect(follower);
    publisher.PublishOther<events::FrameTracked<Extractor>>(1);
    publisher.PublishOther<events::TrackingLost>(1);
    publisher.PublishOther<events::InitializationMeasurement<Extractor>>(1);

    EXPECT_EQ(follower.inbox().size_approx(), 3u);
    EXPECT_EQ(follower.Update(), 0u);
    EXPECT_EQ(follower.num_transactions_applied(), 0u);
    EXPECT_EQ(follower.inbox().size_approx(), 0u);  // released, not left queued
    EXPECT_TRUE(follower.Snapshot()->maps.empty());
}

TEST(MapFollowerTest, SnapshotHoldsTheMapsContent) {
    TFollower follower;
    Publisher publisher;
    const events::Subscription subscription = publisher.Connect(follower);
    publisher.NewMap(1, 4, 3);
    publisher.AddFrame(1, 6, 2.0, 3, 1);
    follower.Update();

    const std::shared_ptr<const MapSnapshot> snapshot = follower.Snapshot();
    EXPECT_EQ(snapshot->num_transactions_applied, 2u);
    EXPECT_EQ(snapshot->active_map, map::MapId{1});
    ASSERT_EQ(snapshot->maps.size(), 1u);
    const MapSnapshot::Map* map = snapshot->Find(map::MapId{1});
    ASSERT_NE(map, nullptr);
    EXPECT_EQ(map, &snapshot->maps[0]);
    EXPECT_EQ(map->epoch, 0u);
    ASSERT_EQ(map->point_ids.size(), 4u);
    ASSERT_EQ(map->point_positions.size(), 4u);
    for (uint32_t i = 0; i < 4; ++i) {
        EXPECT_EQ(map->point_ids[i], map::MapPointId{i});
        EXPECT_FLOAT_EQ(map->point_positions[i].x(), i < 3 ? static_cast<float>(i) : 3.0f);
        EXPECT_FLOAT_EQ(map->point_positions[i].z(), 5.0f);
    }
    ASSERT_EQ(map->frame_ids, (std::vector<map::FrameId>{map::FrameId{4}, map::FrameId{6}}));
    ASSERT_EQ(map->frame_poses.size(), 2u);
    EXPECT_FLOAT_EQ(map->frame_poses[0].translation().x(), 0.0f);
    EXPECT_FLOAT_EQ(map->frame_poses[1].translation().x(), 2.0f);
}

TEST(MapFollowerTest, SnapshotIsImmutableAndOutlivesFollower) {
    auto follower = std::make_unique<TFollower>();
    Publisher publisher;
    const events::Subscription subscription = publisher.Connect(*follower);
    publisher.NewMap(1, 0, 2);
    follower->Update();
    const std::shared_ptr<const MapSnapshot> before = follower->Snapshot();

    publisher.AddFrame(1, 1, 1.0, 2, 3);
    publisher.NewMap(2, 2, 1);
    follower->Update();
    const std::shared_ptr<const MapSnapshot> after = follower->Snapshot();

    // The older snapshot did not move.
    EXPECT_EQ(before->num_transactions_applied, 1u);
    ASSERT_EQ(before->maps.size(), 1u);
    EXPECT_EQ(before->maps[0].point_ids.size(), 2u);
    EXPECT_EQ(before->maps[0].frame_ids.size(), 1u);
    EXPECT_EQ(before->active_map, map::MapId{1});
    EXPECT_EQ(after->num_transactions_applied, 3u);
    ASSERT_EQ(after->maps.size(), 2u);
    EXPECT_EQ(after->maps[0].point_ids.size(), 5u);
    EXPECT_EQ(after->active_map, map::MapId{2});
    EXPECT_NE(before.get(), after.get());

    follower.reset();
    EXPECT_EQ(after->maps[0].point_ids.size(), 5u);
    EXPECT_EQ(before->maps[0].point_ids.size(), 2u);
}

TEST(MapFollowerTest, SnapshotReadOnAnotherThread) {
    TFollower follower;
    Publisher publisher;
    const events::Subscription subscription = publisher.Connect(follower);
    publisher.NewMap(1, 0, 50);
    follower.Update();
    const std::shared_ptr<const MapSnapshot> snapshot = follower.Snapshot();

    std::atomic<bool> done{false};
    double checksum = 0;
    std::thread reader([&] {
        for (int round = 0; round < 200; ++round) {
            checksum = 0;
            for (const MapSnapshot::Map& map : snapshot->maps) {
                for (const auto& position : map.point_positions) checksum += position.x();
                for (const auto& pose : map.frame_poses) checksum += pose.translation().x();
            }
        }
        done = true;
    });
    // The owner keeps updating meanwhile.
    for (uint32_t frame = 1; !done && frame < 400; ++frame) {
        publisher.AddFrame(1, frame, frame, 50 + frame, 1);
        follower.Update();
    }
    reader.join();
    EXPECT_DOUBLE_EQ(checksum, 49.0 * 50.0 / 2.0);  // points 0..49 at x = i, pose x = 0
    EXPECT_EQ(snapshot->maps[0].point_ids.size(), 50u);
}

TEST(MapFollowerTest, RemovedMapLeavesTheSnapshot) {
    TFollower follower;
    Publisher publisher;
    const events::Subscription subscription = publisher.Connect(follower);
    publisher.NewMap(1, 0, 2);
    publisher.NewMap(2, 1, 2);
    follower.Update();
    ASSERT_EQ(follower.Snapshot()->maps.size(), 2u);

    publisher.Publish(2, 2, 1.0, 2, 1, [](TTransaction& event) { event.removed_maps.push_back(map::MapId{1}); });
    EXPECT_EQ(follower.Update(), 1u);
    const std::shared_ptr<const MapSnapshot> snapshot = follower.Snapshot();
    ASSERT_EQ(snapshot->maps.size(), 1u);
    EXPECT_EQ(snapshot->maps[0].id, map::MapId{2});
    EXPECT_EQ(snapshot->Find(map::MapId{1}), nullptr);
    EXPECT_EQ(snapshot->active_map, map::MapId{2});
}

}  // namespace
}  // namespace voyahslam::mapping
