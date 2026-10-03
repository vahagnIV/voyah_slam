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

} // namespace
} // namespace voyahviz
