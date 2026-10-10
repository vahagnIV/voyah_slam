#include <gtest/gtest.h>

#include <memory>
#include <optional>
#include <vector>
#include <voyahslam/events/events.h>
#include <voyahslam/features/orb_feature_extractor.h>
#include <voyahslam/memory/object_pool.h>
#include <voyahslam/tracking/atlas_replica.h>

namespace voyahslam::tracking {
namespace {

using Extractor = features::OrbFeatureExtractor;
using TAtlas = AtlasReplica<Extractor>;
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

// Applies one transaction of map `map` that adds Frame `frame` (if any) and
// names `active` as the active map; `fill` sets the rest.
template <typename TFill>
void Apply(TAtlas& out_atlas, uint32_t map, std::optional<uint32_t> frame, std::optional<uint32_t> active,
           TFill fill) {
    memory::ObjectPool<TTransaction, memory::ThreadSafePolicy> pool(1);
    TTransaction* event = pool.Create();
    event->map_id = map::MapId{map};
    if (frame) {
        events::NewFrame<Extractor>& added = event->added_frames.emplace_back();
        added.id = map::FrameId{*frame};
        added.measurement = EmptyMeasurement();
    }
    if (active) event->active_map = map::MapId{*active};
    fill(*event);
    out_atlas.Apply(*event);
    event->Recycle();
}

// The Mapper's new map `map` with its first Frame `frame`; active.
void NewMap(TAtlas& out_atlas, uint32_t map, uint32_t frame) {
    Apply(out_atlas, map, frame, map, [&](TTransaction& event) {
        event.new_map = true;
        event.start_frame = map::FrameId{frame};
    });
}

// Adds Frame `frame` to map `map`; `active` stays the active map.
void AddFrame(TAtlas& out_atlas, uint32_t map, uint32_t frame, std::optional<uint32_t> active) {
    Apply(out_atlas, map, frame, active, [](TTransaction&) {});
}

// The Mapper's answer to a final loss of map `map`: removed, none active.
void RemoveMap(TAtlas& out_atlas, uint32_t map) {
    Apply(out_atlas, map, std::nullopt, std::nullopt,
          [&](TTransaction& event) { event.removed_maps.push_back(map::MapId{map}); });
}

TEST(AtlasReplicaTest, StartsEmpty) {
    const TAtlas atlas;
    EXPECT_FALSE(atlas.active_map().has_value());
    EXPECT_EQ(atlas.active(), nullptr);
    EXPECT_EQ(atlas.map(map::MapId{1}), nullptr);
    EXPECT_TRUE(atlas.map_ids().empty());
}

TEST(AtlasReplicaTest, NewMapCreatesAndActivates) {
    TAtlas atlas;
    NewMap(atlas, 1, 0);
    ASSERT_EQ(atlas.active_map(), map::MapId{1});
    ASSERT_NE(atlas.active(), nullptr);
    EXPECT_EQ(atlas.active(), atlas.map(map::MapId{1}));
    EXPECT_EQ(atlas.active()->map_id(), map::MapId{1});
    EXPECT_EQ(atlas.active()->keyframe_ids(), std::vector<map::FrameId>{map::FrameId{0}});
    EXPECT_EQ(atlas.map_ids(), std::vector<map::MapId>{map::MapId{1}});

    AddFrame(atlas, 1, 1, 1);
    EXPECT_EQ(atlas.active()->num_keyframes(), 2u);
}

TEST(AtlasReplicaTest, IgnoresUnknownAndRemovedMaps) {
    TAtlas atlas;
    // Unknown: without new_map a map is never created, nor its active_map taken.
    AddFrame(atlas, 1, 0, 1);
    EXPECT_TRUE(atlas.map_ids().empty());
    EXPECT_FALSE(atlas.active_map().has_value());
    // Map 0 is no map, even with new_map.
    NewMap(atlas, 0, 1);
    EXPECT_TRUE(atlas.map_ids().empty());
    EXPECT_FALSE(atlas.active_map().has_value());

    NewMap(atlas, 1, 2);
    RemoveMap(atlas, 1);
    EXPECT_TRUE(atlas.map_ids().empty());
    EXPECT_EQ(atlas.map(map::MapId{1}), nullptr);
    EXPECT_FALSE(atlas.active_map().has_value());

    // A late transaction of the removed map changes nothing, its active_map
    // included.
    AddFrame(atlas, 1, 3, 1);
    EXPECT_TRUE(atlas.map_ids().empty());
    EXPECT_FALSE(atlas.active_map().has_value());

    NewMap(atlas, 2, 4);
    AddFrame(atlas, 1, 5, 1);
    EXPECT_EQ(atlas.map_ids(), std::vector<map::MapId>{map::MapId{2}});
    EXPECT_EQ(atlas.active_map(), map::MapId{2});
    EXPECT_EQ(atlas.active()->keyframe_ids(), std::vector<map::FrameId>{map::FrameId{4}});
}

TEST(AtlasReplicaTest, RemovedMapNeverReadmitted) {
    TAtlas atlas;
    NewMap(atlas, 1, 0);
    RemoveMap(atlas, 1);
    NewMap(atlas, 1, 1);
    EXPECT_TRUE(atlas.map_ids().empty());
    EXPECT_EQ(atlas.map(map::MapId{1}), nullptr);
    EXPECT_FALSE(atlas.active_map().has_value());

    // Also a map removed by a transaction of another map.
    NewMap(atlas, 2, 2);
    NewMap(atlas, 3, 3);
    Apply(atlas, 3, std::nullopt, 3, [](TTransaction& event) { event.removed_maps.push_back(map::MapId{2}); });
    NewMap(atlas, 2, 4);
    EXPECT_EQ(atlas.map_ids(), std::vector<map::MapId>{map::MapId{3}});
    EXPECT_EQ(atlas.active_map(), map::MapId{3});
}

TEST(AtlasReplicaTest, ActiveMapOnlyFromTransaction) {
    TAtlas atlas;
    NewMap(atlas, 1, 0);
    // A transaction of the active map that names none deactivates it, and
    // keeps it.
    AddFrame(atlas, 1, 1, std::nullopt);
    EXPECT_FALSE(atlas.active_map().has_value());
    EXPECT_EQ(atlas.active(), nullptr);
    ASSERT_NE(atlas.map(map::MapId{1}), nullptr);
    EXPECT_EQ(atlas.map(map::MapId{1})->num_keyframes(), 2u);

    // A new map without active_map is created, not activated: not the newest
    // id, only active_map activates.
    Apply(atlas, 2, 2, std::nullopt, [](TTransaction& event) { event.new_map = true; });
    EXPECT_FALSE(atlas.active_map().has_value());
    EXPECT_EQ(atlas.map_ids(), (std::vector<map::MapId>{map::MapId{1}, map::MapId{2}}));

    // A transaction of map 2 may activate map 1.
    AddFrame(atlas, 2, 3, 1);
    EXPECT_EQ(atlas.active_map(), map::MapId{1});
    EXPECT_EQ(atlas.active(), atlas.map(map::MapId{1}));
}

TEST(AtlasReplicaTest, KeepsInactiveMaps) {
    TAtlas atlas;
    NewMap(atlas, 1, 0);
    AddFrame(atlas, 1, 1, 1);
    // Map 1 kept inactive (Task 3's Atlas): map 2 starts without removing it.
    NewMap(atlas, 2, 2);
    EXPECT_EQ(atlas.active_map(), map::MapId{2});
    EXPECT_EQ(atlas.map_ids(), (std::vector<map::MapId>{map::MapId{1}, map::MapId{2}}));
    ASSERT_NE(atlas.map(map::MapId{1}), nullptr);
    EXPECT_EQ(atlas.map(map::MapId{1})->num_keyframes(), 2u);
    EXPECT_EQ(atlas.map(map::MapId{1})->map_id(), map::MapId{1});
    EXPECT_EQ(atlas.active()->keyframe_ids(), std::vector<map::FrameId>{map::FrameId{2}});

    // Each map takes only its own transactions.
    AddFrame(atlas, 1, 3, 2);
    AddFrame(atlas, 2, 4, 2);
    EXPECT_EQ(atlas.map(map::MapId{1})->keyframe_ids(),
              (std::vector<map::FrameId>{map::FrameId{0}, map::FrameId{1}, map::FrameId{3}}));
    EXPECT_EQ(atlas.map(map::MapId{2})->keyframe_ids(),
              (std::vector<map::FrameId>{map::FrameId{2}, map::FrameId{4}}));
}

} // namespace
} // namespace voyahslam::tracking
