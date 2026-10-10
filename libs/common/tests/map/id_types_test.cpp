#include <voyahslam/map/absorbed_point_ids.h>
#include <voyahslam/map/frame_id.h>
#include <voyahslam/map/map_id.h>
#include <voyahslam/map/map_point_id.h>

#include <gtest/gtest.h>

#include <optional>
#include <type_traits>
#include <vector>

namespace voyahslam::map {
namespace {

TEST(MapPointIdTest, Equality) {
    EXPECT_EQ(MapPointId{5}, MapPointId{5});
    EXPECT_NE(MapPointId{5}, MapPointId{6});
}

TEST(MapPointIdTest, Ordering) {
    EXPECT_LT(MapPointId{3}, MapPointId{7});
    EXPECT_GT(MapPointId{10}, MapPointId{2});
}

TEST(FrameIdTest, Equality) {
    EXPECT_EQ(FrameId{1}, FrameId{1});
    EXPECT_NE(FrameId{1}, FrameId{2});
}

TEST(FrameIdTest, Ordering) {
    EXPECT_LT(FrameId{0}, FrameId{100});
}

TEST(IdTypesTest, MapIdOrdering) {
    EXPECT_LT(MapId{1}, MapId{2});
    EXPECT_EQ(MapId{}, MapId{0});
}

TEST(IdTypeSafetyTest, NotConvertible) {
    static_assert(!std::is_convertible_v<MapPointId, FrameId>);
    static_assert(!std::is_convertible_v<FrameId, MapPointId>);
}

// Map merging's id rule: the alive old ids, ascending, take first_new, +1, ...
TEST(AbsorbedPointIdsTest, AliveIdsTakeNextIdsInOrder) {
    const std::vector<bool> alive = {true, false, true, true, false};
    const std::vector<std::optional<MapPointId>> new_ids =
        AbsorbedPointIds(5, MapPointId{40}, [&](MapPointId id) { return static_cast<bool>(alive[id.value]); });
    const std::vector<std::optional<MapPointId>> expected = {MapPointId{40}, std::nullopt, MapPointId{41},
                                                             MapPointId{42}, std::nullopt};
    EXPECT_EQ(new_ids, expected);
    EXPECT_TRUE(AbsorbedPointIds(0, MapPointId{7}, [](MapPointId) { return true; }).empty());
}

} // namespace
} // namespace voyahslam::map
