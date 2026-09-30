#include <voyahslam/map/frame_id.h>
#include <voyahslam/map/map_point_id.h>

#include <gtest/gtest.h>

#include <type_traits>

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

TEST(IdTypeSafetyTest, NotConvertible) {
    static_assert(!std::is_convertible_v<MapPointId, FrameId>);
    static_assert(!std::is_convertible_v<FrameId, MapPointId>);
}

} // namespace
} // namespace voyahslam::map
