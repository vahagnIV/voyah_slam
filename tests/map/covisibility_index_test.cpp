#include <voyahslam/map/covisibility_index.h>

#include <gtest/gtest.h>

#include <utility>
#include <vector>

namespace voyahslam::map {
namespace {

using TCovisible = std::vector<std::pair<FrameId, uint32_t>>;

TEST(CovisibilityIndexTest, AddIgnoresDuplicates) {
    CovisibilityIndex index;
    index.Add(FrameId{1}, 4, MapPointId{7});
    index.Add(FrameId{1}, 5, MapPointId{7});  // same (frame, point): ignored
    index.Add(FrameId{2}, 3, MapPointId{7});

    ASSERT_EQ(index.NumObservations(MapPointId{7}), 2u);
    const auto& observations = index.Observations(MapPointId{7});
    EXPECT_EQ(observations[0].frame, FrameId{1});
    EXPECT_EQ(observations[0].feature, 4u);
    EXPECT_EQ(observations[1].frame, FrameId{2});
    EXPECT_EQ(observations[1].feature, 3u);
    EXPECT_EQ(index.Points(FrameId{1}), std::vector<MapPointId>{MapPointId{7}});
    EXPECT_TRUE(index.Contains(FrameId{1}, MapPointId{7}));
    EXPECT_TRUE(index.Contains(FrameId{2}, MapPointId{7}));
    EXPECT_FALSE(index.Contains(FrameId{3}, MapPointId{7}));

    index.Clear();
    EXPECT_EQ(index.NumObservations(MapPointId{7}), 0u);
    EXPECT_TRUE(index.Points(FrameId{1}).empty());
}

TEST(CovisibilityIndexTest, CovisibleOrdersBySharedCountThenId) {
    const FrameId a{5};
    const FrameId b{2};  // b < c: ties go to the smaller id
    const FrameId c{9};
    const FrameId d{1};
    CovisibilityIndex index;
    for (uint32_t p = 0; p < 7; ++p) index.Add(a, p, MapPointId{p});
    for (uint32_t p = 0; p < 3; ++p) index.Add(b, p, MapPointId{p});
    for (uint32_t p = 3; p < 6; ++p) index.Add(c, p, MapPointId{p});
    index.Add(d, 0, MapPointId{6});
    index.Add(d, 1, MapPointId{100});  // not shared with a

    EXPECT_EQ(index.Covisible(a, 10), (TCovisible{{b, 3}, {c, 3}, {d, 1}}));
    EXPECT_EQ(index.Covisible(a, 2), (TCovisible{{b, 3}, {c, 3}}));
    EXPECT_EQ(index.Covisible(d, 10), (TCovisible{{a, 1}}));
}

TEST(CovisibilityIndexTest, UnknownIdsAreEmpty) {
    CovisibilityIndex index;
    EXPECT_TRUE(index.Observations(MapPointId{3}).empty());
    EXPECT_EQ(index.NumObservations(MapPointId{3}), 0u);
    EXPECT_TRUE(index.Points(FrameId{3}).empty());
    EXPECT_FALSE(index.Contains(FrameId{3}, MapPointId{3}));
    EXPECT_TRUE(index.Covisible(FrameId{3}, 10).empty());

    index.Add(FrameId{1}, 0, MapPointId{0});
    EXPECT_TRUE(index.Observations(MapPointId{3}).empty());
    EXPECT_TRUE(index.Covisible(FrameId{1}, 10).empty());  // excludes the frame itself
}

} // namespace
} // namespace voyahslam::map
