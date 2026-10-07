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

TEST(CovisibilityIndexTest, RemoveVariants) {
    const FrameId a{1};
    const FrameId b{2};
    const FrameId c{3};
    CovisibilityIndex index;
    for (uint32_t p = 0; p < 4; ++p) index.Add(a, p, MapPointId{p});
    for (uint32_t p = 0; p < 3; ++p) index.Add(b, 10 + p, MapPointId{p});
    for (uint32_t p = 1; p < 4; ++p) index.Add(c, 20 + p, MapPointId{p});
    ASSERT_EQ(index.Covisible(a, 10), (TCovisible{{b, 3}, {c, 3}}));

    // One observation: both directions, and the shared counts.
    index.Remove(b, MapPointId{0});
    EXPECT_FALSE(index.Contains(b, MapPointId{0}));
    EXPECT_EQ(index.NumObservations(MapPointId{0}), 1u);
    EXPECT_EQ(index.Points(b), (std::vector<MapPointId>{MapPointId{1}, MapPointId{2}}));
    EXPECT_EQ(index.Covisible(a, 10), (TCovisible{{c, 3}, {b, 2}}));
    index.Remove(b, MapPointId{0});  // absent: no-op
    index.Remove(FrameId{9}, MapPointId{1});
    index.Remove(a, MapPointId{99});
    EXPECT_EQ(index.NumObservations(MapPointId{1}), 3u);

    // A point: every Frame's side.
    index.RemovePoint(MapPointId{1});
    EXPECT_TRUE(index.Observations(MapPointId{1}).empty());
    EXPECT_FALSE(index.Contains(a, MapPointId{1}));
    EXPECT_EQ(index.Points(a),
              (std::vector<MapPointId>{MapPointId{0}, MapPointId{2}, MapPointId{3}}));
    EXPECT_EQ(index.Points(b), std::vector<MapPointId>{MapPointId{2}});
    EXPECT_EQ(index.Covisible(a, 10), (TCovisible{{c, 2}, {b, 1}}));
    index.RemovePoint(MapPointId{1});  // already gone
    index.RemovePoint(MapPointId{99});

    // A Frame: every point's side.
    index.RemoveFrame(c);
    EXPECT_TRUE(index.Points(c).empty());
    EXPECT_EQ(index.NumObservations(MapPointId{2}), 2u);
    EXPECT_EQ(index.NumObservations(MapPointId{3}), 1u);
    EXPECT_FALSE(index.Contains(c, MapPointId{3}));
    EXPECT_EQ(index.Covisible(a, 10), (TCovisible{{b, 1}}));
    EXPECT_EQ(index.Covisible(b, 10), (TCovisible{{a, 1}}));
    index.RemoveFrame(c);  // already gone
    index.RemoveFrame(FrameId{9});

    // Re-adding after removal works.
    index.Add(c, 5, MapPointId{3});
    EXPECT_EQ(index.Covisible(a, 10), (TCovisible{{b, 1}, {c, 1}}));
}

} // namespace
} // namespace voyahslam::map
