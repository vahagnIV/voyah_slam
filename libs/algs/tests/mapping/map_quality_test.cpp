#include <gtest/gtest.h>
#include <voyahslam/mapping/map_quality.h>

namespace voyahslam::mapping {
namespace {

RecentPointStats Healthy() {
    RecentPointStats s;
    s.frames_since_creation = 1;
    s.visible = 10;
    s.found = 10;
    s.weighted_obs = 5;
    s.stereo = false;
    return s;
}

TEST(JudgeRecentPointTest, Boundaries) {
    const MapQualityOptions o;
    auto s = Healthy();
    EXPECT_EQ(JudgeRecentPoint(s, o), RecentPointVerdict::Keep);

    // found / visible: 1/4 keeps, 1/5 removes.
    s.visible = 4;
    s.found = 1;
    EXPECT_EQ(JudgeRecentPoint(s, o), RecentPointVerdict::Keep);
    s.visible = 5;
    EXPECT_EQ(JudgeRecentPoint(s, o), RecentPointVerdict::Remove);
    s.visible = 0;  // never seen: ratio undefined, not removed.
    s.found = 0;
    EXPECT_EQ(JudgeRecentPoint(s, o), RecentPointVerdict::Keep);

    // Observation floor applies from 2 Frames on: mono <= 2, stereo <= 3.
    s = Healthy();
    s.frames_since_creation = 1;
    s.weighted_obs = 0;
    EXPECT_EQ(JudgeRecentPoint(s, o), RecentPointVerdict::Keep);
    s.frames_since_creation = 2;
    s.weighted_obs = 2;
    EXPECT_EQ(JudgeRecentPoint(s, o), RecentPointVerdict::Remove);
    s.weighted_obs = 3;
    EXPECT_EQ(JudgeRecentPoint(s, o), RecentPointVerdict::Keep);
    s.stereo = true;
    s.weighted_obs = 3;
    EXPECT_EQ(JudgeRecentPoint(s, o), RecentPointVerdict::Remove);
    s.weighted_obs = 4;
    EXPECT_EQ(JudgeRecentPoint(s, o), RecentPointVerdict::Keep);

    // Graduation at recent_window Frames.
    s = Healthy();
    s.frames_since_creation = 3;
    EXPECT_EQ(JudgeRecentPoint(s, o), RecentPointVerdict::Graduate);
    s.found = 1;  // bad ratio wins over graduation.
    EXPECT_EQ(JudgeRecentPoint(s, o), RecentPointVerdict::Remove);
}

std::vector<FramePointStats> Points(size_t redundant, size_t total) {
    std::vector<FramePointStats> v;
    for (size_t i = 0; i < total; ++i) v.push_back({0, true, i < redundant ? size_t{3} : size_t{2}});
    return v;
}

TEST(IsRedundantFrameTest, Boundaries) {
    const MapQualityOptions o;
    EXPECT_TRUE(IsRedundantFrame(Points(9, 10), o));
    EXPECT_FALSE(IsRedundantFrame(Points(89, 100), o));
    EXPECT_FALSE(IsRedundantFrame({}, o));

    // Uncounted points (stereo far) are ignored.
    auto v = Points(9, 10);
    v.push_back({0, false, 0});
    EXPECT_TRUE(IsRedundantFrame(v, o));
    EXPECT_FALSE(IsRedundantFrame({{0, false, 5}}, o));
}

}  // namespace
}  // namespace voyahslam::mapping
