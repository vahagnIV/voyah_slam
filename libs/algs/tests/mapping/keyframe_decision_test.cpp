#include <gtest/gtest.h>
#include <voyahslam/mapping/keyframe_decision.h>

namespace voyahslam::mapping {
namespace {

KeyFrameSignals Baseline() {
    KeyFrameSignals s;
    s.stereo = false;
    s.num_frames = 3;
    s.ref_points = 100;
    s.since_last = 0.1;
    s.backlog = 1;
    s.inliers = 50;
    return s;
}

TEST(ShouldCreateFrameTest, NoTriggerNoFrame) {
    EXPECT_FALSE(ShouldCreateFrame(Baseline(), {}));
}

TEST(ShouldCreateFrameTest, IdleAndWeakCreates) {
    auto s = Baseline();
    s.backlog = 0;
    s.inliers = 89;
    EXPECT_TRUE(ShouldCreateFrame(s, {}));
    s.inliers = 90;
    EXPECT_FALSE(ShouldCreateFrame(s, {}));
}

TEST(ShouldCreateFrameTest, MaxIntervalCreates) {
    auto s = Baseline();
    s.since_last = 1.0;
    s.inliers = 89;
    EXPECT_TRUE(ShouldCreateFrame(s, {}));
}

TEST(ShouldCreateFrameTest, MinInliersFloor) {
    auto s = Baseline();
    s.backlog = 0;
    s.inliers = 15;
    EXPECT_FALSE(ShouldCreateFrame(s, {}));
    s.inliers = 16;
    EXPECT_TRUE(ShouldCreateFrame(s, {}));
}

TEST(ShouldCreateFrameTest, StereoWeakTriggersWhileBusy) {
    auto s = Baseline();
    s.stereo = true;
    s.backlog = 1;
    s.inliers = 24;
    EXPECT_TRUE(ShouldCreateFrame(s, {}));
}

TEST(ShouldCreateFrameTest, StereoNeedClose) {
    auto s = Baseline();
    s.stereo = true;
    s.inliers = 90;
    s.close_tracked = 99;
    s.close_untracked = 71;
    s.backlog = 1;
    EXPECT_TRUE(ShouldCreateFrame(s, {}));
    s.close_untracked = 70;
    EXPECT_FALSE(ShouldCreateFrame(s, {}));
}

TEST(ShouldCreateFrameTest, StereoEarlyRatio) {
    auto s = Baseline();
    s.stereo = true;
    s.num_frames = 1;
    s.backlog = 0;
    s.inliers = 39;
    EXPECT_TRUE(ShouldCreateFrame(s, {}));
    s.inliers = 40;
    EXPECT_FALSE(ShouldCreateFrame(s, {}));
}

}  // namespace
}  // namespace voyahslam::mapping
