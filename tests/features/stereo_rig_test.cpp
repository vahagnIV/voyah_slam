#include <voyahslam/features/stereo_rig.h>

#include <gtest/gtest.h>

namespace voyahslam::features {
namespace {

TEST(StereoRigTest, Construction) {
    StereoRig rig{
        .K_left = geometry::TMatrix33F::Identity(),
        .K_right = geometry::TMatrix33F::Identity(),
        .R_rl = geometry::TMatrix33F::Identity(),
        .t_rl = geometry::TVector3DF(0.12, 0.0, 0.0),
        .baseline = 0.12,
    };

    EXPECT_TRUE(rig.K_left.isIdentity());
    EXPECT_TRUE(rig.K_right.isIdentity());
    EXPECT_TRUE(rig.R_rl.isIdentity());
    EXPECT_DOUBLE_EQ(rig.t_rl.x(), 0.12);
    EXPECT_DOUBLE_EQ(rig.t_rl.y(), 0.0);
    EXPECT_DOUBLE_EQ(rig.t_rl.z(), 0.0);
    EXPECT_DOUBLE_EQ(rig.baseline, 0.12);
    EXPECT_EQ(rig.distortion_left, nullptr);
    EXPECT_EQ(rig.distortion_right, nullptr);
}

} // namespace
} // namespace voyahslam::features
