#include <voyahslam/features/stereo_rig.h>

#include <gtest/gtest.h>

namespace voyahslam::features {
namespace {

TEST(StereoRigTest, Construction) {
    StereoRig rig{
        .K_left = geometry::TMatrix33F::Identity(),
        .K_right = geometry::TMatrix33F::Identity(),
        .T_rl = geometry::RigidTransformation(geometry::TMatrix33F::Identity(),
                                              geometry::TVector3DF(-0.12, 0.0, 0.0)),
        .baseline = 0.12,
    };

    EXPECT_TRUE(rig.K_left.isIdentity());
    EXPECT_TRUE(rig.K_right.isIdentity());
    EXPECT_TRUE(rig.T_rl.rotation().isIdentity());
    EXPECT_DOUBLE_EQ(rig.T_rl.translation().x(), -0.12);
    EXPECT_DOUBLE_EQ(rig.T_rl.translation().y(), 0.0);
    EXPECT_DOUBLE_EQ(rig.T_rl.translation().z(), 0.0);
    EXPECT_DOUBLE_EQ(rig.baseline, 0.12);
    EXPECT_EQ(rig.distortion_left, nullptr);
    EXPECT_EQ(rig.distortion_right, nullptr);
}

TEST(StereoRigTest, TrlMapsLeftToRight) {
    StereoRig rig{
        .K_left = geometry::TMatrix33F::Identity(),
        .K_right = geometry::TMatrix33F::Identity(),
        .T_rl = geometry::RigidTransformation(geometry::TMatrix33F::Identity(),
                                              geometry::TVector3DF(-0.12, 0.0, 0.0)),
        .baseline = 0.12,
    };
    geometry::TPoint3DF p_right = rig.T_rl.Transform(geometry::TPoint3DF(1.0, 0.0, 5.0));
    EXPECT_TRUE(p_right.isApprox(geometry::TPoint3DF(0.88, 0.0, 5.0)));
}

} // namespace
} // namespace voyahslam::features
