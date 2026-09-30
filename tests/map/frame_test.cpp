#include <voyahslam/features/orb_feature_extractor.h>
#include <voyahslam/map/frame.h>

#include <gtest/gtest.h>

namespace voyahslam::map {
namespace {

using OrbFeatures = features::ImageFeatures<uint8_t, 32>;
using OrbCameraFrame = features::CameraFrame<uint8_t, 32>;

OrbFeatures MakeFeatures(int n) {
    geometry::TPoints2DF points = geometry::TPoints2DF::Random(2, n) * 100.0 +
                                  geometry::TPoints2DF::Constant(2, n, 320.0);
    geometry::TRowVectorF strengths = geometry::TRowVectorF::Random(n);
    geometry::TRowVectorF angles = geometry::TRowVectorF::Random(n);
    OrbFeatures::TDescriptors descs = OrbFeatures::TDescriptors::Random(32, n);
    return OrbFeatures(std::move(points), std::move(strengths),
                       std::move(angles), std::move(descs));
}

TEST(FrameTest, MonoFrameConstruction) {
    geometry::TPose pose = geometry::TPose::Identity();
    geometry::TMatrix33F K = geometry::TMatrix33F::Identity();
    auto camera_frame = OrbCameraFrame(MakeFeatures(10), K, nullptr);

    MonoFrame<features::OrbFeatureExtractor> frame(
        FrameId{0}, pose, std::move(camera_frame), 1.0);

    EXPECT_EQ(frame.id(), FrameId{0});
    EXPECT_TRUE(frame.pose().isApprox(geometry::TPose::Identity()));
    EXPECT_EQ(frame.camera_frame().points().cols(), 10);
    EXPECT_DOUBLE_EQ(frame.timestamp(), 1.0);
}

TEST(FrameTest, StereoFrameConstruction) {
    geometry::TPose pose = geometry::TPose::Identity();
    features::StereoRig rig{
        .K_left = geometry::TMatrix33F::Identity(),
        .K_right = geometry::TMatrix33F::Identity(),
        .R_rl = geometry::TMatrix33F::Identity(),
        .t_rl = geometry::TVector3DF(0.12, 0, 0),
        .baseline = 0.12,
    };

    features::BinocularCameraFrame<uint8_t, 32> bcf(
        MakeFeatures(5), MakeFeatures(5), rig);

    StereoFrame<features::OrbFeatureExtractor> frame(
        FrameId{1}, pose, std::move(bcf), 2.5);

    EXPECT_EQ(frame.id(), FrameId{1});
    EXPECT_EQ(frame.camera_frame().left().points().cols(), 5);
    EXPECT_DOUBLE_EQ(frame.timestamp(), 2.5);
}

TEST(FrameTest, MutablePose) {
    geometry::TPose pose = geometry::TPose::Identity();
    geometry::TMatrix33F K = geometry::TMatrix33F::Identity();
    auto camera_frame = OrbCameraFrame(MakeFeatures(3), K, nullptr);

    MonoFrame<features::OrbFeatureExtractor> frame(
        FrameId{0}, pose, std::move(camera_frame), 0.0);

    frame.mutable_pose()(0, 3) = 5.0;
    EXPECT_DOUBLE_EQ(frame.pose()(0, 3), 5.0);
}

} // namespace
} // namespace voyahslam::map
