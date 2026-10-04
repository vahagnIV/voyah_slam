#include <voyahslam/features/orb_feature_extractor.h>
#include <voyahslam/map/frame.h>

#include <gtest/gtest.h>

namespace voyahslam::map {
namespace {

using OrbFeatures = features::ImageFeatures<uint8_t, 32>;
using OrbMonocularMeasurement = features::MonocularMeasurement<uint8_t, 32>;

OrbFeatures MakeFeatures(int n) {
    // K = I, so these are normalized coordinates: rays <= ~35 degrees off-axis.
    geometry::TPoints2DF points = geometry::TPoints2DF::Random(2, n) * 0.5;
    geometry::TRowVectorF strengths = geometry::TRowVectorF::Random(n);
    geometry::TRowVectorF angles = geometry::TRowVectorF::Random(n);
    OrbFeatures::TDescriptors descs = OrbFeatures::TDescriptors::Random(32, n);
    return OrbFeatures(std::move(points), std::move(strengths),
                       std::move(angles), std::move(descs),
                       features::TLevels::Zero(n));
}

TEST(FrameTest, MonoFrameConstruction) {
    geometry::TPose pose = geometry::TPose::Identity();
    geometry::TMatrix33F K = geometry::TMatrix33F::Identity();
    auto measurement = OrbMonocularMeasurement(MakeFeatures(10), K, nullptr, 1.0);

    MonoFrame<features::OrbFeatureExtractor> frame(
        FrameId{0}, pose, std::move(measurement));

    EXPECT_EQ(frame.id(), FrameId{0});
    EXPECT_TRUE(frame.pose().isApprox(geometry::TPose::Identity()));
    EXPECT_EQ(frame.measurement().points().cols(), 10);
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

    features::BinocularMeasurement<uint8_t, 32> bcf(
        MakeFeatures(5), MakeFeatures(5), rig, 2.5);

    StereoFrame<features::OrbFeatureExtractor> frame(
        FrameId{1}, pose, std::move(bcf));

    EXPECT_EQ(frame.id(), FrameId{1});
    EXPECT_EQ(frame.measurement().left().points().cols(), 5);
    EXPECT_DOUBLE_EQ(frame.timestamp(), 2.5);
}

TEST(FrameTest, MutablePose) {
    geometry::TPose pose = geometry::TPose::Identity();
    geometry::TMatrix33F K = geometry::TMatrix33F::Identity();
    auto measurement = OrbMonocularMeasurement(MakeFeatures(3), K, nullptr, 0.0);

    MonoFrame<features::OrbFeatureExtractor> frame(
        FrameId{0}, pose, std::move(measurement));

    frame.mutable_pose()(0, 3) = 5.0;
    EXPECT_DOUBLE_EQ(frame.pose()(0, 3), 5.0);
}

} // namespace
} // namespace voyahslam::map
