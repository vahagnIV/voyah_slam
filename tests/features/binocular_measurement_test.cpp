#include <voyahslam/features/binocular_measurement.h>

#include <gtest/gtest.h>

namespace voyahslam::features {
namespace {

using TestFeatures = ImageFeatures<uint8_t, 32>;
using TestBinocular = BinocularMeasurement<uint8_t, 32>;

TestFeatures MakeFeatures(int n) {
    geometry::TPoints2DF points = geometry::TPoints2DF::Random(2, n) * 100.0 +
                                  geometry::TPoints2DF::Constant(2, n, 320.0);
    geometry::TRowVectorF strengths = geometry::TRowVectorF::Random(n);
    geometry::TRowVectorF angles = geometry::TRowVectorF::Random(n);
    TestFeatures::TDescriptors descs = TestFeatures::TDescriptors::Random(32, n);
    return TestFeatures(std::move(points), std::move(strengths),
                        std::move(angles), std::move(descs));
}

StereoRig MakeRig() {
    return StereoRig{
        .K_left = geometry::TMatrix33F::Identity(),
        .K_right = geometry::TMatrix33F::Identity(),
        .R_rl = geometry::TMatrix33F::Identity(),
        .t_rl = geometry::TVector3DF(0.12, 0.0, 0.0),
        .baseline = 0.12,
    };
}

TEST(BinocularMeasurementTest, Construction) {
    auto rig = MakeRig();
    auto bcf = TestBinocular(MakeFeatures(5), MakeFeatures(5), rig, 3.0);

    EXPECT_TRUE(bcf.left().K().isIdentity());
    EXPECT_TRUE(bcf.right().K().isIdentity());
    EXPECT_EQ(bcf.depths().size(), 5);
    EXPECT_EQ(bcf.right_indices().size(), 5);
}

TEST(BinocularMeasurementTest, TimestampIsSharedByBothCameras) {
    auto rig = MakeRig();
    auto measurement = TestBinocular(MakeFeatures(2), MakeFeatures(2), rig, 7.25);
    EXPECT_DOUBLE_EQ(measurement.timestamp(), 7.25);
    EXPECT_DOUBLE_EQ(measurement.left().timestamp(), 7.25);
    EXPECT_DOUBLE_EQ(measurement.right().timestamp(), 7.25);
}

TEST(BinocularMeasurementTest, Accessors) {
    auto rig = MakeRig();
    auto bcf = TestBinocular(MakeFeatures(3), MakeFeatures(4), rig, 3.0);

    EXPECT_DOUBLE_EQ(bcf.depth(0), -1.0);
    EXPECT_EQ(bcf.right_indices()(0), -1);
    EXPECT_EQ(bcf.left().points().cols(), 3);
    EXPECT_EQ(bcf.right().points().cols(), 4);
}

} // namespace
} // namespace voyahslam::features
