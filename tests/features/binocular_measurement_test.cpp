#include <voyahslam/features/binocular_measurement.h>

#include <gtest/gtest.h>

namespace voyahslam::features {
namespace {

using TestFeatures = ImageFeatures<uint8_t, 32>;
using TestBinocular = BinocularMeasurement<uint8_t, 32>;

TestFeatures MakeFeatures(int n) {
    // K = I, so these are normalized coordinates: rays <= ~35 degrees off-axis.
    geometry::TPoints2DF points = geometry::TPoints2DF::Random(2, n) * 0.5;
    geometry::TRowVectorF strengths = geometry::TRowVectorF::Random(n);
    geometry::TRowVectorF angles = geometry::TRowVectorF::Random(n);
    TestFeatures::TDescriptors descs = TestFeatures::TDescriptors::Random(32, n);
    return TestFeatures(std::move(points), std::move(strengths),
                        std::move(angles), std::move(descs),
                        TLevels::Zero(n));
}

StereoRig MakeRig() {
    return StereoRig{
        .K_left = geometry::TMatrix33F::Identity(),
        .K_right = geometry::TMatrix33F::Identity(),
        .T_rl = geometry::RigidTransformation(geometry::TMatrix33F::Identity(),
                                                      geometry::TVector3DF(-0.12, 0.0, 0.0)),
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

TEST(BinocularMeasurementTest, StoresRig) {
    auto rig = MakeRig();
    auto bcf = TestBinocular(MakeFeatures(3), MakeFeatures(3), rig, 1.0);
    EXPECT_TRUE(bcf.rig().T_rl.ToMatrix34().isApprox(rig.T_rl.ToMatrix34()));
    EXPECT_DOUBLE_EQ(bcf.rig().baseline, 0.12);
}

TEST(BinocularMeasurementTest, SetStereoMatches) {
    auto rig = MakeRig();
    auto bcf = TestBinocular(MakeFeatures(4), MakeFeatures(4), rig, 1.0);
    EXPECT_FALSE(bcf.stereo_matched());
    EXPECT_TRUE((bcf.depths().array() == -1.0).all());

    StereoMatches matches;
    matches.right_indices = Eigen::VectorXi::Constant(4, -1);
    matches.right_indices(1) = 2;
    matches.right_indices(3) = 0;
    matches.depths = geometry::TRowVectorF::Constant(4, -1.0);
    matches.depths(1) = 4.0;
    matches.depths(3) = 7.5;
    matches.num_matches = 2;
    geometry::TRowVectorF expected_depths = matches.depths;
    Eigen::VectorXi expected_indices = matches.right_indices;

    bcf.SetStereoMatches(std::move(matches));
    EXPECT_TRUE(bcf.stereo_matched());
    EXPECT_TRUE(bcf.depths().isApprox(expected_depths));
    EXPECT_EQ(bcf.right_indices(), expected_indices);
}

} // namespace
} // namespace voyahslam::features
