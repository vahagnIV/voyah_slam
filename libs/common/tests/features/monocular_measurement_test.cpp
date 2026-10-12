#include <gtest/gtest.h>
#include "voyahslam/features/monocular_measurement.h"
#include "voyahslam/geometry/distortion_models/dummy_distortion_model.h"
#include "voyahslam/geometry/distortion_models/kannala_brandt_distortion_model.h"
#include <cmath>
#include "voyahslam/defs.h"

using voyahslam::features::ImageFeatures;
using namespace voyahslam::geometry;
using namespace voyahslam::features;
using voyahslam::features::MonocularMeasurement;
using voyahslam::geometry::distortion_models::DummyDistortionModel;
using voyahslam::precision_t;

class MonocularMeasurementTest : public ::testing::Test {
protected:
    static constexpr int DescriptorSize = 32;
    typedef uint8_t TDescriptorScalarType;
    typedef ImageFeatures<TDescriptorScalarType, DescriptorSize> TFeatures;
    typedef TFeatures::TDescriptor TDescriptor;
    typedef TFeatures::TDescriptors TDescriptors;
    void SetUp() override {
        // Create test data
        points_ = TPoints2DF::Random(TPoints2DF::RowsAtCompileTime, 3); // 3 points
        strengths_ = TRowVectorF::Random(3);
        angles_ = TRowVectorF::Random( 3);
        descriptors_ = TDescriptors::Random(TDescriptors::RowsAtCompileTime, 3);

        // Create a simple camera matrix
        K_ = Eigen::Matrix<precision_t, 3, 3>::Identity();
        K_(0, 0) = 500.0;  // fx
        K_(1, 1) = 500.0;  // fy
        K_(0, 2) = 320.0;  // cx
        K_(1, 2) = 240.0;  // cy
    }

    TPoints2DF points_;
    TRowVectorF strengths_;
    TRowVectorF angles_;
    TDescriptors descriptors_;
    Eigen::Matrix<precision_t, 3, 3> K_;
};

TEST_F(MonocularMeasurementTest, ConstructorAndGetters) {
    auto points = points_;
    // Create original features
    ImageFeatures<TDescriptorScalarType, DescriptorSize> original_features(
        std::move(points_),
        std::move(strengths_),
        std::move(angles_),
        std::move(descriptors_),
        TLevels::Zero(3)
    );

    // Create undistorted features
    const auto dummy_model = std::make_shared<DummyDistortionModel>();

    // Create frame features
    MonocularMeasurement measurement(
        std::move(original_features),
        K_,
        dummy_model,
        12.5
    );

    // Verify camera matrix
    EXPECT_DOUBLE_EQ(measurement.timestamp(), 12.5);
    EXPECT_TRUE(measurement.K().isApprox(K_));

    // Verify normalized points
    // For our test case with identity-like K, normalized points should be similar to original points
    // but with z=1 (homogeneous coordinates)
    THomogenousPointsF expected_normalized(3, 3);
    expected_normalized.topRows(2) = points;
    expected_normalized.row(2).setOnes();
    expected_normalized = K_.inverse() * expected_normalized;
    EXPECT_TRUE(measurement.normalized_points().isApprox(expected_normalized));

    // Verify features getter
    EXPECT_TRUE(measurement.points().isApprox(points));
}

TEST_F(MonocularMeasurementTest, MoveSemantics) {
    auto points = points_;
    // Create original features
    ImageFeatures<TDescriptorScalarType, DescriptorSize> original_features(
        std::move(points_),
        std::move(strengths_),
        std::move(angles_),
        std::move(descriptors_),
        TLevels::Zero(3)
    );

    // Create undistorted features
    const auto dummy_model = std::make_shared<DummyDistortionModel>();

    // Create frame features
    MonocularMeasurement measurement1(
        std::move(original_features),
        K_,
        dummy_model,
        0.0
    );

    // Move construct
    MonocularMeasurement measurement2(std::move(measurement1));

    // Verify that the data was moved correctly
    EXPECT_TRUE(measurement2.K().isApprox(K_));
    EXPECT_EQ(measurement2.normalized_points().cols(), 3);
    EXPECT_EQ(measurement2.normalized_points().rows(), 3);
    EXPECT_TRUE(measurement2.points().isApprox(points));
}

namespace {

using voyahslam::geometry::distortion_models::KannalaBrandtDistortionModel;

constexpr precision_t kDeg = 3.14159265358979323846 / 180.0;

// TUM-VI cam0 (equidistant fisheye, 1024x1024).
const precision_t kKB[4] = {0.010171079892421483, -0.010816440029919381,
                            0.005942781769412756, -0.001662284667857643};

TMatrix33F TumViK() {
    TMatrix33F K;
    K << 380.81, 0, 510.29, 0, 380.81, 514.33, 0, 0, 1;
    return K;
}

// Pixel of a ray `theta` off the optical axis at azimuth `phi` (KB projection).
TPoint2DF ProjectKB(const TMatrix33F& K, precision_t theta, precision_t phi) {
    precision_t t2 = theta * theta;
    precision_t theta_d =
        theta * (1 + kKB[0] * t2 + kKB[1] * t2 * t2 + kKB[2] * t2 * t2 * t2 + kKB[3] * t2 * t2 * t2 * t2);
    return {K(0, 0) * theta_d * std::cos(phi) + K(0, 2), K(1, 1) * theta_d * std::sin(phi) + K(1, 2)};
}

using TTestFeatures = ImageFeatures<uint8_t, 32>;

// Feature i has strength i, angle 10 * i and a descriptor filled with i.
TTestFeatures MakeFeaturesAt(const TPoints2DF& points) {
    const auto n = points.cols();
    TTestFeatures::TDescriptors descriptors(32, n);
    TRowVectorF strengths(n), angles(n);
    for (Eigen::Index i = 0; i < n; ++i) {
        descriptors.col(i).setConstant(static_cast<uint8_t>(i));
        strengths(i) = static_cast<precision_t>(i);
        angles(i) = 10.0 * static_cast<precision_t>(i);
    }
    TPoints2DF copy = points;
    return TTestFeatures(std::move(copy), std::move(strengths), std::move(angles), std::move(descriptors),
                         TLevels::Zero(n));
}

} // namespace

TEST(MonocularMeasurementRayAngleTest, DropsFisheyeFeaturesBeyondMaxRayAngle) {
    const auto model = std::make_shared<KannalaBrandtDistortionModel>(kKB[0], kKB[1], kKB[2], kKB[3]);
    TMatrix33F K = TumViK();
    model->PrecomputeInitialGuessLookup(K, 1024, 1024);

    // Rays at 30, 100, 80, 86 and 89 degrees; only 30 and 80 survive.
    const precision_t thetas[] = {30 * kDeg, 100 * kDeg, 80 * kDeg, 86 * kDeg, 89 * kDeg};
    TPoints2DF pixels(2, 5);
    for (int i = 0; i < 5; ++i) pixels.col(i) = ProjectKB(K, thetas[i], 0.7 * i);

    MonocularMeasurement<uint8_t, 32> measurement(MakeFeaturesAt(pixels), K, model, 0.0);

    ASSERT_EQ(measurement.points().cols(), 2);
    ASSERT_EQ(measurement.normalized_points().cols(), 2);
    ASSERT_EQ(measurement.descriptors().cols(), 2);
    // Surviving features keep their own data, in order: original 0 and 2.
    EXPECT_EQ(measurement.descriptors()(0, 0), 0);
    EXPECT_EQ(measurement.descriptors()(31, 1), 2);
    EXPECT_DOUBLE_EQ(measurement.strengths()(1), 2.0);
    EXPECT_DOUBLE_EQ(measurement.angles()(1), 20.0);
    // ... and are undistorted correctly: |(x, y)| = tan(theta) on z = 1.
    EXPECT_NEAR(measurement.normalized_point(0).hnormalized().norm(), std::tan(30 * kDeg), 1e-6);
    EXPECT_NEAR(measurement.normalized_point(1).hnormalized().norm(), std::tan(80 * kDeg), 1e-5);
}

TEST(MonocularMeasurementRayAngleTest, DropsPinholeFeaturesBeyondMaxRayAngle) {
    TMatrix33F K;
    K << 100, 0, 0, 0, 100, 0, 0, 0, 1;
    TPoints2DF pixels(2, 2);
    pixels.col(0) << 100 * std::tan(60 * kDeg), 0;  // 60 degrees: kept
    pixels.col(1) << 0, 100 * std::tan(87 * kDeg);  // 87 degrees: dropped

    MonocularMeasurement<uint8_t, 32> measurement(MakeFeaturesAt(pixels), K, nullptr, 0.0);

    ASSERT_EQ(measurement.points().cols(), 1);
    EXPECT_NEAR(measurement.points()(0, 0), 100 * std::tan(60 * kDeg), 1e-9);
}

TEST_F(MonocularMeasurementTest, DropKeepsLevelsAligned) {
    constexpr int n = 4;
    TPoints2DF points(2, n);
    points.col(0) << 320, 240;
    points.col(1) << 320 + 500 * std::tan(87 * kDeg), 240;  // beyond 85 degrees: dropped
    points.col(2) << 400, 260;
    points.col(3) << 250, 200;
    TLevels levels(n);
    levels << 0, 1, 2, 3;
    TFeatures features(std::move(points), TRowVectorF::Ones(n), TRowVectorF::Zero(n),
                       TDescriptors::Zero(DescriptorSize, n), std::move(levels));

    MonocularMeasurement<TDescriptorScalarType, DescriptorSize> measurement(
        std::move(features), K_, nullptr, 0.0);

    ASSERT_EQ(measurement.points().cols(), 3);
    ASSERT_EQ(measurement.levels().cols(), 3);
    EXPECT_EQ(measurement.levels()(0), 0);
    EXPECT_EQ(measurement.levels()(1), 2);
    EXPECT_EQ(measurement.levels()(2), 3);
    EXPECT_FLOAT_EQ(measurement.points()(0, 1), 400.0F);
    EXPECT_FLOAT_EQ(measurement.points()(0, 2), 250.0F);
}

TEST_F(MonocularMeasurementTest, DistortedPointsAndCamera) {
    const auto model = std::make_shared<KannalaBrandtDistortionModel>(kKB[0], kKB[1], kKB[2], kKB[3]);
    TMatrix33F K = TumViK();
    model->PrecomputeInitialGuessLookup(K, 1024, 1024);
    TPoints2DF pixels(2, 3);
    for (int i = 0; i < 3; ++i) pixels.col(i) = ProjectKB(K, (20 + 15 * i) * kDeg, 0.9 * i);

    MonocularMeasurement<uint8_t, 32> measurement(MakeFeaturesAt(pixels), K, model, 0.0);

    ASSERT_EQ(measurement.distorted_points().cols(), 3);
    EXPECT_TRUE(measurement.distorted_points().isApprox(pixels, 1e-12));
    EXPECT_TRUE(measurement.camera().K.isApprox(K));
    EXPECT_EQ(measurement.camera().distortion, model);
}

TEST_F(MonocularMeasurementTest, DistortedPointsCompactedWithDrop) {
    const auto model = std::make_shared<KannalaBrandtDistortionModel>(kKB[0], kKB[1], kKB[2], kKB[3]);
    TMatrix33F K = TumViK();
    model->PrecomputeInitialGuessLookup(K, 1024, 1024);
    const precision_t thetas[] = {30 * kDeg, 100 * kDeg, 80 * kDeg, 86 * kDeg, 50 * kDeg};
    TPoints2DF pixels(2, 5);
    for (int i = 0; i < 5; ++i) pixels.col(i) = ProjectKB(K, thetas[i], 0.7 * i);

    MonocularMeasurement<uint8_t, 32> measurement(MakeFeaturesAt(pixels), K, model, 0.0);

    ASSERT_EQ(measurement.points().cols(), 3);
    ASSERT_EQ(measurement.distorted_points().cols(), 3);
    ASSERT_EQ(measurement.levels().cols(), 3);
    EXPECT_EQ(measurement.descriptors()(0, 2), 4);  // original feature 4 survived last
    for (int i = 0; i < 3; ++i) {
        EXPECT_LT((measurement.camera().Project(measurement.normalized_point(i)) -
                   measurement.distorted_points().col(i)).norm(), 1e-6);
    }
}
