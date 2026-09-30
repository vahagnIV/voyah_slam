#include <gtest/gtest.h>
#include "voyahslam/features/camera_frame.h"
#include "voyahslam/features/distortion_models/dummy_distortion_model.h"
#include "voyahslam/defs.h"

using voyahslam::features::ImageFeatures;
using namespace voyahslam::geometry;
using namespace voyahslam::features;
using voyahslam::features::CameraFrame;
using voyahslam::features::distortion_models::DummyDistortionModel;
using voyahslam::precision_t;

class CameraFrameTest : public ::testing::Test {
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

TEST_F(CameraFrameTest, ConstructorAndGetters) {
    auto points = points_;
    // Create original features
    ImageFeatures<TDescriptorScalarType, DescriptorSize> original_features(
        std::move(points_),
        std::move(strengths_),
        std::move(angles_),
        std::move(descriptors_)
    );

    // Create undistorted features
    DummyDistortionModel dummy_model;

    // Create frame features
    CameraFrame camera_frame(
        std::move(original_features),
        K_,
        &dummy_model
    );

    // Verify camera matrix
    EXPECT_TRUE(camera_frame.K().isApprox(K_));

    // Verify normalized points
    // For our test case with identity-like K, normalized points should be similar to original points
    // but with z=1 (homogeneous coordinates)
    THomogenousPointsF expected_normalized(3, 3);
    expected_normalized.topRows(2) = points;
    expected_normalized.row(2).setOnes();
    expected_normalized = K_.inverse() * expected_normalized;
    EXPECT_TRUE(camera_frame.normalized_points().isApprox(expected_normalized));

    // Verify features getter
    EXPECT_TRUE(camera_frame.points().isApprox(points));
}

TEST_F(CameraFrameTest, MoveSemantics) {
    auto points = points_;
    // Create original features
    ImageFeatures<TDescriptorScalarType, DescriptorSize> original_features(
        std::move(points_),
        std::move(strengths_),
        std::move(angles_),
        std::move(descriptors_)
    );

    // Create undistorted features
    DummyDistortionModel dummy_model;

    // Create frame features
    CameraFrame camera_frame1(
        std::move(original_features),
        K_,
        &dummy_model
    );

    // Move construct
    CameraFrame camera_frame2(std::move(camera_frame1));

    // Verify that the data was moved correctly
    EXPECT_TRUE(camera_frame2.K().isApprox(K_));
    EXPECT_EQ(camera_frame2.normalized_points().cols(), 3);
    EXPECT_EQ(camera_frame2.normalized_points().rows(), 3);
    EXPECT_TRUE(camera_frame2.points().isApprox(points));
}
