#include <gtest/gtest.h>
#include "voyahslam/features/image_features.h"
#include "voyahslam/defs.h"
#include "voyahslam/features/orb_feature_extractor.h"

using voyahslam::features::ImageFeatures;
using namespace voyahslam::geometry;
using namespace voyahslam::features;

class ImageFeaturesTest : public ::testing::Test {
protected:
    typedef OrbFeatureExtractor FeatureExtractor;
    void SetUp() override {
        // Create test data
        points_ = TPoints2DF::Random(TPoints2DF::RowsAtCompileTime, 3); // 3 points
        strengths_ = TRowVectorF::Random(3);
        angles_ = TRowVectorF::Random( 3);
        descriptors_ = FeatureExtractor::TDescriptors::Random(FeatureExtractor::DescriptorSize, 3);
    }

    TPoints2DF points_;
    TRowVectorF strengths_;
    TRowVectorF angles_;
    FeatureExtractor::TDescriptors descriptors_;
};

TEST_F(ImageFeaturesTest, ConstructorAndGetters) {
    // Create a copy of the data for verification
    auto points_copy = points_;
    auto strengths_copy = strengths_;
    auto angles_copy = angles_;
    auto descriptors_copy = descriptors_;

    // Create ImageFeatures instance
    FeatureExtractor::TFeatures features(
        std::move(points_),
        std::move(strengths_),
        std::move(angles_),
        std::move(descriptors_)
    );

    // Verify that the data was moved correctly
    EXPECT_EQ(features.points().rows(), 2);
    EXPECT_EQ(features.points().cols(), 3);
    EXPECT_TRUE(features.points().isApprox(points_copy));

    EXPECT_EQ(features.strengths().rows(), 1);
    EXPECT_EQ(features.strengths().cols(), 3);
    EXPECT_TRUE(features.strengths().isApprox(strengths_copy));

    EXPECT_EQ(features.angles().rows(), 1);
    EXPECT_EQ(features.angles().cols(), 3);
    EXPECT_TRUE(features.angles().isApprox(angles_copy));

    EXPECT_EQ(features.descriptors().rows(), FeatureExtractor::DescriptorSize);
    EXPECT_EQ(features.descriptors().cols(), 3);
    EXPECT_TRUE(features.descriptors().isApprox(descriptors_copy));
}

TEST_F(ImageFeaturesTest, ConstCorrectness) {
    FeatureExtractor::TFeatures features(
        std::move(points_),
        std::move(strengths_),
        std::move(angles_),
        std::move(descriptors_)
    );

    // Verify that getters return const references
    const auto& points = features.points();
    const auto& strengths = features.strengths();
    const auto& angles = features.angles();
    const auto& descriptors = features.descriptors();

    // These should compile
    EXPECT_EQ(points.rows(), 2);
    EXPECT_EQ(strengths.cols(), 3);
    EXPECT_EQ(angles.cols(), 3);
    EXPECT_EQ(descriptors.rows(), FeatureExtractor::DescriptorSize);
}
