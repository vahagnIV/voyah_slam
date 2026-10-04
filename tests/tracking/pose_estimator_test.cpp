#include <gtest/gtest.h>
#include <support/test_measurements.h>
#include <voyahslam/tracking/pose_estimator.h>

namespace voyahslam::tracking {
namespace {

using Extractor = test::TestExtractor;
using Measurement = features::TMeasurement<Extractor>;

geometry::RigidTransformation NonIdentityPose() {
    geometry::TVector3DF translation(1.5F, -2.0F, 0.0F);
    return geometry::RigidTransformation(geometry::TMatrix33F::Identity(), translation);
}

TEST(PoseEstimatorTest, SucceedsWithEnoughFeatures) {
    PoseEstimator<Extractor> estimator;
    geometry::RigidTransformation last = NonIdentityPose();
    auto result = estimator.Estimate(Measurement(test::MakeMonocular(50, 0.0)), last);
    EXPECT_TRUE(result.success);
    EXPECT_TRUE(result.pose.ToMatrix34().isApprox(last.ToMatrix34()));
}

TEST(PoseEstimatorTest, FailsWithTooFewFeatures) {
    PoseEstimator<Extractor> estimator;
    auto result = estimator.Estimate(Measurement(test::MakeMonocular(49, 0.0)), NonIdentityPose());
    EXPECT_FALSE(result.success);
}

TEST(PoseEstimatorTest, UsesLeftCameraForStereo) {
    PoseEstimator<Extractor> estimator;
    EXPECT_TRUE(estimator.Estimate(Measurement(test::MakeBinocular(50, 0.0)), NonIdentityPose()).success);
    EXPECT_FALSE(estimator.Estimate(Measurement(test::MakeBinocular(0, 0.0)), NonIdentityPose()).success);
}

} // namespace
} // namespace voyahslam::tracking
