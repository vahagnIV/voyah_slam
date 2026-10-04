#include <gtest/gtest.h>

#include <voyahslam/estimators/essential_estimator.h>
#include <voyahslam/estimators/homography_estimator.h>
#include <voyahslam/estimators/ransac.h>

#include "support/two_view_scene.h"

namespace voyahslam::estimators {
namespace {

void ExpectRecovered(const test::TwoViewScene& a_scene, const ModelScore& a_score) {
    size_t true_inliers = 0, inliers_found = 0, outliers = 0, outliers_flagged = 0;
    for (Eigen::Index i = 0; i < a_scene.is_outlier.size(); ++i) {
        if (a_scene.is_outlier(i)) {
            ++outliers;
            outliers_flagged += a_score.inliers(i) ? 1 : 0;
        } else {
            ++true_inliers;
            inliers_found += a_score.inliers(i) ? 1 : 0;
        }
    }
    EXPECT_GE(static_cast<double>(inliers_found), 0.50 * static_cast<double>(true_inliers));
    EXPECT_LE(static_cast<double>(outliers_flagged), 0.05 * static_cast<double>(outliers));
}

// Bound is 50%: minimal-sample models overfit noise (no local optimization yet; TODO).
void CheckEssentialRecovery(double a_outlier_fraction) {
    const auto scene = test::MakeGeneralScene(300, a_outlier_fraction, test::NormalizedSigma(1.0), 11,
                                              test::DefaultTransform());
    const auto result = Ransac<EssentialEstimator>().Run(scene.correspondences, RansacOptions{});
    ASSERT_TRUE(result.success);
    ExpectRecovered(scene, result.score);
}

TEST(RansacTest, RecoversEssentialWith30PercentOutliers) { CheckEssentialRecovery(0.3); }

TEST(RansacTest, RecoversEssentialWith50PercentOutliers) { CheckEssentialRecovery(0.5); }

TEST(RansacTest, RecoversHomographyOnPlanarScene) {
    const auto scene = test::MakePlanarScene(300, 0.3, test::NormalizedSigma(1.0), 12, test::DefaultTransform());
    const auto result = Ransac<HomographyEstimator>().Run(scene.correspondences, RansacOptions{});
    ASSERT_TRUE(result.success);
    ExpectRecovered(scene, result.score);
}

TEST(RansacTest, CleanDataStopsEarly) {
    auto scene = test::MakeGeneralScene(300, 0.0, 0.0, 13, test::DefaultTransform());
    scene.correspondences.sigma = test::NormalizedSigma(1.0);
    const auto result = Ransac<EssentialEstimator>().Run(scene.correspondences, RansacOptions{});
    ASSERT_TRUE(result.success);
    EXPECT_LT(result.iterations, 200u);
}

TEST(RansacTest, SameSeedSameResult) {
    const auto scene = test::MakeGeneralScene(300, 0.4, test::NormalizedSigma(1.0), 14, test::DefaultTransform());
    RansacOptions options;
    options.seed = 7;
    const auto a = Ransac<EssentialEstimator>().Run(scene.correspondences, options);
    const auto b = Ransac<EssentialEstimator>().Run(scene.correspondences, options);
    EXPECT_EQ(a.iterations, b.iterations);
    EXPECT_EQ(a.model, b.model);
    EXPECT_EQ(a.score.num_inliers, b.score.num_inliers);
}

TEST(RansacTest, EmptyInputFails) {
    geometry::TwoViewCorrespondences c;
    c.sigma = 0.01;
    c.x1.resize(2, 0);
    c.x2.resize(2, 0);
    EXPECT_FALSE(Ransac<EssentialEstimator>().Run(c, RansacOptions{}).success);
}

TEST(RansacTest, FewerThanSampleSizeFails) {
    const auto scene = test::MakeGeneralScene(7, 0.0, 0.0, 15, test::DefaultTransform());
    EXPECT_FALSE(Ransac<EssentialEstimator>().Run(scene.correspondences, RansacOptions{}).success);
}

TEST(RansacTest, ExactlySampleSizeRuns) {
    auto scene = test::MakeGeneralScene(8, 0.0, 0.0, 16, test::DefaultTransform());
    scene.correspondences.sigma = test::NormalizedSigma(1.0);
    const auto result = Ransac<EssentialEstimator>().Run(scene.correspondences, RansacOptions{});
    ASSERT_TRUE(result.success);
    EXPECT_EQ(result.score.num_inliers, 8u);
}

} // namespace
} // namespace voyahslam::estimators
