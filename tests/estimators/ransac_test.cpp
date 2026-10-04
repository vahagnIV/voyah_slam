#include <gtest/gtest.h>

#include <cstddef>
#include <span>
#include <vector>

#include <voyahslam/estimators/essential_estimator.h>
#include <voyahslam/estimators/homography_estimator.h>
#include <voyahslam/estimators/ransac.h>

#include "support/two_view_scene.h"

namespace voyahslam::estimators {
namespace {

struct Recovery {
    double inlier_fraction = 0.0;   ///< True inliers flagged / true inliers.
    double outlier_fraction = 0.0;  ///< Outliers flagged / outliers.
};

Recovery MeasureRecovery(const test::TwoViewScene& a_scene, const ModelScore& a_score) {
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
    Recovery recovery;
    recovery.inlier_fraction = static_cast<double>(inliers_found) / static_cast<double>(true_inliers);
    if (outliers > 0) {
        recovery.outlier_fraction = static_cast<double>(outliers_flagged) / static_cast<double>(outliers);
    }
    return recovery;
}

void ExpectRecovered(const test::TwoViewScene& a_scene, const ModelScore& a_score) {
    const Recovery recovery = MeasureRecovery(a_scene, a_score);
    EXPECT_GE(recovery.inlier_fraction, 0.50);
    EXPECT_LE(recovery.outlier_fraction, 0.05);
}

// Bound is 50% (ledger ruling, set before local refinement existed); refinement is now on by
// default, and RefinementImprovesRecovery checks the tighter bound.
void CheckEssentialRecovery(double a_outlier_fraction) {
    const auto scene = test::MakeGeneralScene(300, a_outlier_fraction, test::NormalizedSigma(1.0), 11,
                                              test::DefaultTransform());
    const auto result = Ransac<EssentialEstimator>().Run(scene.correspondences, RansacOptions{});
    ASSERT_TRUE(result.success);
    ExpectRecovered(scene, result.score);
}

// Refinement over all inliers removes the minimal-sample overfit (spec 3.5). Wide-FOV scene
// (half_fov_tan = 1.0): in the default narrow scene, 8-point RANSAC often ends in a wrong basin
// that refinement cannot leave (ledger ruling). Refined recovery must not fall below unrefined.
void CheckRefinedEssentialRecovery(double a_outlier_fraction, double a_min_inlier_fraction) {
    const auto scene = test::MakeGeneralScene(300, a_outlier_fraction, test::NormalizedSigma(1.0), 11,
                                              test::DefaultTransform(), 1.0);
    RansacOptions unrefined_options;
    unrefined_options.refinement_rounds = 0;
    const auto unrefined = Ransac<EssentialEstimator>().Run(scene.correspondences, unrefined_options);
    const auto refined = Ransac<EssentialEstimator>().Run(scene.correspondences, RansacOptions{});
    ASSERT_TRUE(unrefined.success);
    ASSERT_TRUE(refined.success);

    const Recovery before = MeasureRecovery(scene, unrefined.score);
    const Recovery after = MeasureRecovery(scene, refined.score);
    EXPECT_GE(after.inlier_fraction, a_min_inlier_fraction);
    EXPECT_LE(after.outlier_fraction, 0.05);
    EXPECT_GE(after.inlier_fraction, before.inlier_fraction);
}

// Every model scores exactly one inlier, so the all-inlier sample probability w^k underflows.
struct OneInlierEstimator {
    using TModel = int;
    static constexpr size_t kSampleSize = 8;

    [[nodiscard]] std::vector<TModel> Estimate(const geometry::TwoViewCorrespondences& /*a_correspondences*/,
                                               std::span<const Eigen::Index> /*a_sample*/) const {
        return {0};
    }

    [[nodiscard]] ModelScore Score(const TModel& /*a_model*/,
                                   const geometry::TwoViewCorrespondences& a_correspondences) const {
        ModelScore score;
        score.inliers = Eigen::Array<bool, Eigen::Dynamic, 1>::Constant(a_correspondences.size(), false);
        score.inliers(0) = true;
        score.num_inliers = 1;
        score.score = 1.0;
        return score;
    }
};

static_assert(TwoViewEstimator<OneInlierEstimator>);

geometry::TwoViewCorrespondences MakeZeroCorrespondences(Eigen::Index a_n) {
    geometry::TwoViewCorrespondences correspondences;
    correspondences.x1 = geometry::TPoints2DF::Zero(2, a_n);
    correspondences.x2 = geometry::TPoints2DF::Zero(2, a_n);
    correspondences.sigma = 0.01;
    return correspondences;
}

TEST(RansacTest, TinyInlierRatioRunsMaxIterations) {
    // (1/300)^8 ~ 1.5e-20: 1 - w^k rounds to 1, which must not end the run at min_iterations.
    const RansacOptions options;
    const auto result = Ransac<OneInlierEstimator>().Run(MakeZeroCorrespondences(300), options);
    ASSERT_TRUE(result.success);
    EXPECT_EQ(result.iterations, options.max_iterations);
}

TEST(RansacTest, MinIterationsAboveMaxIsBoundedByMax) {
    RansacOptions options;
    options.max_iterations = 10;
    options.min_iterations = 50;
    const auto result = Ransac<OneInlierEstimator>().Run(MakeZeroCorrespondences(300), options);
    ASSERT_TRUE(result.success);
    EXPECT_LE(result.iterations, options.max_iterations);
}

TEST(RansacTest, RecoversEssentialWith30PercentOutliers) { CheckEssentialRecovery(0.3); }

TEST(RansacTest, RecoversEssentialWith50PercentOutliers) { CheckEssentialRecovery(0.5); }

TEST(RansacTest, RefinementImprovesRecoveryWith30PercentOutliers) { CheckRefinedEssentialRecovery(0.3, 0.75); }

// Bound = minimum observed - 0.05. Refined recovery at seeds 11-18 (half_fov_tan = 1.0):
// 0.793, 0.520, 0.793, 0.760, 0.807, 0.827, 0.827, 0.573 (seed 12 stays in a wrong basin).
TEST(RansacTest, RefinementImprovesRecoveryWith50PercentOutliers) { CheckRefinedEssentialRecovery(0.5, 0.47); }

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
