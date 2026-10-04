#include <gtest/gtest.h>

#include <array>

#include <voyahslam/estimators/homography_estimator.h>

#include "support/two_view_scene.h"

namespace voyahslam::estimators {
namespace {

static_assert(TwoViewEstimator<HomographyEstimator>);

geometry::TwoViewCorrespondences Identity2(double a_offset_x) {
    geometry::TwoViewCorrespondences c;
    c.sigma = 0.01;
    c.x1.resize(2, 2);
    c.x1 << 0.1, -0.2, 0.3, 0.4;
    c.x2 = c.x1;
    c.x2(0, 1) += a_offset_x;
    return c;
}

double SummedTransferError(const geometry::TMatrix33F& a_H, const geometry::TwoViewCorrespondences& a_c,
                           const Eigen::Array<bool, Eigen::Dynamic, 1>& a_mask) {
    const geometry::TMatrix33F H_inv = a_H.inverse();
    double sum = 0.0;
    for (Eigen::Index i = 0; i < a_c.size(); ++i) {
        if (!a_mask(i)) continue;
        const geometry::TVector3DF p2 = a_H * geometry::TVector3DF(a_c.x1(0, i), a_c.x1(1, i), 1.0);
        const geometry::TVector3DF p1 = H_inv * geometry::TVector3DF(a_c.x2(0, i), a_c.x2(1, i), 1.0);
        sum += (a_c.x2.col(i) - p2.head<2>() / p2.z()).squaredNorm() +
               (a_c.x1.col(i) - p1.head<2>() / p1.z()).squaredNorm();
    }
    return sum;
}

TEST(HomographyEstimatorTest, ExactFromFourPlanarPoints) {
    const auto scene = test::MakePlanarScene(50, 0.0, 0.0, 1, test::DefaultTransform());
    const std::array<Eigen::Index, 4> sample{0, 10, 20, 30};
    const auto models = HomographyEstimator().Estimate(scene.correspondences, sample);
    ASSERT_EQ(models.size(), 1u);
    const auto& H = models[0];
    for (Eigen::Index i = 0; i < scene.correspondences.size(); ++i) {
        const geometry::TVector3DF p = H * geometry::TVector3DF(scene.correspondences.x1(0, i),
                                                                scene.correspondences.x1(1, i), 1.0);
        EXPECT_LT((scene.correspondences.x2.col(i) - p.head<2>() / p.z()).norm(), 1e-9);
    }
}

TEST(HomographyEstimatorTest, CollinearSampleGivesNoModel) {
    geometry::TwoViewCorrespondences c;
    c.sigma = 0.01;
    c.x1.resize(2, 4);
    c.x1 << 0.0, 0.1, 0.2, 0.3, 0.0, 0.1, 0.2, 0.3;
    c.x2.resize(2, 4);
    c.x2 << 0.05, 0.12, 0.26, 0.31, 0.0, 0.2, 0.1, 0.3;
    const std::array<Eigen::Index, 4> sample{0, 1, 2, 3};
    EXPECT_TRUE(HomographyEstimator().Estimate(c, sample).empty());
}

TEST(HomographyEstimatorTest, IdenticalPointsGiveNoModel) {
    geometry::TwoViewCorrespondences c;
    c.sigma = 0.01;
    c.x1 = geometry::TPoints2DF::Constant(2, 4, 0.2);
    c.x2 = geometry::TPoints2DF::Constant(2, 4, 0.3);
    const std::array<Eigen::Index, 4> sample{0, 1, 2, 3};
    EXPECT_TRUE(HomographyEstimator().Estimate(c, sample).empty());
}

TEST(HomographyEstimatorTest, TrueModelOutscoresPerturbed) {
    const auto scene = test::MakePlanarScene(100, 0.0, test::NormalizedSigma(1.0), 2, test::DefaultTransform());
    std::vector<Eigen::Index> sample{0, 20, 40, 60};
    HomographyEstimator estimator;
    const auto models = estimator.Estimate(scene.correspondences, sample);
    ASSERT_EQ(models.size(), 1u);
    geometry::TMatrix33F perturbed = models[0];
    perturbed(0, 2) += 0.05;
    const auto good = estimator.Score(models[0], scene.correspondences);
    const auto bad = estimator.Score(perturbed, scene.correspondences);
    EXPECT_GT(good.score, bad.score);
    EXPECT_GT(good.num_inliers, bad.num_inliers);
}

TEST(HomographyEstimatorTest, HandComputedScore) {
    const auto c = Identity2(0.02);
    const auto s = HomographyEstimator().Score(geometry::TMatrix33F::Identity(), c);
    EXPECT_NEAR(s.score, 2 * 5.991 + 2 * 1.991, 1e-9);
    EXPECT_EQ(s.num_inliers, 2u);
    EXPECT_TRUE(s.inliers(0));
    EXPECT_TRUE(s.inliers(1));
}

TEST(HomographyEstimatorTest, HomographyScoreFiniteForPointMappedToInfinity) {
    const auto c = Identity2(0.0);
    geometry::TMatrix33F H = geometry::TMatrix33F::Identity();
    H.row(2) << 1.0, 0.0, -0.1;  // w = x - 0.1 -> zero for x1 column 0
    const auto s = HomographyEstimator().Score(H, c);
    EXPECT_TRUE(std::isfinite(s.score));
    EXPECT_FALSE(s.inliers(0));
}

TEST(HomographyEstimatorTest, ThreeCollinearSampleGivesNoModel) {
    geometry::TwoViewCorrespondences c;
    c.sigma = 0.01;
    c.x1.resize(2, 4);
    c.x1 << 0.0, 0.1, 0.2, 0.3, 0.0, 0.1, 0.2, -0.3;
    c.x2.resize(2, 4);
    c.x2 << 0.05, -0.12, 0.26, 0.31, 0.02, 0.2, -0.1, 0.3;
    const std::array<Eigen::Index, 4> sample{0, 1, 2, 3};
    EXPECT_TRUE(HomographyEstimator().Estimate(c, sample).empty());
}

TEST(HomographyEstimatorTest, SingularModelScoresZero) {
    auto c = Identity2(0.0);
    c.x2.colwise() = geometry::TPoint2DF(1.0, 0.5);  // all points map onto the rank-1 image v
    const geometry::TVector3DF v(1.0, 0.5, 1.0), l(0.3, -0.2, 1.0);
    const geometry::TMatrix33F H = v * l.transpose() + 1e-15 * geometry::TMatrix33F::Identity();
    const auto s = HomographyEstimator().Score(H, c);
    EXPECT_EQ(s.score, 0.0);
    EXPECT_EQ(s.num_inliers, 0u);
}

TEST(HomographyEstimatorTest, RefineLowersTransferError) {
    const auto scene = test::MakePlanarScene(200, 0.0, test::NormalizedSigma(1.0), 23, test::DefaultTransform());
    const std::array<Eigen::Index, 4> sample{0, 50, 100, 150};
    const HomographyEstimator estimator;
    const auto models = estimator.Estimate(scene.correspondences, sample);
    ASSERT_EQ(models.size(), 1u);
    const Eigen::Array<bool, Eigen::Dynamic, 1> inliers = !scene.is_outlier;

    const geometry::TMatrix33F refined = estimator.Refine(models[0], scene.correspondences, inliers);
    ASSERT_TRUE(refined.allFinite());
    EXPECT_LT(SummedTransferError(refined, scene.correspondences, inliers),
              SummedTransferError(models[0], scene.correspondences, inliers));
    const Eigen::JacobiSVD<geometry::TMatrix33F> svd(refined);
    EXPECT_GT(svd.singularValues()(2), 1e-8 * svd.singularValues()(0));
}

TEST(HomographyEstimatorTest, RefineKeepsModelOnDegenerateInput) {
    const auto scene = test::MakePlanarScene(50, 0.0, test::NormalizedSigma(1.0), 24, test::DefaultTransform());
    const std::array<Eigen::Index, 4> sample{0, 10, 20, 30};
    const HomographyEstimator estimator;
    const auto models = estimator.Estimate(scene.correspondences, sample);
    ASSERT_EQ(models.size(), 1u);
    Eigen::Array<bool, Eigen::Dynamic, 1> inliers = Eigen::Array<bool, Eigen::Dynamic, 1>::Constant(50, false);
    EXPECT_EQ(estimator.Refine(models[0], scene.correspondences, inliers), models[0]);
    inliers.head(3).setConstant(true);  // fewer than kSampleSize
    EXPECT_EQ(estimator.Refine(models[0], scene.correspondences, inliers), models[0]);
}

} // namespace
} // namespace voyahslam::estimators
