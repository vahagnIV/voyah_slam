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

} // namespace
} // namespace voyahslam::estimators
