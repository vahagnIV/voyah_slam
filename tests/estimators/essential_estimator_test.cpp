#include <gtest/gtest.h>

#include <array>
#include <cmath>

#include <voyahslam/estimators/essential_estimator.h>

#include "support/two_view_scene.h"

namespace voyahslam::estimators {
namespace {

static_assert(TwoViewEstimator<EssentialEstimator>);

geometry::TMatrix33F Skew(const geometry::TVector3DF& a_t) {
    geometry::TMatrix33F S;
    S << 0, -a_t.z(), a_t.y(), a_t.z(), 0, -a_t.x(), -a_t.y(), a_t.x(), 0;
    return S;
}

geometry::TMatrix33F GroundTruthE(const test::TwoViewScene& a_scene) {
    return Skew(a_scene.T_21.translation()) * a_scene.T_21.rotation();
}

double Residual(const geometry::TMatrix33F& a_E, const geometry::TwoViewCorrespondences& a_c, Eigen::Index a_i) {
    return std::abs(geometry::TVector3DF(a_c.x2(0, a_i), a_c.x2(1, a_i), 1.0).dot(
        a_E * geometry::TVector3DF(a_c.x1(0, a_i), a_c.x1(1, a_i), 1.0)));
}

TEST(EssentialEstimatorTest, ExactFromEightPoints) {
    const auto scene = test::MakeGeneralScene(60, 0.0, 0.0, 3, test::DefaultTransform());
    const std::array<Eigen::Index, 8> sample{0, 7, 13, 21, 30, 38, 47, 55};
    const auto models = EssentialEstimator().Estimate(scene.correspondences, sample);
    ASSERT_EQ(models.size(), 1u);
    const auto& E = models[0];
    for (Eigen::Index i = 0; i < scene.correspondences.size(); ++i) {
        EXPECT_LT(Residual(E, scene.correspondences, i), 1e-9);
    }
    const geometry::TMatrix33F gt = GroundTruthE(scene);
    const geometry::TMatrix33F e = E / E.norm();
    const geometry::TMatrix33F g = gt / gt.norm();
    EXPECT_LT(std::min((e - g).norm(), (e + g).norm()), 1e-6);
}

TEST(EssentialEstimatorTest, SingularValuesOnEssentialManifold) {
    const auto scene = test::MakeGeneralScene(60, 0.0, test::NormalizedSigma(0.5), 4, test::DefaultTransform());
    const std::array<Eigen::Index, 8> sample{1, 8, 14, 22, 31, 39, 48, 56};
    const auto models = EssentialEstimator().Estimate(scene.correspondences, sample);
    ASSERT_EQ(models.size(), 1u);
    const Eigen::JacobiSVD<geometry::TMatrix33F> svd(models[0]);
    const auto& s = svd.singularValues();
    EXPECT_NEAR(s(0), s(1), 1e-9 * s(0));
    EXPECT_LT(s(2), 1e-9 * s(0));
}

TEST(EssentialEstimatorTest, DegenerateSampleGivesNoModel) {
    geometry::TwoViewCorrespondences c;
    c.sigma = 0.01;
    c.x1 = Eigen::Matrix<double, 2, Eigen::Dynamic>::Constant(2, 8, 0.1);
    c.x2.resize(2, 8);
    c.x2 << -0.3, -0.2, -0.1, 0.0, 0.1, 0.2, 0.3, 0.4, 0.1, -0.2, 0.3, 0.0, -0.1, 0.25, 0.05, 0.2;
    const std::array<Eigen::Index, 8> sample{0, 1, 2, 3, 4, 5, 6, 7};
    EXPECT_TRUE(EssentialEstimator().Estimate(c, sample).empty());
}

TEST(EssentialEstimatorTest, IdenticalPointsGiveNoModel) {
    geometry::TwoViewCorrespondences c;
    c.sigma = 0.01;
    c.x1 = Eigen::Matrix<double, 2, Eigen::Dynamic>::Constant(2, 8, 0.2);
    c.x2 = c.x1;
    const std::array<Eigen::Index, 8> sample{0, 1, 2, 3, 4, 5, 6, 7};
    EXPECT_TRUE(EssentialEstimator().Estimate(c, sample).empty());
}

TEST(EssentialEstimatorTest, ConstantSecondViewGivesNoModel) {
    geometry::TwoViewCorrespondences c;
    c.sigma = 0.01;
    c.x2 = Eigen::Matrix<double, 2, Eigen::Dynamic>::Constant(2, 8, 0.1);
    c.x1.resize(2, 8);
    c.x1 << -0.3, -0.2, -0.1, 0.0, 0.1, 0.2, 0.3, 0.4, 0.1, -0.2, 0.3, 0.0, -0.1, 0.25, 0.05, 0.2;
    const std::array<Eigen::Index, 8> sample{0, 1, 2, 3, 4, 5, 6, 7};
    EXPECT_TRUE(EssentialEstimator().Estimate(c, sample).empty());
}

TEST(EssentialEstimatorTest, CollinearFirstViewGivesNoModel) {
    const auto scene = test::MakeGeneralScene(8, 0.0, 0.0, 6, test::DefaultTransform());
    geometry::TwoViewCorrespondences c = scene.correspondences;
    for (Eigen::Index i = 0; i < 8; ++i) c.x1.col(i) = geometry::TVector2DF(0.1 * i, 0.2 * i + 0.05);
    const std::array<Eigen::Index, 8> sample{0, 1, 2, 3, 4, 5, 6, 7};
    EXPECT_TRUE(EssentialEstimator().Estimate(c, sample).empty());
}

TEST(EssentialEstimatorTest, PlanarSampleGivesNoModel) {
    const auto scene = test::MakePlanarScene(40, 0.0, 0.0, 7, test::DefaultTransform());
    const std::array<Eigen::Index, 8> sample{0, 5, 10, 15, 20, 25, 30, 35};
    const auto models = EssentialEstimator().Estimate(scene.correspondences, sample);
    // A planar sample leaves the 8-point system rank deficient; it must be rejected.
    EXPECT_TRUE(models.empty());
}

TEST(EssentialEstimatorTest, TrueModelOutscoresPerturbed) {
    const auto scene = test::MakeGeneralScene(100, 0.0, test::NormalizedSigma(0.5), 5, test::DefaultTransform());
    const geometry::TMatrix33F E = GroundTruthE(scene);
    const auto good = EssentialEstimator().Score(E, scene.correspondences);
    geometry::TMatrix33F perturbed = E;
    perturbed(0, 2) += 0.05 * E.norm();
    perturbed(2, 1) -= 0.05 * E.norm();
    const auto bad = EssentialEstimator().Score(perturbed, scene.correspondences);
    EXPECT_GT(good.score, bad.score);
    EXPECT_GT(good.num_inliers, 75u);  // both directions pass at 3.841: ~90% expected
}

TEST(EssentialEstimatorTest, HandComputedScore) {
    const geometry::TMatrix33F E = Skew(geometry::TVector3DF(1.0, 0.0, 0.0));
    geometry::TwoViewCorrespondences c;
    c.sigma = 0.01;
    c.x1.resize(2, 1);
    c.x1 << 0.2, 0.1;
    c.x2.resize(2, 1);
    c.x2 << 0.5, 0.11;
    const auto result = EssentialEstimator().Score(E, c);
    EXPECT_NEAR(result.score, 2 * (5.991 - 1.0), 1e-9);
    EXPECT_EQ(result.num_inliers, 1u);
    EXPECT_TRUE(result.inliers(0));
}

TEST(EssentialEstimatorTest, EssentialScoreFiniteForZeroEpipolarLine) {
    const geometry::TMatrix33F E = Skew(geometry::TVector3DF(0.0, 0.0, 1.0));  // E * (0,0,1) = 0
    geometry::TwoViewCorrespondences c;
    c.sigma = 0.01;
    c.x1.resize(2, 1);
    c.x1 << 0.0, 0.0;
    c.x2.resize(2, 1);
    c.x2 << 0.3, 0.2;
    const auto result = EssentialEstimator().Score(E, c);
    EXPECT_TRUE(std::isfinite(result.score));
    EXPECT_EQ(result.num_inliers, 0u);
    EXPECT_FALSE(result.inliers(0));
}

} // namespace
} // namespace voyahslam::estimators
