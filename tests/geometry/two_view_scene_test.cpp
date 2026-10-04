#include <gtest/gtest.h>

#include "support/two_view_scene.h"

namespace voyahslam::test {
namespace {

geometry::TMatrix33F Skew(const geometry::TVector3DF& v) {
    geometry::TMatrix33F m;
    m << 0.0, -v.z(), v.y(), v.z(), 0.0, -v.x(), -v.y(), v.x(), 0.0;
    return m;
}

TEST(TwoViewSceneTest, NoiseFreeGeneralSceneSatisfiesEpipolarConstraint) {
    const geometry::RigidTransformation T_21 = DefaultTransform();
    const TwoViewScene scene = MakeGeneralScene(200, 0.3, 0.0, 1, T_21);
    const geometry::TMatrix33F E = Skew(T_21.translation()) * T_21.rotation();
    for (Eigen::Index i = 0; i < scene.correspondences.size(); ++i) {
        if (scene.is_outlier(i)) {
            continue;
        }
        const geometry::TVector3DF x1(scene.correspondences.x1(0, i),
                                      scene.correspondences.x1(1, i), 1.0);
        const geometry::TVector3DF x2(scene.correspondences.x2(0, i),
                                      scene.correspondences.x2(1, i), 1.0);
        EXPECT_LT(std::abs(x2.dot(E * x1)), 1e-10);
    }
}

TEST(TwoViewSceneTest, PlanarPointsLieOnPlane) {
    const TwoViewScene flat = MakePlanarScene(100, 0.0, 0.0, 2, DefaultTransform());
    EXPECT_LT((flat.points.row(2).array() - 5.0).abs().maxCoeff(), 1e-12);

    const TwoViewScene tilted = MakePlanarScene(100, 0.0, 0.0, 2, DefaultTransform(), 15.0);
    const double tilt = 15.0 * M_PI / 180.0;
    const geometry::TVector3DF normal(std::sin(tilt), 0.0, std::cos(tilt));
    const double offset = 5.0 * std::cos(tilt);
    for (Eigen::Index i = 0; i < tilted.points.cols(); ++i) {
        EXPECT_NEAR(normal.dot(tilted.points.col(i)), offset, 1e-12);
    }
}

TEST(TwoViewSceneTest, OutlierFractionRespected) {
    const TwoViewScene scene = MakeGeneralScene(200, 0.3, 0.001, 3, DefaultTransform());
    EXPECT_EQ(scene.is_outlier.count(), 60);
    EXPECT_EQ(scene.correspondences.size(), 200);
}

TEST(TwoViewSceneTest, SameSeedSameScene) {
    const TwoViewScene a = MakeGeneralScene(50, 0.2, 0.002, 7, DefaultTransform());
    const TwoViewScene b = MakeGeneralScene(50, 0.2, 0.002, 7, DefaultTransform());
    EXPECT_EQ(a.correspondences.x1, b.correspondences.x1);
    EXPECT_EQ(a.correspondences.x2, b.correspondences.x2);
    EXPECT_EQ(a.points, b.points);
    EXPECT_TRUE((a.is_outlier == b.is_outlier).all());
    const TwoViewScene c = MakeGeneralScene(50, 0.2, 0.002, 8, DefaultTransform());
    EXPECT_NE(a.correspondences.x1, c.correspondences.x1);
}

TEST(TwoViewSceneTest, HalfFovScalesPointSpread) {
    const auto max_abs = [](const TwoViewScene& a_scene) {
        return a_scene.correspondences.x1.cwiseAbs().maxCoeff();
    };
    const TwoViewScene planar = MakePlanarScene(200, 0.0, 0.0, 4, DefaultTransform());
    const TwoViewScene planar_wide = MakePlanarScene(200, 0.0, 0.0, 4, DefaultTransform(), 0.0, 1.0);
    EXPECT_LE(max_abs(planar), 0.5);
    EXPECT_LE(max_abs(planar_wide), 1.0);
    EXPECT_GT(max_abs(planar_wide), 0.9);
    EXPECT_NEAR(max_abs(planar_wide), 2.0 * max_abs(planar), 1e-12);  // same seed: u, v scale exactly

    const TwoViewScene general_wide = MakeGeneralScene(200, 0.0, 0.0, 5, DefaultTransform(), 1.0);
    EXPECT_LE(max_abs(general_wide), 1.0);
    EXPECT_GT(max_abs(general_wide), 0.9);
}

} // namespace
} // namespace voyahslam::test
