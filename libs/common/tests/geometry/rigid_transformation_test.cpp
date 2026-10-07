#include <gtest/gtest.h>

#include "voyahslam/geometry/rigid_transformation.h"

namespace voyahslam::geometry {
namespace {

RigidTransformation RandomTransform() {
    const TMatrix33F rotation = Eigen::Quaternion<precision_t>::UnitRandom().toRotationMatrix();
    return RigidTransformation(rotation, TVector3DF::Random());
}

TEST(RigidTransformationTest, DefaultIsIdentity) {
    const RigidTransformation transform;
    EXPECT_EQ(transform.rotation(), TMatrix33F::Identity());
    EXPECT_EQ(transform.translation(), TVector3DF::Zero());
    EXPECT_EQ(RigidTransformation::Identity().rotation(), TMatrix33F::Identity());
}

TEST(RigidTransformationTest, InverseRoundTrip) {
    const RigidTransformation transform = RandomTransform();
    const RigidTransformation product = transform * transform.Inverse();
    EXPECT_TRUE(product.rotation().isApprox(TMatrix33F::Identity(), 1e-12));
    EXPECT_NEAR(product.translation().norm(), 0.0, 1e-12);
}

TEST(RigidTransformationTest, CompositionOrder) {
    const RigidTransformation t_ab = RandomTransform();
    const RigidTransformation t_bc = RandomTransform();
    const TPoint3DF point = TPoint3DF::Random();
    const TPoint3DF composed = (t_ab * t_bc).Transform(point);
    const TPoint3DF sequential = t_ab.Transform(t_bc.Transform(point));
    EXPECT_NEAR((composed - sequential).norm(), 0.0, 1e-12);
}

TEST(RigidTransformationTest, TransformPointsMatchesColumns) {
    const RigidTransformation transform = RandomTransform();
    const TPoints3DF points = TPoints3DF::Random(3, 7);
    const TPoints3DF result = transform.Transform(points);
    ASSERT_EQ(result.cols(), 7);
    for (Eigen::Index i = 0; i < points.cols(); ++i) {
        const TPoint3DF expected = transform.Transform(TPoint3DF(points.col(i)));
        EXPECT_NEAR((result.col(i) - expected).norm(), 0.0, 1e-12);
    }
}

TEST(RigidTransformationTest, MatrixExports) {
    const RigidTransformation transform = RandomTransform();
    const TMatrix34F m34 = transform.ToMatrix34();
    const TMatrix33F m34_rotation = m34.leftCols<3>();
    const TVector3DF m34_translation = m34.col(3);
    EXPECT_EQ(m34_rotation, transform.rotation());
    EXPECT_EQ(m34_translation, transform.translation());
    const TMatrix44F m44 = transform.ToMatrix44();
    const TMatrix34F m44_top = m44.topRows<3>();
    const Eigen::Matrix<precision_t, 1, 4> bottom_row = m44.row(3);
    const Eigen::Matrix<precision_t, 1, 4> expected_row(0, 0, 0, 1);
    EXPECT_EQ(m44_top, m34);
    EXPECT_EQ(bottom_row, expected_row);
}

} // namespace
} // namespace voyahslam::geometry
