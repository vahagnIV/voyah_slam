#include <gtest/gtest.h>
#include "voyahslam/geometry/geometry.h"
#include "voyahslam/algebra/algebra.h"
#include "utils_test.h"
#include <Eigen/Dense>
#include <array>

using namespace voyahslam;
using namespace voyahslam::test;
using namespace voyahslam::geometry;

class GeometryTest : public ::testing::Test {
protected:
    void SetUp() override {}
};

TEST_F(GeometryTest, SquaredSampsonErrorZero) {
    TwoViewSystem sys;
    // Known rotation and translation (along x)
    sys.R = CreateRotationMatrixD(10, 7, 12);
    sys.T << 1, 0, 0;
    TMatrix33F E_true = CrossProductMatrix(sys.T) * sys.R;
    // Generate random 3D points in front of the camera
    int N = 8;
    sys.triangulated = CreateRandomMatrix(3, N);
    sys.triangulated.row(2).array() += 5.0; // Ensure points are in front of camera
    // Normalized points in first camera
    auto nps1 = sys.triangulated.colwise().hnormalized(); // 2xn
    sys.nps1 = nps1.colwise().homogeneous(); // 3xn
    // Normalized points in second camera (R, t)
    TPoints3DF X2 = sys.R * sys.triangulated;
    X2.colwise() += sys.T;
    auto nps2 = X2.colwise().hnormalized(); // 2xn
    sys.nps2 = nps2.colwise().homogeneous(); // 3xn
    // Sampson error should be near zero for perfect correspondences
    auto err = SquaredSampsonError(sys.nps1, sys.nps2, E_true);
    ASSERT_EQ(err.size(), N);
    EXPECT_TRUE(err.cwiseAbs().maxCoeff() < 1e-15);
}

TEST_F(GeometryTest, SquaredSampsonErrorNominal) {
    // Example taken from src/colmap/estimators/utils_test.cc in
    // COLMAP github.
    TwoViewSystem sys;
    // Known rotation and translation (along x)
    sys.R = TMatrix33F::Identity();
    sys.T << 1, 0, 0;
    TMatrix33F E_true = CrossProductMatrix(sys.T) * sys.R;
    // Add points
    sys.nps1 = THomogenousPointsF::Zero(3, 3);
    sys.nps1.row(2).setOnes();
    sys.nps2 = THomogenousPointsF::Ones(3, 3);
    sys.nps2.row(0) << 2, 2, 2;
    sys.nps2.row(1) << 0, 1, 2;
    auto err = SquaredSampsonError(sys.nps1, sys.nps2, E_true);
    TColVectorF expected_err(3);
    expected_err << 0, 0.5, 2; // Expected errors for the given points
    EXPECT_TRUE((err - expected_err).isZero());
}

TEST_F(GeometryTest, SquaredSampsonErrorScaleIndependent) {
    TwoViewSystem sys;
    // Known rotation and translation (along x)
    sys.R = TMatrix33F::Identity();
    sys.T << 1, 0, 0;
    TMatrix33F E_true = CrossProductMatrix(sys.T) * sys.R;
    // Add points
    sys.nps1 = CreateRandomMatrix(3, 100);
    sys.nps2 = CreateRandomMatrix(3, 100);
    auto err = SquaredSampsonError(sys.nps1, sys.nps2, E_true);

    // Scale the essential matrix
    TMatrix33F E_scaled = E_true * 2.432432; //random scale
    auto err_e_scaled = SquaredSampsonError(sys.nps1, sys.nps2, E_scaled);
    EXPECT_TRUE((err - err_e_scaled).isZero(1e-6));

    // Scale first image  points
    TRowArrayF rand_scales_nps1 = CreateRandomMatrix(1, sys.nps1.cols());
    THomogenousPointsF nps1_scaled = sys.nps1.array().rowwise() * rand_scales_nps1;
    auto err_nps1_scaled = SquaredSampsonError(nps1_scaled, sys.nps2, E_true);
    EXPECT_TRUE((err - err_nps1_scaled).isZero(1e-6));

    // Scale second image  points
    TRowArrayF rand_scales_nps2 = CreateRandomMatrix(1, sys.nps2.cols());
    THomogenousPointsF nps2_scaled = sys.nps2.array().rowwise() * rand_scales_nps2;
    auto err_nps2_scaled = SquaredSampsonError(sys.nps1, nps2_scaled, E_true);
    EXPECT_TRUE((err - err_nps2_scaled).isZero(1e-6));
}

TEST_F(GeometryTest, TriangulateNominal) {
    geometry::TPoint3DF expected_triangulated(-0.2, 0.1, 4);
    auto R = CreateRotationMatrixD(10, 7, 12);
    geometry::TVector3DF T(0.3, 0.4, 0.5);

    auto normalized1 = expected_triangulated / expected_triangulated.z();
    geometry::THomogenousPointF normalized2 = R * expected_triangulated + T;
    normalized2 /= normalized2.z();

    TMatrix34F P1 = TMatrix34F::Zero();
    P1.block<3,3>(0,0) = TMatrix33F::Identity();
    TMatrix34F P2 = TMatrix34F::Zero();
    P2 << R, T;

    geometry::TPoint3DF triangulated;
    ASSERT_TRUE(
        geometry::Triangulate(normalized1, normalized2, P1, P2, triangulated));

    precision_t diff = (triangulated - expected_triangulated).norm();
    ASSERT_LE(diff, 1e-13);
}

TEST_F(GeometryTest, DecomposeEssentialToPoses) {
    // Create a random valid essential matrix, which a basically a matrix
    // with two equal and one 0 singular values
    TMatrix33F E = CreateRandomMatrix(3, 3);
    Eigen::BDCSVD<TMatrix33F> svd(E, Eigen::ComputeFullU | Eigen::ComputeFullV);
    TVector3DF singularValues = svd.singularValues();
    singularValues(2) = 0;  // Enforce rank 2 on E
    singularValues(0) = singularValues(1);
    E = svd.matrixU() * singularValues.asDiagonal() * svd.matrixV().transpose();

    // Decompose the matrix
    auto poses = DecomposeEssentialToPoses(E);
    ASSERT_EQ(poses.size(), 4);
    auto E_scaled = E / E.norm();
    // check all possible posses compose the scaled version of inital E
    for (int i = 0; i < 4; ++i) {
        TMatrix33F R = poses[i].block<3,3>(0,0);
        TVector3DF t = poses[i].block<3,1>(0,3);
        EXPECT_NEAR(R.determinant(), 1.0f, 1e-5f);
        EXPECT_NEAR(t.norm(), 1.0f, 1e-5f);
        // Check that the pose recovers the essential matrix (up to scale)
        TMatrix33F E_recovered = CrossProductMatrix(t) * R;
        E_recovered /= E_recovered.norm();
        EXPECT_TRUE(E_recovered.isApprox(E_scaled) or E_recovered.isApprox(-E_scaled));
    }
}
