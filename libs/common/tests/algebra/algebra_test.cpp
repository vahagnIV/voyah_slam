#include <random>
#include <gtest/gtest.h>
#include "voyahslam/algebra/algebra.h"
#include "utils_test.h"

using namespace voyahslam;
using namespace voyahslam::test;
using namespace voyahslam::algebra;

class HartleyNormalizationTest : public ::testing::Test {
protected:
    void SetUp() override {
        pts = CreateRandomMatrix(2, 100);
    }
    geometry::TPoints2DF pts;
};

// Test that the normalization matrix normalizes centroid to (0,0) and mean distance to sqrt(2)
TEST_F(HartleyNormalizationTest, NormalizesCentroidAndScale) {
    auto T = GetHartleyNormalizationMatrix(pts);
    // Transform points
    Eigen::Matrix<voyahslam::precision_t, 3, -1> pts_h(3, pts.cols());
    pts_h.topRows(2) = pts;
    pts_h.row(2).setOnes();
    auto normed = T * pts_h;
    // Centroid should be (0,0)
    Eigen::Vector2d centroid = normed.topRows(2).rowwise().mean();
    EXPECT_NEAR(centroid(0), 0.0, 1e-12);
    EXPECT_NEAR(centroid(1), 0.0, 1e-12);
    // Mean distance to origin should be sqrt(2)
    double mean_dist = (normed.topRows(2).colwise().squaredNorm()).mean();
    mean_dist = std::sqrt(mean_dist);
    EXPECT_NEAR(mean_dist, std::sqrt(2.0), 1e-12);
}

TEST(SolveHomogeneousSystem, SquareRankFull) {
    int count = 1000;
    double err = 0;
    Eigen::Index rows = 9;
    Eigen::Index cols = 9;
    for (int i = 0; i < count; ++i){
        Eigen::MatrixXd A = CreateRandomMatrix(rows, cols);
        ASSERT_EQ(GetMatrixRank(A), 9);
        auto X = algebra::SolveHomogeneousSystem(A);
        ASSERT_FALSE(X.isZero());
        err += (A*X).norm();
    }
    err = err / count;
    ASSERT_LE(err, 0.08);
}

TEST(SolveHomogeneousSystem, SquareRankDeficient) {
    int count = 1000;
    double err = 0;
    Eigen::Index rows = 9;
    Eigen::Index cols = 9;
    for (int i = 0; i < count; ++i){
        Eigen::MatrixXd A = CreateRandomMatrix(rows, cols);
        auto B = algebra::ReduceMatrixRank(A, 1);
        ASSERT_EQ(GetMatrixRank(B), 8);
        auto X = algebra::SolveHomogeneousSystem(B);
        ASSERT_FALSE(X.isZero());
        err += (B*X).norm();
    }
    err = err / count;
    ASSERT_LE(err, 1e-15);
}

TEST(SolveHomogeneousSystem, RectRankFull) {
    int count = 1000;
    double err = 0;
    Eigen::Index rows = 15;
    Eigen::Index cols = 9;
    for (int i = 0; i < count; ++i){
        Eigen::MatrixXd A = CreateRandomMatrix(rows, cols);
        ASSERT_EQ(GetMatrixRank(A), 9);
        auto X = algebra::SolveHomogeneousSystem(A);
        ASSERT_FALSE(X.isZero());
        err += (A*X).norm();
    }
    err = err / count;
    ASSERT_LE(err, 0.4);
}

TEST(SolveHomogeneousSystem, RectRankDeficient) {
    int count = 1000;
    double err = 0;
    Eigen::Index rows = 15;
    Eigen::Index cols = 9;
    for (int i = 0; i < count; ++i){
        Eigen::MatrixXd A = CreateRandomMatrix(rows, cols);
        auto B = algebra::ReduceMatrixRank(A, 1);
        ASSERT_EQ(GetMatrixRank(B), 8);
        auto X = algebra::SolveHomogeneousSystem(B);
        ASSERT_FALSE(X.isZero());
        err += (B*X).norm();
    }
    err = err / count;
    ASSERT_LE(err, 1e-15);
}

TEST(ReduceMatrixRank, Square) {
    Eigen::Index rows = 100;
    Eigen::Index cols = 100;
    Eigen::MatrixXd A = CreateRandomMatrix(rows, cols);
    int rank = 0;
    // ensure maximal rank
    while ((rank = GetMatrixRank(A)) != std::min(rows, cols)){
        A = CreateRandomMatrix(rows, cols);
    }

    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<int> reduce(0, 10);
    // try reducing until the rank is 0
    while (rank > 0){
        int reduce_by = reduce(gen);
        A = algebra::ReduceMatrixRank(A, reduce_by);
        ASSERT_EQ(A.rows(), rows);
        ASSERT_EQ(A.cols(), cols);
        auto newRank = GetMatrixRank(A);
        auto expRank = std::max(0, rank - reduce_by);
        ASSERT_EQ(expRank, newRank)
            << "Unexpected rank after reduction: exp " << expRank << " actual: " << newRank;
        rank = expRank;
    }
    ASSERT_TRUE(A.isZero());
}

TEST(ReduceMatrixRank, Rect) {
    Eigen::Index rows = 100;
    Eigen::Index cols = 110;
    Eigen::MatrixXd A = CreateRandomMatrix(rows, cols);
    int rank = 0;
    // ensure maximal rank
    while ((rank = GetMatrixRank(A)) != std::min(rows, cols)){
        A = CreateRandomMatrix(rows, cols);
    }

    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<int> reduce(0, 10);
    // try reducing until the rank is 0
    while (rank > 0){
        int reduce_by = reduce(gen);
        A = algebra::ReduceMatrixRank(A, reduce_by);
        ASSERT_EQ(A.rows(), rows);
        ASSERT_EQ(A.cols(), cols);
        auto newRank = GetMatrixRank(A);
        auto expRank = std::max(0, rank - reduce_by);
        ASSERT_EQ(expRank, newRank)
            << "Unexpected rank after reduction: exp " << expRank << " actual: " << newRank;
        rank = expRank;
    }
    ASSERT_TRUE(A.isZero());
}

TEST(ReduceMatrixRank, ByZero) {
    Eigen::Index rows = 100;
    Eigen::Index cols = 100;
    Eigen::MatrixXd A = CreateRandomMatrix(rows, cols);
    auto B = algebra::ReduceMatrixRank(A, 0);
    EXPECT_EQ(GetMatrixRank(A), GetMatrixRank(B)) << "Reduction by 0 changed matrix rank";
    ASSERT_TRUE(A == B) << "Reduction by 0 modified the matrix";
}

TEST(ReduceMatrixRank, ZeroMatrix) {
    const Eigen::Index rows = 100;
    const Eigen::Index cols = 100;
    Eigen::MatrixXd A = Eigen::Matrix<double, rows, cols>::Zero(rows, cols);
    auto B = algebra::ReduceMatrixRank(A, 5);
    EXPECT_EQ(GetMatrixRank(B), 0) << "Reduced matrix rank is different from 0";
    ASSERT_TRUE(B.isZero()) << "Reduced matrix is not 0";
}

TEST(ReduceMatrixRank, ByBigValue) {
    const Eigen::Index rows = 100;
    const Eigen::Index cols = 100;
    Eigen::MatrixXd A = CreateRandomMatrix(rows, cols);
    auto B = algebra::ReduceMatrixRank(A, 101);
    EXPECT_EQ(GetMatrixRank(B), 0) << "Reduced matrix rank is different from original";
    ASSERT_TRUE(B.isZero()) << "Reduced matrix is different from original";
}
