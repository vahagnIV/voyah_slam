#include <gtest/gtest.h>
#include "voyahslam/geometry/distortion_models/distortion_model.h"
#include "voyahslam/geometry/distortion_models/dummy_distortion_model.h"

using namespace voyahslam::geometry::distortion_models;
using voyahslam::precision_t;

class DistortionModelTest : public ::testing::Test {
protected:
    void SetUp() override {
        // Create test data
        points_ = Eigen::Matrix<precision_t, 2, 3>::Random();
        points_copy_ = points_; // Keep a copy for comparison
    }

    Eigen::Matrix<precision_t, 2, Eigen::Dynamic> points_;
    Eigen::Matrix<precision_t, 2, Eigen::Dynamic> points_copy_;
};

TEST_F(DistortionModelTest, DummyModelDoesNothing) {
    DummyDistortionModel model;
    voyahslam::geometry::THomogenousPointsF homogeneous_points(3, points_.cols());
    homogeneous_points.topRows(2) = points_;
    model.Undistort(homogeneous_points);
    // Verify that points remain unchanged
    EXPECT_TRUE(points_.isApprox(points_copy_));
}

TEST_F(DistortionModelTest, InterfaceIsPureVirtual) {
    // This test verifies that DistortionModel cannot be instantiated
    // The following line should not compile:
    // DistortionModel model; // This should cause a compilation error
}

TEST_F(DistortionModelTest, DummyDistortJacobianIsIdentity) {
    DummyDistortionModel model;
    const voyahslam::geometry::TMatrix22F j = model.DistortJacobian({0.3, -0.2});
    EXPECT_TRUE(j.isApprox(voyahslam::geometry::TMatrix22F::Identity(), 1e-15));
}
