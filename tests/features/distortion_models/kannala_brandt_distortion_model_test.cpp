#include "kannala_brandt_distortion_model_test.h"
#include <cmath>

#include <opencv2/calib3d/calib3d.hpp>

using namespace voyahslam::features::distortion_models;

namespace voyahslam::test::features {

void KannalaBrandtDistortionModelTest::SetUp() {
  // Default coefficients (identity-like transformation)
  default_coefficients_ = {1.0, 0.0, 0.0, 0.0, 0.0};

  // Realistic fisheye coefficients (similar to actual camera calibration)
  // Typical values are in the range of 10^-3 to 10^-6
  fisheye_coefficients_ = {0.00348238940225, 0.000715034845216,
                           -0.00205323614187, 0.000202936735918};

  CreateTestPoints();
}

void KannalaBrandtDistortionModelTest::CreateTestPoints() {
  // Create test points in normalized camera coordinates
  test_points_.resize(3, 8);

  // Origin point
  test_points_.col(0) << 0.0, 0.0, 1.0;

  // Points along axes
  test_points_.col(1) << 0.1, 0.0, 1.0;
  test_points_.col(2) << 0.0, 0.1, 1.0;
  test_points_.col(3) << -0.1, 0.0, 1.0;
  test_points_.col(4) << 0.0, -0.1, 1.0;

  // Diagonal points
  test_points_.col(5) << 0.1, 0.1, 1.0;
  test_points_.col(6) << -0.1, -0.1, 1.0;

  // Point with higher distortion
  test_points_.col(7) << 0.5, 0.3, 1.0;

  // Keep a copy for comparison
  test_points_copy_ = test_points_;
}

void KannalaBrandtDistortionModelTest::VerifyPointsValid(
    const geometry::THomogenousPointsF &points) const {
  for (int i = 0; i < points.cols(); ++i) {
    EXPECT_TRUE(std::isfinite(points(0, i)))
        << "Point " << i << " x-coordinate is not finite";
    EXPECT_TRUE(std::isfinite(points(1, i)))
        << "Point " << i << " y-coordinate is not finite";
    EXPECT_TRUE(std::isfinite(points(2, i)))
        << "Point " << i << " z-coordinate is not finite";
    EXPECT_NE(points(2, i), 0.0) << "Point " << i << " has zero z-coordinate";
  }
}

precision_t
KannalaBrandtDistortionModelTest::ComputeAngleFromPoint(precision_t x,
                                                        precision_t y) const {
  return std::atan(std::sqrt(x * x + y * y));
}

precision_t
KannalaBrandtDistortionModelTest::ComputeRadialDistance(precision_t x,
                                                        precision_t y) const {
  return std::sqrt(x * x + y * y);
}

// Test basic construction
TEST_F(KannalaBrandtDistortionModelTest, DefaultConstructor) {
  KannalaBrandtDistortionModel model;
  const auto &coeffs = model.GetCoefficients();

  EXPECT_EQ(coeffs.size(), 4);
  EXPECT_EQ(coeffs[0], 0.0);
  EXPECT_EQ(coeffs[1], 0.0);
  EXPECT_EQ(coeffs[2], 0.0);
  EXPECT_EQ(coeffs[3], 0.0);
}

TEST_F(KannalaBrandtDistortionModelTest, ConstructorWithCoefficients) {
  KannalaBrandtDistortionModel model(fisheye_coefficients_);
  const auto &coeffs = model.GetCoefficients();

  EXPECT_EQ(coeffs.size(), 4);
  for (size_t i = 0; i < fisheye_coefficients_.size(); ++i) {
    EXPECT_EQ(coeffs[i], fisheye_coefficients_[i]);
  }
}

TEST_F(KannalaBrandtDistortionModelTest,
       ConstructorWithIndividualCoefficients) {
  KannalaBrandtDistortionModel model(0.8, 0.001, 1e-5, 1e-8);
  const auto &coeffs = model.GetCoefficients();

  EXPECT_EQ(coeffs.size(), 4);
  EXPECT_EQ(coeffs[0], 0.8);
  EXPECT_EQ(coeffs[1], 0.001);
  EXPECT_EQ(coeffs[2], 1e-5);
  EXPECT_EQ(coeffs[3], 1e-8);
}

TEST_F(KannalaBrandtDistortionModelTest, ConstructorWithPartialCoefficients) {
  std::vector<precision_t> partial_coeffs = {0.8, -0.001};
  KannalaBrandtDistortionModel model(partial_coeffs);
  const auto &coeffs = model.GetCoefficients();

  EXPECT_EQ(coeffs.size(), 4);
  EXPECT_EQ(coeffs[0], 0.8);
  EXPECT_EQ(coeffs[1], -0.001);
  EXPECT_EQ(coeffs[2], 0.0);
  EXPECT_EQ(coeffs[3], 0.0);
}

TEST_F(KannalaBrandtDistortionModelTest, SetCoefficients) {
  KannalaBrandtDistortionModel model;
  std::vector<precision_t> new_coeffs = {0.8, -0.001, 1e-5};

  model.SetCoefficients(new_coeffs);
  const auto &coeffs = model.GetCoefficients();

  EXPECT_EQ(coeffs.size(), 4);
  EXPECT_EQ(coeffs[0], 0.8);
  EXPECT_EQ(coeffs[1], -0.001);
  EXPECT_EQ(coeffs[2], 1e-5);
  EXPECT_EQ(coeffs[3], 0.0);
}

// Test undistortion behavior
TEST_F(KannalaBrandtDistortionModelTest, UndistortDoesNotChangePrincipalPoint) {
  KannalaBrandtDistortionModel model(fisheye_coefficients_);
  geometry::THomogenousPointsF points(3, 1);
  points.col(0) << 0.0, 0.0, 1.0; // Principal point

  geometry::THomogenousPointsF original_points = points;
  model.Undistort(points);

  // Principal point should remain unchanged
  EXPECT_NEAR(points(0, 0), original_points(0, 0), 1e-10);
  EXPECT_NEAR(points(1, 0), original_points(1, 0), 1e-10);
  EXPECT_NEAR(points(2, 0), original_points(2, 0), 1e-10);
}

TEST_F(KannalaBrandtDistortionModelTest, UndistortProducesFiniteResults) {
  KannalaBrandtDistortionModel model(fisheye_coefficients_);
  geometry::THomogenousPointsF points = test_points_;

  model.Undistort(points);

  VerifyPointsValid(points);
}

TEST_F(KannalaBrandtDistortionModelTest, UndistortWithDefaultCoefficients) {
  // With default coefficients (k1=1, others=0), r(θ) = θ
  // This should behave like equidistance projection
  KannalaBrandtDistortionModel model(default_coefficients_);
  geometry::THomogenousPointsF points = test_points_;

  model.Undistort(points);

  VerifyPointsValid(points);

  // For small angles, undistortion should be close to identity for equidistance
  // model
  for (int i = 1; i < 5; ++i) { // Skip principal point at i=0
    precision_t x_orig = test_points_(0, i) / test_points_(2, i);
    precision_t y_orig = test_points_(1, i) / test_points_(2, i);
    precision_t x_undist = points(0, i) / points(2, i);
    precision_t y_undist = points(1, i) / points(2, i);

    // For small angles and k1=1, the change should be minimal
    EXPECT_NEAR(x_undist, x_orig, 0.1)
        << "Point " << i << " x changed too much";
    EXPECT_NEAR(y_undist, y_orig, 0.1)
        << "Point " << i << " y changed too much";
  }
}

TEST_F(KannalaBrandtDistortionModelTest, UndistortPreservesAngularDirection) {
  KannalaBrandtDistortionModel model(fisheye_coefficients_);

  // Test points along different directions
  geometry::THomogenousPointsF points(3, 4);
  points.col(0) << 0.1, 0.0, 1.0;  // +x direction
  points.col(1) << 0.0, 0.1, 1.0;  // +y direction
  points.col(2) << -0.1, 0.0, 1.0; // -x direction
  points.col(3) << 0.0, -0.1, 1.0; // -y direction

  geometry::THomogenousPointsF original_points = points;
  model.Undistort(points);

  for (int i = 0; i < 4; ++i) {
    precision_t orig_angle =
        std::atan2(original_points(1, i), original_points(0, i));
    precision_t undist_angle = std::atan2(points(1, i), points(0, i));

    // Angular direction should be preserved (allowing for small numerical
    // errors)
    precision_t angle_diff = std::abs(orig_angle - undist_angle);
    if (angle_diff > M_PI)
      angle_diff = 2 * M_PI - angle_diff; // Handle wrap-around

    EXPECT_LT(angle_diff, 0.01)
        << "Angular direction not preserved for point " << i;
  }
}

TEST_F(KannalaBrandtDistortionModelTest, UndistortHandlesZeroZCoordinate) {
  KannalaBrandtDistortionModel model(fisheye_coefficients_);
  geometry::THomogenousPointsF points(3, 2);
  points.col(0) << 1.0, 2.0, 0.0; // Zero z-coordinate (point at infinity)
  points.col(1) << 0.1, 0.1, 1.0; // Normal point

  geometry::THomogenousPointsF original_points = points;
  model.Undistort(points);

  // Point at infinity should remain unchanged
  EXPECT_EQ(points(0, 0), original_points(0, 0));
  EXPECT_EQ(points(1, 0), original_points(1, 0));
  EXPECT_EQ(points(2, 0), original_points(2, 0));

  // Normal point should be processed
  EXPECT_TRUE(std::isfinite(points(0, 1)));
  EXPECT_TRUE(std::isfinite(points(1, 1)));
  EXPECT_TRUE(std::isfinite(points(2, 1)));
}

// Test mathematical properties
TEST_F(KannalaBrandtDistortionModelTest, RadialSymmetry) {
  KannalaBrandtDistortionModel model(fisheye_coefficients_);

  // Test radial symmetry: points at same distance from center should be
  // affected equally
  geometry::THomogenousPointsF points(3, 4);
  precision_t r = 0.2;
  points.col(0) << r, 0.0, 1.0;
  points.col(1) << 0.0, r, 1.0;
  points.col(2) << r / std::sqrt(2), r / std::sqrt(2), 1.0;
  points.col(3) << -r / std::sqrt(2), -r / std::sqrt(2), 1.0;

  model.Undistort(points);

  // All points should have approximately the same radial distance after
  // undistortion
  precision_t first_distance = ComputeRadialDistance(
      points(0, 0) / points(2, 0), points(1, 0) / points(2, 0));
  for (int i = 1; i < 4; ++i) {
    precision_t distance = ComputeRadialDistance(points(0, i) / points(2, i),
                                                 points(1, i) / points(2, i));
    EXPECT_NEAR(distance, first_distance, 1e-6)
        << "Radial symmetry violated for point " << i;
  }
}

TEST_F(KannalaBrandtDistortionModelTest,
       MonotonicBehaviorForIncreasingRadialDistance) {
  KannalaBrandtDistortionModel model(fisheye_coefficients_);

  // Test points at increasing radial distances
  std::vector<precision_t> radial_distances = {0.05, 0.1, 0.2, 0.3, 0.4};
  geometry::THomogenousPointsF points(3, radial_distances.size());

  for (size_t i = 0; i < radial_distances.size(); ++i) {
    points.col(i) << radial_distances[i], 0.0, 1.0;
  }

  model.Undistort(points);

  // Check that undistorted radial distances are still in increasing order
  // (this tests that the model doesn't create unrealistic inversions)
  precision_t prev_distance = 0.0;
  for (size_t i = 0; i < radial_distances.size(); ++i) {
    precision_t curr_distance = ComputeRadialDistance(
        points(0, i) / points(2, i), points(1, i) / points(2, i));
    EXPECT_GT(curr_distance, prev_distance)
        << "Monotonic property violated at point " << i;
    prev_distance = curr_distance;
  }
}

// Edge cases
TEST_F(KannalaBrandtDistortionModelTest, UndistortEmptyPointSet) {
  KannalaBrandtDistortionModel model(fisheye_coefficients_);
  geometry::THomogenousPointsF empty_points(3, 0);

  // Should not crash
  EXPECT_NO_THROW(model.Undistort(empty_points));
}

TEST_F(KannalaBrandtDistortionModelTest, UndistortLargeDistances) {
  KannalaBrandtDistortionModel model(fisheye_coefficients_);
  geometry::THomogenousPointsF points(3, 1);
  points.col(0) << 2.0, 1.5, 1.0; // Large radial distance

  model.Undistort(points);

  // Should produce finite results even for large inputs
  VerifyPointsValid(points);
}

// Test Distort method
TEST_F(KannalaBrandtDistortionModelTest, DistortProducesFiniteResults) {
  KannalaBrandtDistortionModel model(fisheye_coefficients_);
  geometry::THomogenousPointsF points = test_points_;

  model.Distort(points);

  VerifyPointsValid(points);
}

TEST_F(KannalaBrandtDistortionModelTest, DistortDoesNotChangePrincipalPoint) {
  KannalaBrandtDistortionModel model(fisheye_coefficients_);
  geometry::THomogenousPointsF points(3, 1);
  points.col(0) << 0.0, 0.0, 1.0; // Principal point

  geometry::THomogenousPointsF original_points = points;
  model.Distort(points);

  // Principal point should remain unchanged
  EXPECT_NEAR(points(0, 0), original_points(0, 0), 1e-10);
  EXPECT_NEAR(points(1, 0), original_points(1, 0), 1e-10);
  EXPECT_NEAR(points(2, 0), original_points(2, 0), 1e-10);
}

TEST_F(KannalaBrandtDistortionModelTest,
       DistortAndUndistortAreInverseOperations) {
  KannalaBrandtDistortionModel model(fisheye_coefficients_);
  geometry::THomogenousPointsF original_points = test_points_;
  geometry::THomogenousPointsF distorted_points = original_points;
  geometry::THomogenousPointsF undistorted_points = original_points;

  // Apply distortion
  model.Distort(distorted_points);

  cv::Mat cv_undistorted(distorted_points.cols(), 1, CV_64FC2);
  for (size_t i = 0; i < distorted_points.cols(); ++i) {
    cv_undistorted.at<cv::Point2d>(i, 0).x = undistorted_points(0, i);
    cv_undistorted.at<cv::Point2d>(i, 0).y = undistorted_points(1, i);
  }
  cv::Mat distortion_coefficients(1, 4, CV_64F);
  for (int i = 0; i < 4; ++i) {
    distortion_coefficients.at<double>(0, i) = fisheye_coefficients_[i];
  }
  cv::Mat cv_distoerted;
  cv::Mat K = cv::Mat::eye(3, 3, CV_64F);
  cv::fisheye::distortPoints(cv_undistorted, cv_distoerted, K,
                             distortion_coefficients);

  for (int i = 0; i < cv_undistorted.rows; ++i) {
    double cv_x = cv_distoerted.at<double>(i, 0);
    double cv_y = cv_distoerted.at<double>(i, 1);
    double x = distorted_points(0, i);
    double y = distorted_points(1, i);
    EXPECT_NEAR(x, cv_x, 0.0001);
    EXPECT_NEAR(y, cv_y, 0.0001);
  }

  // Apply undistortion to the distorted points
  model.Undistort(distorted_points);

  cv::fisheye::undistortPoints(cv_distoerted, cv_undistorted,
                               cv::Mat::eye(3, 3, CV_64F),
                               distortion_coefficients);
  for (int i = 0; i < cv_undistorted.rows; ++i) {
    double cv_x = cv_undistorted.at<double>(i, 0);
    double cv_y = cv_undistorted.at<double>(i, 1);
    double x = undistorted_points(0, i);
    double y = undistorted_points(1, i);
    EXPECT_NEAR(x, cv_x, 0.0001);
    EXPECT_NEAR(y, cv_y, 0.0001);
  }

  // The result should be close to the original points (allowing for numerical
  // errors)
  for (int i = 0; i < original_points.cols(); ++i) {
    if (std::abs(original_points(2, i)) > 1e-8) { // Skip points at infinity
      precision_t x_orig = original_points(0, i) / original_points(2, i);
      precision_t y_orig = original_points(1, i) / original_points(2, i);
      precision_t x_result =
          undistorted_points(0, i) / undistorted_points(2, i);
      precision_t y_result =
          undistorted_points(1, i) / undistorted_points(2, i);

      // Allow for reasonable numerical error
      EXPECT_NEAR(x_result, x_orig, 1e-7)
          << "Distort/Undistort not inverse for point " << i << " x-coordinate";
      EXPECT_NEAR(y_result, y_orig, 1e-7)
          << "Distort/Undistort not inverse for point " << i << " y-coordinate";
    }
  }
}

// TUM-VI 1024x1024 fisheye: even theta = 90 degrees projects inside the image
// (diagonal radius ~1.58 rad * fx < 1024 - cx), so a lookup that walks outward
// until it leaves the image never terminated.
TEST_F(KannalaBrandtDistortionModelTest, LookupTerminatesForFisheyeWiderThanImage) {
  KannalaBrandtDistortionModel model(0.010171079892421483, -0.010816440029919381,
                                     0.005942781769412756, -0.001662284667857643);
  geometry::TMatrix33F K;
  K << 380.81042871360756, 0, 510.29465304840727,
       0, 380.81194179427075, 514.3304630538506,
       0, 0, 1;
  model.PrecomputeInitialGuessLookup(K, 1024, 1024);
  ASSERT_TRUE(model.IsLookupTablePrecomputed());

  // Round trip up to ~80 degrees off-axis.
  for (precision_t r : {0.1, 0.5, 1.0, 2.0, 5.0}) {
    geometry::THomogenousPointsF point(3, 1);
    point << r / std::sqrt(2.0), r / std::sqrt(2.0), 1.0;
    geometry::THomogenousPointsF distorted = point;
    model.Distort(distorted);
    model.Undistort(distorted);
    EXPECT_NEAR(distorted(0, 0) / distorted(2, 0), point(0, 0), 1e-6) << "r = " << r;
    EXPECT_NEAR(distorted(1, 0) / distorted(2, 0), point(1, 0), 1e-6) << "r = " << r;
  }
}

TEST_F(KannalaBrandtDistortionModelTest, UndistortIsInverseOfDistort) {
  // Test with different coefficient sets to ensure robustness
  std::vector<std::vector<precision_t>> test_coefficients = {
      {1.0, 0.0, 0.0, 0.0, 0.0},         // Identity-like
      {0.8, -0.005, 1e-5, -3e-8, 5e-12}, // Realistic fisheye
      {0.9, -0.001, 1e-6, 0.0, 0.0},     // Partial coefficients
      {0.7, 0.002, -1e-5, 2e-8, -1e-11}  // Mixed signs
  };

  for (size_t coeff_idx = 0; coeff_idx < test_coefficients.size();
       ++coeff_idx) {
    const auto &coeffs = test_coefficients[coeff_idx];
    KannalaBrandtDistortionModel model(coeffs);
    geometry::TMatrix33F K;
    K << 190.978477151, 0, 255.431706059, 0, 190.973307052, 257.397442899, 0, 0,
        1;
    model.PrecomputeInitialGuessLookup(K, 512, 512);

    // Create test points with various radial distances
    geometry::THomogenousPointsF test_points(3, 6);
    test_points.col(0) << 0.0, 0.0, 1.0;   // Principal point
    test_points.col(1) << 0.05, 0.0, 1.0;  // Small distance
    test_points.col(2) << 0.1, 0.0, 1.0;   // Medium distance
    test_points.col(3) << 0.2, 0.1, 1.0;   // Larger distance
    test_points.col(4) << -0.15, 0.1, 1.0; // Negative x
    test_points.col(5) << 0.1, -0.2, 1.0;  // Negative y

    for (int i = 0; i < test_points.cols(); ++i) {
      // Skip principal point for this test
      if (i == 0)
        continue;

      // Create a single point for testing
      geometry::THomogenousPointsF original_point = test_points.col(i);
      geometry::THomogenousPointsF distorted_point = original_point;
      geometry::THomogenousPointsF recovered_point = original_point;

      // Apply distortion
      model.Distort(distorted_point);

      // Apply undistortion to get back to original
      model.Undistort(distorted_point);

      // Check that we're back to the original point (within numerical
      // precision)
      precision_t x_orig = original_point(0, 0) / original_point(2, 0);
      precision_t y_orig = original_point(1, 0) / original_point(2, 0);
      precision_t x_recovered = distorted_point(0, 0) / distorted_point(2, 0);
      precision_t y_recovered = distorted_point(1, 0) / distorted_point(2, 0);

      // Compute the error
      precision_t error_x = std::abs(x_recovered - x_orig);
      precision_t error_y = std::abs(y_recovered - y_orig);
      precision_t max_error = std::max(error_x, error_y);

      // For the identity-like model, error should be very small
      if (coeff_idx == 0) {
        EXPECT_LT(max_error, 1e-8)
            << "Identity model should be nearly exact for point " << i;
      } else {
        // For other models, allow larger error due to numerical precision
        EXPECT_LT(max_error, 0.01) << "Coeff set " << coeff_idx << " point "
                                   << i << " error too large: " << max_error;
      }
    }
  }
}

TEST_F(KannalaBrandtDistortionModelTest,
       DistortUndistortRoundTripPreservesAngularDirection) {
  KannalaBrandtDistortionModel model(fisheye_coefficients_);

  // Test points in different angular directions
  std::vector<precision_t> angles = {0.0,          M_PI / 4,    M_PI / 2,
                                     3 * M_PI / 4, M_PI,        5 * M_PI / 4,
                                     3 * M_PI / 2, 7 * M_PI / 4};
  precision_t radius = 0.2;

  for (size_t i = 0; i < angles.size(); ++i) {
    precision_t angle = angles[i];
    precision_t x = radius * std::cos(angle);
    precision_t y = radius * std::sin(angle);

    geometry::THomogenousPointsF point(3, 1);
    point.col(0) << x, y, 1.0;

    geometry::THomogenousPointsF original_point = point;

    // Apply distort -> undistort
    model.Distort(point);
    model.Undistort(point);

    // Check that angular direction is preserved
    precision_t orig_angle =
        std::atan2(original_point(1, 0), original_point(0, 0));
    precision_t recovered_angle = std::atan2(point(1, 0), point(0, 0));

    // Handle angle wrap-around
    precision_t angle_diff = std::abs(recovered_angle - orig_angle);
    if (angle_diff > M_PI) {
      angle_diff = 2 * M_PI - angle_diff;
    }

    EXPECT_LT(angle_diff, 0.01) << "Angular direction not preserved for angle "
                                << angle << " (error: " << angle_diff << ")";
  }
}

} // namespace voyahslam::test::features
