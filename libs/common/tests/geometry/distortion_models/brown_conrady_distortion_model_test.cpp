#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <vector>

#include <opencv2/calib3d.hpp>

#include "voyahslam/defs.h"
#include "voyahslam/geometry/distortion_models/brown_conrady_distortion_model.h"

using namespace voyahslam::geometry::distortion_models;
using voyahslam::precision_t;
using voyahslam::geometry::THomogenousPointsF;
using voyahslam::geometry::TMatrix22F;
using voyahslam::geometry::TPoint2DF;

namespace {

// EuRoC MH_01 cam0 (radial-tangential, 752x480).
constexpr precision_t kK1 = -0.28340811;
constexpr precision_t kK2 = 0.07395907;
constexpr precision_t kP1 = 0.00019359;
constexpr precision_t kP2 = 1.76187114e-05;
constexpr precision_t kK3 = 0.05;  // EuRoC has k3 = 0; a nonzero value exercises the r^6 term.

constexpr precision_t kFu = 458.654880721;
constexpr precision_t kFv = 457.296696463;
constexpr precision_t kCu = 367.215803962;
constexpr precision_t kCv = 248.375340610;
constexpr int kWidth = 752;
constexpr int kHeight = 480;

BrownConradyDistortionModel EurocModel(precision_t k3 = 0.0) {
  return BrownConradyDistortionModel(kK1, kK2, kP1, kP2, k3);
}

// {x, y, x_d, y_d}: undistorted -> distorted normalized points (z = 1).
using Reference = std::array<precision_t, 4>;

// Produced once with OpenCV 5.1.0 cv::projectPoints (K = I, rvec = tvec = 0,
// dist = [k1, k2, p1, p2, k3] above, object points (x, y, 1)), printed with
// "%.17g" by a throwaway program. Kept as fixed values next to the live
// OpenCV comparisons below, so a change of OpenCV's model cannot go unnoticed.
constexpr std::array<Reference, 8> kReferenceK3Zero = {{
    {0, 0, 0, 0},
    {0.1, 0, 0.099717860042042, 1.9359000000000004e-06},
    {0, -0.2, 7.0474845600000023e-07, -0.19773317122240003},
    {0.3, 0.2, 0.28935074879543399, 0.192924138941968},
    {-0.5, 0.4, -0.44817900025612595, 0.35862835104224},
    {0.7, -0.45, 0.58735269825619763, -0.37744197290264137},
    {-0.75, -0.5, -0.61373726094392811, -0.40901042555629374},
    {0.8, 0.5, 0.64527291376133811, 0.40345806579262006},
}};

constexpr std::array<Reference, 8> kReferenceK3NonZero = {{
    {0, 0, 0, 0},
    {0.1, 0, 0.099717865042042012, 1.9359000000000004e-06},
    {0, -0.2, 7.0474845600000023e-07, -0.19773381122240002},
    {0.3, 0.2, 0.28938370379543399, 0.19294610894196798},
    {-0.5, 0.4, -0.44990202525612594, 0.36000677104223999},
    {0.7, -0.45, 0.59897594286557265, -0.38491405872295387},
    {-0.75, -0.5, -0.63385139668611556, -0.42241984938441873},
    {0.8, 0.5, 0.6734716737613381, 0.42108229079262005},
}};

void ExpectMatchesReference(const BrownConradyDistortionModel& model,
                            const std::array<Reference, 8>& references) {
  for (const Reference& ref : references) {
    THomogenousPointsF point(3, 1);
    point << ref[0], ref[1], 1.0;
    model.Distort(point);
    EXPECT_NEAR(point(0, 0), ref[2], 1e-12) << "at (" << ref[0] << ", " << ref[1] << ")";
    EXPECT_NEAR(point(1, 0), ref[3], 1e-12) << "at (" << ref[0] << ", " << ref[1] << ")";
    EXPECT_EQ(point(2, 0), 1.0);
  }
}

cv::Mat OpenCvCoefficients(const BrownConradyDistortionModel& model) {
  const auto& c = model.GetCoefficients();
  // OpenCV 5's Mat_ comma initializer does not accept doubles; copy instead.
  return cv::Mat(1, 5, CV_64F, const_cast<precision_t*>(c.data())).clone();
}

cv::Mat EurocCameraMatrix() {
  precision_t k[9] = {kFu, 0, kCu, 0, kFv, kCv, 0, 0, 1};
  return cv::Mat(3, 3, CV_64F, k).clone();
}

// Pixel of a distorted normalized point.
TPoint2DF ToPixel(precision_t x_d, precision_t y_d) {
  return {kFu * x_d + kCu, kFv * y_d + kCv};
}

}  // namespace

TEST(BrownConradyDistortionModelTest, DistortMatchesOpenCvWithK3Zero) {
  ExpectMatchesReference(EurocModel(), kReferenceK3Zero);
}

TEST(BrownConradyDistortionModelTest, DistortMatchesOpenCvWithK3NonZero) {
  ExpectMatchesReference(EurocModel(kK3), kReferenceK3NonZero);
}

// Live comparison with cv::projectPoints (K = I, no pose) over a grid that
// covers the EuRoC image, for EuRoC, nonzero-k3 and strong-tangential models.
TEST(BrownConradyDistortionModelTest, DistortMatchesOpenCvProjectPoints) {
  for (const auto& model : {EurocModel(), EurocModel(kK3), BrownConradyDistortionModel(kK1, kK2, 0.01, -0.02, kK3)}) {
    std::vector<cv::Point3d> object_points;
    THomogenousPointsF points(3, 0);
    for (precision_t x = -1.3; x <= 1.3; x += 0.1) {
      for (precision_t y = -0.9; y <= 0.9; y += 0.1) {
        object_points.emplace_back(x, y, 1.0);
        points.conservativeResize(3, points.cols() + 1);
        points.col(points.cols() - 1) << x, y, 1.0;
      }
    }
    std::vector<cv::Point2d> image_points;
    const cv::Mat zero = cv::Mat::zeros(3, 1, CV_64F);
    cv::projectPoints(object_points, zero, zero, cv::Mat::eye(3, 3, CV_64F), OpenCvCoefficients(model),
                      image_points);
    model.Distort(points);
    for (size_t i = 0; i < image_points.size(); ++i) {
      EXPECT_NEAR(points(0, i), image_points[i].x, 1e-12) << "at " << object_points[i];
      EXPECT_NEAR(points(1, i), image_points[i].y, 1e-12) << "at " << object_points[i];
    }
  }
}

// Live comparison with cv::undistortPoints on raw EuRoC pixels. OpenCV's
// fixed-point iteration stops after 5 iterations by default, far from 1e-10;
// it is run for 200 here. Newton (to 1e-12) is the stricter of the two.
TEST(BrownConradyDistortionModelTest, UndistortMatchesOpenCvUndistortPoints) {
  for (const auto& model : {EurocModel(), EurocModel(kK3)}) {
    std::vector<cv::Point2d> pixels;
    for (int u = 0; u <= kWidth; u += 16) {
      for (int v = 0; v <= kHeight; v += 16) {
        pixels.emplace_back(u, v);
      }
    }
    std::vector<cv::Point2d> reference;
    cv::undistortPoints(pixels, reference, EurocCameraMatrix(), OpenCvCoefficients(model), cv::noArray(),
                        cv::noArray(), cv::TermCriteria(cv::TermCriteria::COUNT, 200, 0));
    for (size_t i = 0; i < pixels.size(); ++i) {
      THomogenousPointsF point(3, 1);
      point << (pixels[i].x - kCu) / kFu, (pixels[i].y - kCv) / kFv, 1.0;
      model.Undistort(point);
      EXPECT_NEAR(point(0, 0), reference[i].x, 1e-10) << "pixel " << pixels[i];
      EXPECT_NEAR(point(1, 0), reference[i].y, 1e-10) << "pixel " << pixels[i];
    }
  }
}

TEST(BrownConradyDistortionModelTest, DistortNormalizesHomogeneousPoints) {
  const auto model = EurocModel();
  THomogenousPointsF point(3, 1);
  point << 2 * 0.3, 2 * 0.2, 2.0;
  model.Distort(point);
  EXPECT_NEAR(point(0, 0), 0.28935074879543399, 1e-12);
  EXPECT_NEAR(point(1, 0), 0.192924138941968, 1e-12);
  EXPECT_EQ(point(2, 0), 1.0);
}

TEST(BrownConradyDistortionModelTest, KeepsCoefficients) {
  const auto model = EurocModel(kK3);
  const auto& c = model.GetCoefficients();
  EXPECT_EQ(c[0], kK1);
  EXPECT_EQ(c[1], kK2);
  EXPECT_EQ(c[2], kP1);
  EXPECT_EQ(c[3], kP2);
  EXPECT_EQ(c[4], kK3);
}

// A grid of undistorted points whose distorted image covers the whole EuRoC
// image (checked below), so the round trip is exercised over the full field of view.
TEST(BrownConradyDistortionModelTest, UndistortInvertsDistortOverEurocFieldOfView) {
  constexpr precision_t kHalfWidth = 1.3;
  constexpr precision_t kHalfHeight = 0.9;
  constexpr int kSteps = 40;
  for (precision_t k3 : {0.0, kK3}) {
    const auto model = EurocModel(k3);

    // The box boundary distorts outside the image on every side.
    THomogenousPointsF boundary(3, 4);
    boundary << -kHalfWidth, kHalfWidth, 0.0, 0.0,
                0.0, 0.0, -kHalfHeight, kHalfHeight,
                1.0, 1.0, 1.0, 1.0;
    model.Distort(boundary);
    EXPECT_LT(ToPixel(boundary(0, 0), boundary(1, 0)).x(), 0.0);
    EXPECT_GT(ToPixel(boundary(0, 1), boundary(1, 1)).x(), kWidth);
    EXPECT_LT(ToPixel(boundary(0, 2), boundary(1, 2)).y(), 0.0);
    EXPECT_GT(ToPixel(boundary(0, 3), boundary(1, 3)).y(), kHeight);
    for (precision_t sx : {-1.0, 1.0}) {
      for (precision_t sy : {-1.0, 1.0}) {
        THomogenousPointsF corner(3, 1);
        corner << sx * kHalfWidth, sy * kHalfHeight, 1.0;
        model.Distort(corner);
        const TPoint2DF pixel = ToPixel(corner(0, 0), corner(1, 0));
        EXPECT_TRUE(pixel.x() < 0 || pixel.x() > kWidth) << "corner " << sx << ", " << sy;
        EXPECT_TRUE(pixel.y() < 0 || pixel.y() > kHeight) << "corner " << sx << ", " << sy;
      }
    }

    for (int i = 0; i <= kSteps; ++i) {
      for (int j = 0; j <= kSteps; ++j) {
        const precision_t x = -kHalfWidth + 2 * kHalfWidth * i / kSteps;
        const precision_t y = -kHalfHeight + 2 * kHalfHeight * j / kSteps;
        THomogenousPointsF point(3, 1);
        point << x, y, 1.0;
        model.Distort(point);
        model.Undistort(point);
        EXPECT_NEAR(point(0, 0), x, 1e-10) << "k3=" << k3 << " at (" << x << ", " << y << ")";
        EXPECT_NEAR(point(1, 0), y, 1e-10) << "k3=" << k3 << " at (" << x << ", " << y << ")";
        EXPECT_EQ(point(2, 0), 1.0);
      }
    }
  }
}

TEST(BrownConradyDistortionModelTest, UndistortKeepsHomogeneousScale) {
  const auto model = EurocModel();
  THomogenousPointsF point(3, 1);
  point << 3 * 0.28935074879543399, 3 * 0.192924138941968, 3.0;
  model.Undistort(point);
  EXPECT_NEAR(point(0, 0), 3 * 0.3, 1e-10);
  EXPECT_NEAR(point(1, 0), 3 * 0.2, 1e-10);
  EXPECT_EQ(point(2, 0), 3.0);
}

TEST(BrownConradyDistortionModelTest, DistortJacobianMatchesFiniteDifferences) {
  for (precision_t k3 : {0.0, kK3}) {
    // Exaggerated tangential terms so their derivatives are visible.
    for (const auto& model : {EurocModel(k3), BrownConradyDistortionModel(kK1, kK2, 0.01, -0.02, k3)}) {
      for (const TPoint2DF p : {TPoint2DF(0, 0), TPoint2DF(0.1, 0), TPoint2DF(0, -0.2), TPoint2DF(0.3, 0.2),
                                TPoint2DF(-0.5, 0.4), TPoint2DF(0.7, -0.45), TPoint2DF(-1.2, -0.8)}) {
        const TMatrix22F analytic = model.DistortJacobian(p);
        TMatrix22F numeric;
        const precision_t h = 1e-6;
        for (int c = 0; c < 2; ++c) {
          THomogenousPointsF plus(3, 1), minus(3, 1);
          plus << p.x(), p.y(), 1.0;
          minus = plus;
          plus(c, 0) += h;
          minus(c, 0) -= h;
          model.Distort(plus);
          model.Distort(minus);
          numeric.col(c) = (plus.col(0).head<2>() - minus.col(0).head<2>()) / (2 * h);
        }
        EXPECT_LT((analytic - numeric).norm(), 1e-8) << "k3=" << k3 << " at " << p.transpose();
      }
    }
  }
}

TEST(BrownConradyDistortionModelTest, ZeroCoefficientsAreIdentity) {
  const BrownConradyDistortionModel model(0, 0, 0, 0);
  THomogenousPointsF points(3, 4);
  points << 0.0, 0.3, -0.7, 1.5,
            0.0, -0.2, 0.4, 2.0,
            1.0, 1.0, 1.0, 1.0;
  const THomogenousPointsF original = points;
  model.Distort(points);
  EXPECT_TRUE(points.isApprox(original, 1e-15));
  model.Undistort(points);
  EXPECT_TRUE(points.isApprox(original, 1e-15));
  EXPECT_TRUE(model.DistortJacobian({0.3, -0.2}).isApprox(TMatrix22F::Identity(), 1e-15));
}

// k1 = -0.5 folds the image at r = sqrt(2/3): no undistorted point within the
// invertible region distorts to radius 1. Newton cycles there (1 -> 0 -> 1),
// so Undistort leaves the point exactly as it was, homogeneous scale included.
TEST(BrownConradyDistortionModelTest, UndistortLeavesNonConvergingPointUnchanged) {
  const BrownConradyDistortionModel model(-0.5, 0, 0, 0);
  THomogenousPointsF points(3, 3);
  points << 1.0, 2.0, 0.1,
            0.0, 0.0, 0.0,
            1.0, 2.0, 1.0;
  model.Undistort(points);
  EXPECT_EQ(points(0, 0), 1.0);
  EXPECT_EQ(points(1, 0), 0.0);
  EXPECT_EQ(points(2, 0), 1.0);
  EXPECT_EQ(points(0, 1), 2.0);
  EXPECT_EQ(points(1, 1), 0.0);
  EXPECT_EQ(points(2, 1), 2.0);
  // A point inside the invertible region in the same batch is still undistorted.
  THomogenousPointsF check = points.col(2);
  model.Distort(check);
  EXPECT_NEAR(check(0, 0), 0.1, 1e-12);
}

// The same fold also has a mirrored preimage (x < 0 for a point at x > 0).
// Newton must never return it: a solution on the folded branch counts as a
// failure and the point is left unchanged.
TEST(BrownConradyDistortionModelTest, UndistortRejectsSolutionsOnTheFoldedBranch) {
  const BrownConradyDistortionModel model(-0.5, 0, 0, 0);
  for (precision_t x_d = 0.55; x_d < 3.0; x_d += 0.01) {
    THomogenousPointsF point(3, 1);
    point << x_d, 0.0, 1.0;
    model.Undistort(point);
    const bool unchanged = point(0, 0) == x_d;
    const bool on_main_branch = point(0, 0) > 0 && point(0, 0) * point(0, 0) < 2.0 / 3.0;
    EXPECT_TRUE(unchanged || on_main_branch) << "x_d=" << x_d << " -> " << point(0, 0);
  }
}

TEST(BrownConradyDistortionModelTest, NearZeroDepthIsLeftUnchanged) {
  const auto model = EurocModel();
  THomogenousPointsF point(3, 1);
  point << 0.3, 0.2, 1e-12;
  const THomogenousPointsF original = point;
  model.Undistort(point);
  EXPECT_EQ(point, original);
  model.Distort(point);
  EXPECT_EQ(point, original);
}
