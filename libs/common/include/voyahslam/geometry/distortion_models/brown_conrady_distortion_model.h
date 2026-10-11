#ifndef VOYAHSLAM_GEOMETRY_DISTORTION_MODELS_BROWN_CONRADY_DISTORTION_MODEL_H
#define VOYAHSLAM_GEOMETRY_DISTORTION_MODELS_BROWN_CONRADY_DISTORTION_MODEL_H

#include <voyahslam/defs.h>
#include <voyahslam/geometry/distortion_models/distortion_model.h>

#include <array>

namespace voyahslam::geometry::distortion_models {

// Brown-Conrady (radial-tangential) lens distortion, the OpenCV standard model
// with coefficients [k1, k2, p1, p2, k3]. Also known as ROS "plumb_bob",
// Kalibr "radtan" and EuRoC "radial-tangential". On the z = 1 plane, with
// r^2 = x^2 + y^2:
//   x_d = x (1 + k1 r^2 + k2 r^4 + k3 r^6) + 2 p1 x y + p2 (r^2 + 2 x^2)
//   y_d = y (1 + k1 r^2 + k2 r^4 + k3 r^6) + p1 (r^2 + 2 y^2) + 2 p2 x y
//
// Homogeneous points: Distort() maps each column to the z = 1 plane first and
// returns it with z = 1; Undistort() works on (x/z, y/z) and writes the result
// back at the input's z. Columns with |z| < 1e-8 are left unchanged by both.
//
// Undistort() inverts the polynomial with Newton's method on the analytic
// Jacobian, starting at the distorted point, until the residual is below 1e-12
// (at most 20 iterations). The inversion FAILS when Newton does not converge
// or converges onto the folded branch (where the Jacobian is no longer
// orientation-preserving with a positive trace, e.g. the mirrored preimage
// k1 < 0 creates beyond its turning radius). A failed column is left
// UNCHANGED, i.e. it stays the distorted normalized point; no error is
// reported. This matches the Kannala-Brandt model and only happens far outside
// the region a calibration is valid for (beyond the image of a real lens).
// Callers do not check for it (DESIGN.md "Camera models (lens distortion)").
class BrownConradyDistortionModel : public DistortionModel {
public:
  static constexpr int kMaxUndistortIterations = 20;
  static constexpr precision_t kUndistortTolerance = 1e-12;

  BrownConradyDistortionModel(precision_t k1, precision_t k2, precision_t p1,
                              precision_t p2, precision_t k3 = 0.0);

  void Undistort(geometry::THomogenousPointsF &points) const override;
  void Distort(geometry::THomogenousPointsF &points) const override;
  [[nodiscard]] geometry::TMatrix22F DistortJacobian(
      const geometry::TPoint2DF &undistorted) const override;

  // [k1, k2, p1, p2, k3], the OpenCV order.
  [[nodiscard]] const std::array<precision_t, 5> &GetCoefficients() const {
    return coefficients_;
  }

private:
  [[nodiscard]] geometry::TPoint2DF DistortPoint(const geometry::TPoint2DF &p) const;

  std::array<precision_t, 5> coefficients_;
};

} // namespace voyahslam::geometry::distortion_models

#endif // VOYAHSLAM_GEOMETRY_DISTORTION_MODELS_BROWN_CONRADY_DISTORTION_MODEL_H
