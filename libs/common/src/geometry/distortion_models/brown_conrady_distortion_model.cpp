#include "voyahslam/geometry/distortion_models/brown_conrady_distortion_model.h"

#include <cmath>

namespace voyahslam::geometry::distortion_models {

BrownConradyDistortionModel::BrownConradyDistortionModel(precision_t k1, precision_t k2,
                                                         precision_t p1, precision_t p2,
                                                         precision_t k3)
    : coefficients_{k1, k2, p1, p2, k3} {}

namespace {

constexpr precision_t kMinDepth = 1e-8;

} // namespace

geometry::TPoint2DF BrownConradyDistortionModel::DistortPoint(const geometry::TPoint2DF &p) const {
  const auto &[k1, k2, p1, p2, k3] = coefficients_;
  const precision_t x = p.x();
  const precision_t y = p.y();
  const precision_t r2 = x * x + y * y;
  const precision_t radial = 1 + r2 * (k1 + r2 * (k2 + r2 * k3));
  return {x * radial + 2 * p1 * x * y + p2 * (r2 + 2 * x * x),
          y * radial + p1 * (r2 + 2 * y * y) + 2 * p2 * x * y};
}

void BrownConradyDistortionModel::Distort(geometry::THomogenousPointsF &points) const {
  for (Eigen::Index i = 0; i < points.cols(); ++i) {
    const precision_t z = points(2, i);
    if (std::abs(z) < kMinDepth) {
      continue;
    }
    const geometry::TPoint2DF distorted = DistortPoint(points.col(i).head<2>() / z);
    points.col(i) << distorted, 1.0;
  }
}

void BrownConradyDistortionModel::Undistort(geometry::THomogenousPointsF &points) const {
  for (Eigen::Index i = 0; i < points.cols(); ++i) {
    const precision_t z = points(2, i);
    if (std::abs(z) < kMinDepth) {
      continue;
    }
    const geometry::TPoint2DF distorted = points.col(i).head<2>() / z;
    geometry::TPoint2DF undistorted = distorted;
    bool converged = false;
    for (int iteration = 0; iteration <= kMaxUndistortIterations; ++iteration) {
      const geometry::TPoint2DF residual = DistortPoint(undistorted) - distorted;
      if (!residual.allFinite()) {
        break;
      }
      if (residual.lpNorm<Eigen::Infinity>() < kUndistortTolerance) {
        converged = true;
        break;
      }
      if (iteration == kMaxUndistortIterations) {
        break;
      }
      const geometry::TMatrix22F jacobian = DistortJacobian(undistorted);
      const precision_t determinant = jacobian.determinant();
      if (!std::isfinite(determinant) || determinant == 0.0) {
        break;
      }
      undistorted -= jacobian.inverse() * residual;
    }
    if (!converged) {
      continue;
    }
    // A root where the map is folded (not orientation-preserving with a
    // positive trace) is a spurious preimage, e.g. the mirrored point beyond
    // the turning radius of k1 < 0.
    const geometry::TMatrix22F jacobian = DistortJacobian(undistorted);
    if (jacobian.determinant() <= 0.0 || jacobian.trace() <= 0.0) {
      continue;
    }
    points.col(i) << undistorted * z, z;
  }
}

geometry::TMatrix22F BrownConradyDistortionModel::DistortJacobian(
    const geometry::TPoint2DF &undistorted) const {
  const auto &[k1, k2, p1, p2, k3] = coefficients_;
  const precision_t x = undistorted.x();
  const precision_t y = undistorted.y();
  const precision_t r2 = x * x + y * y;
  const precision_t radial = 1 + r2 * (k1 + r2 * (k2 + r2 * k3));
  // d(radial)/dx = 2 x radial_slope, d(radial)/dy = 2 y radial_slope.
  const precision_t radial_slope = k1 + r2 * (2 * k2 + r2 * 3 * k3);
  // The Jacobian is symmetric: both off-diagonal terms are equal.
  const precision_t off_diagonal = 2 * x * y * radial_slope + 2 * p1 * x + 2 * p2 * y;
  geometry::TMatrix22F jacobian;
  jacobian << radial + 2 * x * x * radial_slope + 2 * p1 * y + 6 * p2 * x, off_diagonal,
              off_diagonal, radial + 2 * y * y * radial_slope + 6 * p1 * y + 2 * p2 * x;
  return jacobian;
}

} // namespace voyahslam::geometry::distortion_models
