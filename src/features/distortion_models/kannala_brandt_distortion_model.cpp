#include "voyahslam/features/distortion_models/kannala_brandt_distortion_model.h"
#include <algorithm>
#include <cassert>
#include <cmath>

namespace voyahslam::features::distortion_models {

KannalaBrandtDistortionModel::KannalaBrandtDistortionModel(
    const std::vector<precision_t> &coefficients)
    : coefficients_(5, 0.0) {
  SetCoefficients(coefficients);
}

KannalaBrandtDistortionModel::KannalaBrandtDistortionModel(precision_t k1,
                                                           precision_t k2,
                                                           precision_t k3,
                                                           precision_t k4)
    : coefficients_{k1, k2, k3, k4} {}

KannalaBrandtDistortionModel::KannalaBrandtDistortionModel()
    : coefficients_{0.0, 0.0, 0.0, 0.0} {}

void KannalaBrandtDistortionModel::SetCoefficients(
    const std::vector<precision_t> &coefficients) {
  coefficients_.assign(4, 0.0);
  size_t count = std::min(coefficients.size(), coefficients_.size());
  for (size_t i = 0; i < count; ++i) {
    coefficients_[i] = coefficients[i];
  }
  initial_guess_lookup_.clear();
}

void KannalaBrandtDistortionModel::PrecomputeInitialGuessLookup(
    geometry::TMatrix33F &K, size_t image_width, size_t image_height) {
  // Sample theta uniformly along the image diagonal direction until the
  // distorted point leaves the image. Rays at or beyond 90 degrees have no
  // pinhole (z = 1) representation, so stop at pi/2 for fisheye lenses whose
  // image extends that far; also stop if the polynomial stops increasing.
  std::map<precision_t, precision_t> initial_guess_lookup_map;
  const precision_t theta_step = 1e-3;
  const precision_t diagonal = 1.0 / std::sqrt(2.0);

  for (precision_t theta = 0.0; theta < M_PI_2; theta += theta_step) {
    precision_t theta_d = ComputeRadialDistance(theta);
    if (!initial_guess_lookup_map.empty() &&
        theta_d <= initial_guess_lookup_map.rbegin()->first) {
      break;
    }
    initial_guess_lookup_map[theta_d] = theta;
    precision_t px = K(0, 0) * theta_d * diagonal + K(0, 2);
    precision_t py = K(1, 1) * theta_d * diagonal + K(1, 2);
    if (px >= image_width || py >= image_height) {
      break;
    }
  }

  ComputeInitialGuess(initial_guess_lookup_map);
}

precision_t KannalaBrandtDistortionModel::ComputeSmallestStepInPrecomputedMap(
    const std::map<precision_t, precision_t> &precomputed_lookup) {
  auto prev = precomputed_lookup.begin();
  decltype(prev) it;
  precision_t min_step = std::numeric_limits<precision_t>::max();
  while ((it = std::next(prev)) != precomputed_lookup.end()) {
    min_step = std::min(min_step, it->first - prev->first);
    prev = it;
  }
  return min_step;
}

void KannalaBrandtDistortionModel::ComputeInitialGuess(
    const std::map<precision_t, precision_t> &precomputed_lookup) {
  precision_t min_step =
      ComputeSmallestStepInPrecomputedMap(precomputed_lookup);
  precomputed_step_inv_ = 1.0 / min_step;

  initial_guess_lookup_.clear();
  for (auto it : precomputed_lookup) {
    double theta_d = it.first;
    double theta = it.second;
    size_t distance_from_0 =
        static_cast<size_t>(theta_d * precomputed_step_inv_);
    for (size_t i = initial_guess_lookup_.size(); i <= distance_from_0; ++i) {
      initial_guess_lookup_.push_back(theta);
    }
  }
}

precision_t
KannalaBrandtDistortionModel::ComputeRadialDistance(precision_t theta) const {
  const precision_t theta2 = theta * theta;
  const precision_t theta4 = theta2 * theta2;
  const precision_t theta6 = theta4 * theta2;
  const precision_t theta8 = theta4 * theta4;

  return theta * (1 + coefficients_[0] * theta2 + coefficients_[1] * theta4 +
                  coefficients_[2] * theta6 + coefficients_[3] * theta8);
}

precision_t KannalaBrandtDistortionModel::ComputeRadialDistanceDerivative(
    precision_t theta) const {
  const precision_t theta2 = theta * theta;
  const precision_t theta4 = theta2 * theta2;
  const precision_t theta6 = theta4 * theta2;
  const precision_t theta8 = theta6 * theta2;

  return 1 + 3.0 * coefficients_[0] * theta2 + 5.0 * coefficients_[1] * theta4 +
         7.0 * coefficients_[2] * theta6 + 9.0 * coefficients_[3] * theta8;
}

precision_t
KannalaBrandtDistortionModel::GetBestGuess(precision_t r_target) const {
  if (initial_guess_lookup_.empty()) {
    return r_target;
  }

  size_t order =
      static_cast<size_t>(std::round(r_target * precomputed_step_inv_));
  if (order >= initial_guess_lookup_.size()) {
    return initial_guess_lookup_.back();
  }
  return initial_guess_lookup_[order];
}

precision_t KannalaBrandtDistortionModel::SolveForTheta(
    precision_t r_target, int max_iterations, precision_t tolerance) const {
  if (std::abs(r_target) < tolerance) {
    return 0.0;
  }

  precision_t theta = GetBestGuess(r_target);

  for (int i = 0; i < max_iterations; ++i) {
    const precision_t r_current = ComputeRadialDistance(theta);
    const precision_t error = r_current - r_target;

    if (std::abs(error) < tolerance) {
      break;
    }

    const precision_t derivative = ComputeRadialDistanceDerivative(theta);

    if (std::abs(derivative) < tolerance) {
      break;
    }

    theta -= error / derivative;
    theta = std::max(0.0, std::min(M_PI, theta));
  }

  return theta;
}

void KannalaBrandtDistortionModel::Undistort(
    geometry::THomogenousPointsF &points) const {
  const int num_points = points.cols();

  for (int i = 0; i < num_points; ++i) {
    const precision_t x = points(0, i);
    const precision_t y = points(1, i);
    const precision_t z = points(2, i);

    if (std::abs(z) < 1e-8) {
      continue;
    }

    const precision_t x_norm = x / z;
    const precision_t y_norm = y / z;

    const precision_t r_distorted =
        std::sqrt(x_norm * x_norm + y_norm * y_norm);

    if (r_distorted < 1e-8) {
      continue;
    }

    const precision_t theta = SolveForTheta(r_distorted, 20, 1e-8);
    const precision_t r_undistorted = std::tan(theta);

    if (r_distorted > 1e-8 && std::isfinite(r_undistorted)) {
      const precision_t scale = r_undistorted / r_distorted;
      points(0, i) = x_norm * scale * z;
      points(1, i) = y_norm * scale * z;
    }
  }
}

void KannalaBrandtDistortionModel::Distort(
    geometry::THomogenousPointsF &points) const {
  const int num_points = points.cols();

  for (int i = 0; i < num_points; ++i) {
    if (points.col(i).z() != 1.) {
      points.col(i) = points.col(i) / points.col(i).z();
    }
    const precision_t x = points(0, i);
    const precision_t y = points(1, i);

    const precision_t r = std::sqrt(x * x + y * y);

    if (r < 1e-8) {
      continue;
    }

    const precision_t theta = std::atan(r);
    const precision_t theta_d = ComputeRadialDistance(theta);

    if (std::isfinite(theta_d)) {
      const precision_t scale = theta_d / r;
      points(0, i) = x * scale;
      points(1, i) = y * scale;
      points(2, i) = 1;
    }
  }
}

} // namespace voyahslam::features::distortion_models
