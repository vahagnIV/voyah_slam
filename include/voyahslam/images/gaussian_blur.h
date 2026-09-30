#ifndef VOYAHSLAM_IMAGES_GAUSSIAN_BLUR_H
#define VOYAHSLAM_IMAGES_GAUSSIAN_BLUR_H

#include <voyahslam/images/image.h>
#include <voyahslam/defs.h>
#include <cmath>
#include <algorithm>

namespace voyahslam::images {

inline Image<float, 1> CreateGaussianKernelFromDimensions(unsigned width, unsigned height, float sigma = -1) {
  if (sigma < 0) {
    sigma = static_cast<float>(std::min(width, height) - 1) / 6.0f;
    if (sigma < 0.5f) {
      sigma = 0.5f;
    }
  }

  Image<float, 1> kernel(width, height);
  const float two_sigma_sq = 2.0f * sigma * sigma;
  const float pi_sigma_sq = static_cast<float>(M_PI) * two_sigma_sq;
  const float normalization = 1.0f / pi_sigma_sq;

  const int center_x = static_cast<int>(width) / 2;
  const int center_y = static_cast<int>(height) / 2;

  float sum = 0.0f;
  for (unsigned y = 0; y < height; ++y) {
    for (unsigned x = 0; x < width; ++x) {
      const int dx = static_cast<int>(x) - center_x;
      const int dy = static_cast<int>(y) - center_y;
      const float dist_sq = static_cast<float>(dx * dx + dy * dy);
      const float value = normalization * std::exp(-dist_sq / two_sigma_sq);
      kernel.Set(y, x, 0, value);
      sum += value;
    }
  }

  if (sum > 0.0f) {
    for (unsigned y = 0; y < height; ++y) {
      for (unsigned x = 0; x < width; ++x) {
        kernel.Set(y, x, 0, kernel.At(y, x, 0) / sum);
      }
    }
  }

  return kernel;
}

inline Image<float, 1> CreateGaussianKernel(float sigma, unsigned width = 0, unsigned height = 0) {
  if (width == 0) {
    width = static_cast<unsigned>(std::ceil(6.0f * sigma)) + 1;
    if (width % 2 == 0) {
      width += 1;
    }
  }

  if (height == 0) {
    height = static_cast<unsigned>(std::ceil(6.0f * sigma)) + 1;
    if (height % 2 == 0) {
      height += 1;
    }
  }

  return CreateGaussianKernelFromDimensions(width, height, sigma);
}

inline Image<float, 1> CreateGaussianKernelSeparableXFromSize(unsigned width, float sigma = -1) {
  if (sigma < 0) {
    sigma = static_cast<float>(width - 1) / 6.0f;
    if (sigma < 0.5f) {
      sigma = 0.5f;
    }
  }

  Image<float, 1> kernel(width, 1);
  const float two_sigma_sq = 2.0f * sigma * sigma;
  const float sqrt_two_pi_sigma = sigma * std::sqrt(2.0f * static_cast<float>(M_PI));
  const float normalization = 1.0f / sqrt_two_pi_sigma;

  const int center = static_cast<int>(width) / 2;

  float sum = 0.0f;
  for (unsigned x = 0; x < width; ++x) {
    const int dx = static_cast<int>(x) - center;
    const float dist_sq = static_cast<float>(dx * dx);
    const float value = normalization * std::exp(-dist_sq / two_sigma_sq);
    kernel.Set(0, x, 0, value);
    sum += value;
  }

  if (sum > 0.0f) {
    for (unsigned x = 0; x < width; ++x) {
      kernel.Set(0, x, 0, kernel.At(0, x, 0) / sum);
    }
  }

  return kernel;
}

inline Image<float, 1> CreateGaussianKernelSeparableYFromSize(unsigned height, float sigma = -1) {
  if (sigma < 0) {
    sigma = static_cast<float>(height - 1) / 6.0f;
    if (sigma < 0.5f) {
      sigma = 0.5f;
    }
  }

  Image<float, 1> kernel(1, height);
  const float two_sigma_sq = 2.0f * sigma * sigma;
  const float sqrt_two_pi_sigma = sigma * std::sqrt(2.0f * static_cast<float>(M_PI));
  const float normalization = 1.0f / sqrt_two_pi_sigma;

  const int center = static_cast<int>(height) / 2;

  float sum = 0.0f;
  for (unsigned y = 0; y < height; ++y) {
    const int dy = static_cast<int>(y) - center;
    const float dist_sq = static_cast<float>(dy * dy);
    const float value = normalization * std::exp(-dist_sq / two_sigma_sq);
    kernel.Set(y, 0, 0, value);
    sum += value;
  }

  if (sum > 0.0f) {
    for (unsigned y = 0; y < height; ++y) {
      kernel.Set(y, 0, 0, kernel.At(y, 0, 0) / sum);
    }
  }

  return kernel;
}

inline Image<float, 1> CreateGaussianKernelSeparableX(float sigma, unsigned width = 0) {
  if (width == 0) {
    width = static_cast<unsigned>(std::ceil(6.0f * sigma)) + 1;
    if (width % 2 == 0) {
      width += 1;
    }
  }

  return CreateGaussianKernelSeparableXFromSize(width, sigma);
}

inline Image<float, 1> CreateGaussianKernelSeparableY(float sigma, unsigned height = 0) {
  if (height == 0) {
    height = static_cast<unsigned>(std::ceil(6.0f * sigma)) + 1;
    if (height % 2 == 0) {
      height += 1;
    }
  }

  return CreateGaussianKernelSeparableYFromSize(height, sigma);
}

} // namespace voyahslam::images

#endif // VOYAHSLAM_IMAGES_GAUSSIAN_BLUR_H
