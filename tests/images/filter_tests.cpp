#include "filter_tests.h"

#include "voyahslam/images/filter.h"
#include "voyahslam/images/gaussian_blur.h"
#include <cmath>

namespace voyahslam::tests {

TEST_F(FilterTests, Apply2DKernel_SimpleBoxFilter) {
  // Create a simple 3x3 box filter (all ones)
  images::Image<float, 1> kernel(3, 3);
  for (unsigned y = 0; y < 3; ++y) {
    for (unsigned x = 0; x < 3; ++x) {
      kernel.Set(y, x, 0, 1.0f);
    }
  }

  // Create a simple test image
  images::Image<float, 1> image(5, 5);
  for (unsigned y = 0; y < 5; ++y) {
    for (unsigned x = 0; x < 5; ++x) {
      image.Set(y, x, 0, static_cast<float>(y * 5 + x));
    }
  }

  auto result = images::Filter<float>::Apply(kernel, image);

  // Check center pixel (should be average of 3x3 neighborhood)
  // Center is at (2,2), which is value 12
  // Neighborhood: 6,7,8, 11,12,13, 16,17,18
  // Average = (6+7+8+11+12+13+16+17+18) / 9 = 12.0
  ASSERT_NEAR(result.At(2, 2, 0), 12.0f, 0.001f);
}

TEST_F(FilterTests, Apply2DKernel_EdgeClamping) {
  // Create a simple 3x3 box filter
  images::Image<float, 1> kernel(3, 3);
  for (unsigned y = 0; y < 3; ++y) {
    for (unsigned x = 0; x < 3; ++x) {
      kernel.Set(y, x, 0, 1.0f);
    }
  }

  // Create a test image with constant value
  images::Image<float, 1> image(3, 3);
  for (unsigned y = 0; y < 3; ++y) {
    for (unsigned x = 0; x < 3; ++x) {
      image.Set(y, x, 0, 10.0f);
    }
  }

  auto result = images::Filter<float>::Apply(kernel, image);

  // All pixels should be 10.0 (edge clamping replicates the value)
  for (unsigned y = 0; y < 3; ++y) {
    for (unsigned x = 0; x < 3; ++x) {
      ASSERT_NEAR(result.At(y, x, 0), 10.0f, 0.001f);
    }
  }
}

TEST_F(FilterTests, Apply2DKernel_IdentityKernel) {
  // Create an identity-like kernel (only center is 1, rest are 0)
  images::Image<float, 1> kernel(3, 3);
  for (unsigned y = 0; y < 3; ++y) {
    for (unsigned x = 0; x < 3; ++x) {
      kernel.Set(y, x, 0, 0.0f);
    }
  }
  kernel.Set(1, 1, 0, 1.0f);

  // Create a test image
  images::Image<float, 1> image(3, 3);
  for (unsigned y = 0; y < 3; ++y) {
    for (unsigned x = 0; x < 3; ++x) {
      image.Set(y, x, 0, static_cast<float>(y * 3 + x));
    }
  }

  auto result = images::Filter<float>::Apply(kernel, image);

  // Result should be close to original (with edge clamping effects)
  // Center pixel should be exactly the same
  ASSERT_NEAR(result.At(1, 1, 0), 4.0f, 0.001f);
}

TEST_F(FilterTests, ApplySeparable_HorizontalKernel) {
  // Create a simple horizontal averaging kernel [1, 1, 1]
  images::Image<float, 1> kernel_x(3, 1);
  kernel_x.Set(0, 0, 0, 1.0f);
  kernel_x.Set(0, 1, 0, 1.0f);
  kernel_x.Set(0, 2, 0, 1.0f);

  // Create a vertical kernel [1] (no vertical filtering)
  images::Image<float, 1> kernel_y(1, 1);
  kernel_y.Set(0, 0, 0, 1.0f);

  // Create a test image
  images::Image<float, 1> image(3, 3);
  for (unsigned y = 0; y < 3; ++y) {
    for (unsigned x = 0; x < 3; ++x) {
      image.Set(y, x, 0, static_cast<float>(x)); // Values: 0, 1, 2 per row
    }
  }

  auto result = images::Filter<float>::ApplySeparable(kernel_x, kernel_y, image);

  // Each pixel should be average of its horizontal neighbors
  // Row 0: [0,1,2] -> with clamping: [0,0,1], [0,1,2], [1,2,2]
  // Averages: (0+0+1)/3=0.33, (0+1+2)/3=1.0, (1+2+2)/3=1.67
  ASSERT_NEAR(result.At(0, 0, 0), 0.666, 0.1f);
  ASSERT_NEAR(result.At(0, 1, 0), 1.0f, 0.1f);
  ASSERT_NEAR(result.At(0, 2, 0), 1.667f, 0.1f);
}

TEST_F(FilterTests, ApplySeparable_VerticalKernel) {
  // Create a horizontal kernel [1] (no horizontal filtering)
  images::Image<float, 1> kernel_x(1, 1);
  kernel_x.Set(0, 0, 0, 1.0f);

  // Create a simple vertical averaging kernel [1, 1, 1]^T
  images::Image<float, 1> kernel_y(1, 3);
  kernel_y.Set(0, 0, 0, 1.0f);
  kernel_y.Set(1, 0, 0, 1.0f);
  kernel_y.Set(2, 0, 0, 1.0f);

  // Create a test image
  images::Image<float, 1> image(3, 3);
  for (unsigned y = 0; y < 3; ++y) {
    for (unsigned x = 0; x < 3; ++x) {
      image.Set(y, x, 0, static_cast<float>(y)); // Values: 0, 1, 2 per column
    }
  }

  auto result = images::Filter<float>::ApplySeparable(kernel_x, kernel_y, image);

  // Each pixel should be average of its vertical neighbors
  // Column 0: [0,1,2]^T -> with clamping: [0,0,1], [0,1,2], [1,2,2]
  // Averages: (0+0+1)/3=0.33, (0+1+2)/3=1.0, (1+2+2)/3=1.67
  ASSERT_NEAR(result.At(0, 0, 0), 0.666, 0.1f);
  ASSERT_NEAR(result.At(1, 0, 0), 1.0f, 0.1f);
  ASSERT_NEAR(result.At(2, 0, 0), 1.667f, 0.1f);
}

TEST_F(FilterTests, ApplySeparable_BothDirections) {
  // Create horizontal kernel [1, 1, 1]
  images::Image<float, 1> kernel_x(3, 1);
  kernel_x.Set(0, 0, 0, 1.0f);
  kernel_x.Set(0, 1, 0, 1.0f);
  kernel_x.Set(0, 2, 0, 1.0f);

  // Create vertical kernel [1, 1, 1]^T
  images::Image<float, 1> kernel_y(1, 3);
  kernel_y.Set(0, 0, 0, 1.0f);
  kernel_y.Set(1, 0, 0, 1.0f);
  kernel_y.Set(2, 0, 0, 1.0f);

  // Create a constant test image
  images::Image<float, 1> image(3, 3);
  for (unsigned y = 0; y < 3; ++y) {
    for (unsigned x = 0; x < 3; ++x) {
      image.Set(y, x, 0, 5.0f);
    }
  }

  auto result = images::Filter<float>::ApplySeparable(kernel_x, kernel_y, image);

  // All pixels should remain 5.0 (averaging constant values)
  for (unsigned y = 0; y < 3; ++y) {
    for (unsigned x = 0; x < 3; ++x) {
      ASSERT_NEAR(result.At(y, x, 0), 5.0f, 0.001f);
    }
  }
}

TEST_F(FilterTests, ApplySeparable_WithGaussianKernel) {
  // Create small Gaussian kernels
  auto kernel_x = images::CreateGaussianKernelFromDimensions(3, 1, 0.5f); // Row vector
  auto kernel_y = images::CreateGaussianKernelFromDimensions(1, 3, 0.5f); // Column vector

  // Create a test image with a single bright pixel
  images::Image<float, 1> image(5, 5);
  for (unsigned y = 0; y < 5; ++y) {
    for (unsigned x = 0; x < 5; ++x) {
      image.Set(y, x, 0, 0.0f);
    }
  }
  image.Set(2, 2, 0, 1.0f); // Center pixel is bright

  auto result = images::Filter<float>::ApplySeparable(kernel_x, kernel_y, image);

  // Center should have highest value
  float center = result.At(2, 2, 0);
  ASSERT_GT(center, 0.0f);

  // Values should decrease away from center
  ASSERT_GT(center, result.At(2, 1, 0));
  ASSERT_GT(center, result.At(1, 2, 0));
}

TEST_F(FilterTests, Apply2DKernel_WithGaussianKernel) {
  // Create a 2D Gaussian kernel
  auto kernel = images::CreateGaussianKernelFromDimensions(5, 5, 1.0f);

  // Create a test image with a single bright pixel
  images::Image<float, 1> image(5, 5);
  for (unsigned y = 0; y < 5; ++y) {
    for (unsigned x = 0; x < 5; ++x) {
      image.Set(y, x, 0, 0.0f);
    }
  }
  image.Set(2, 2, 0, 1.0f); // Center pixel is bright

  auto result = images::Filter<float>::Apply(kernel, image);

  // Center should have highest value
  float center = result.At(2, 2, 0);
  ASSERT_GT(center, 0.0f);

  // Values should decrease away from center
  ASSERT_GT(center, result.At(2, 1, 0));
  ASSERT_GT(center, result.At(1, 2, 0));
}

TEST_F(FilterTests, Apply2DKernel_DifferentTypes) {
  // Test with uint8_t image and float kernel
  images::Image<float, 1> kernel(3, 3);
  for (unsigned y = 0; y < 3; ++y) {
    for (unsigned x = 0; x < 3; ++x) {
      kernel.Set(y, x, 0, 1.0f);
    }
  }

  images::Image<uint8_t, 1> image(3, 3);
  for (unsigned y = 0; y < 3; ++y) {
    for (unsigned x = 0; x < 3; ++x) {
      image.Set(y, x, 0, 100);
    }
  }

  auto result = images::Filter<uint8_t, 1, float>::Apply(kernel, image);

  // Result type should be float (from uint8_t * float)
  // All pixels should be 100 (averaging constant values)
  for (unsigned y = 0; y < 3; ++y) {
    for (unsigned x = 0; x < 3; ++x) {
      ASSERT_NEAR(result.At(y, x, 0), 100.0f, 0.1f);
    }
  }
}

TEST_F(FilterTests, Apply2DKernel_Normalization) {
  // Create a kernel that doesn't sum to 1
  images::Image<float, 1> kernel(3, 3);
  for (unsigned y = 0; y < 3; ++y) {
    for (unsigned x = 0; x < 3; ++x) {
      kernel.Set(y, x, 0, 2.0f); // Sum = 18
    }
  }

  // Create a constant test image
  images::Image<float, 1> image(3, 3);
  for (unsigned y = 0; y < 3; ++y) {
    for (unsigned x = 0; x < 3; ++x) {
      image.Set(y, x, 0, 10.0f);
    }
  }

  auto result = images::Filter<float>::Apply(kernel, image);

  // Result should be normalized (kernel sum is used for normalization)
  // So result should be 10.0 (the original value)
  for (unsigned y = 0; y < 3; ++y) {
    for (unsigned x = 0; x < 3; ++x) {
      ASSERT_NEAR(result.At(y, x, 0), 10.0f, 0.001f);
    }
  }
}

TEST_F(FilterTests, ApplySeparable_Matches2DForSeparableKernel) {
  // Create separable kernels
  images::Image<float, 1> kernel_x(3, 1);
  kernel_x.Set(0, 0, 0, 0.25f);
  kernel_x.Set(0, 1, 0, 0.5f);
  kernel_x.Set(0, 2, 0, 0.25f);

  images::Image<float, 1> kernel_y(1, 3);
  kernel_y.Set(0, 0, 0, 0.25f);
  kernel_y.Set(1, 0, 0, 0.5f);
  kernel_y.Set(2, 0, 0, 0.25f);

  // Create the equivalent 2D kernel (outer product)
  images::Image<float, 1> kernel_2d(3, 3);
  for (unsigned y = 0; y < 3; ++y) {
    for (unsigned x = 0; x < 3; ++x) {
      kernel_2d.Set(y, x, 0, kernel_y.At(y, 0, 0) * kernel_x.At(0, x, 0));
    }
  }

  // Create a test image
  images::Image<float, 1> image(5, 5);
  for (unsigned y = 0; y < 5; ++y) {
    for (unsigned x = 0; x < 5; ++x) {
      image.Set(y, x, 0, static_cast<float>(y * 5 + x));
    }
  }

  auto result_separable = images::Filter<float>::ApplySeparable(kernel_x, kernel_y, image);
  auto result_2d = images::Filter<float>::Apply(kernel_2d, image);

  // Results should be very similar (allowing for floating point differences)
  for (unsigned y = 0; y < 5; ++y) {
    for (unsigned x = 0; x < 5; ++x) {
      ASSERT_NEAR(result_separable.At(y, x, 0), result_2d.At(y, x, 0), 0.01f);
    }
  }
}

} // namespace voyahslam::tests
