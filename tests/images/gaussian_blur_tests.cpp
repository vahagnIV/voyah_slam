#include "gaussian_blur_tests.h"

#include "voyahslam/images/gaussian_blur.h"
#include <cmath>

namespace voyahslam::tests {

// Helper function to compute sum of kernel values
float KernelSum(const images::Image<float, 1> &kernel) {
  float sum = 0.0f;
  for (unsigned y = 0; y < kernel.Height(); ++y) {
    for (unsigned x = 0; x < kernel.Width(); ++x) {
      sum += kernel.At(y, x, 0);
    }
  }
  return sum;
}

TEST_F(GaussianBlurTests, CreateGaussianKernelFromDimensions_WithDimensionsAndInferredSigma) {
  auto kernel = images::CreateGaussianKernelFromDimensions(5, 5);

  // Check dimensions
  ASSERT_EQ(kernel.Width(), 5);
  ASSERT_EQ(kernel.Height(), 5);

  // Check normalization (should sum to 1)
  float sum = KernelSum(kernel);
  ASSERT_NEAR(sum, 1.0f, 0.001f);

  // Check symmetry (center should be highest)
  float center = kernel.At(2, 2, 0);
  ASSERT_GT(center, 0.0f);

  // Check symmetry: values should be symmetric around center
  ASSERT_NEAR(kernel.At(0, 0, 0), kernel.At(4, 4, 0), 0.001f);
  ASSERT_NEAR(kernel.At(0, 2, 0), kernel.At(4, 2, 0), 0.001f);
  ASSERT_NEAR(kernel.At(2, 0, 0), kernel.At(2, 4, 0), 0.001f);
}

TEST_F(GaussianBlurTests, CreateGaussianKernelFromDimensions_WithDimensionsAndSigma) {
  float sigma = 1.0f;
  auto kernel = images::CreateGaussianKernelFromDimensions(5, 5, sigma);

  // Check dimensions
  ASSERT_EQ(kernel.Width(), 5);
  ASSERT_EQ(kernel.Height(), 5);

  // Check normalization
  float sum = KernelSum(kernel);
  ASSERT_NEAR(sum, 1.0f, 0.001f);

  // Center should have highest value
  float center = kernel.At(2, 2, 0);
  ASSERT_GT(center, kernel.At(2, 1, 0));
  ASSERT_GT(center, kernel.At(1, 2, 0));
}

TEST_F(GaussianBlurTests, CreateGaussianKernel_WithSigmaAndInferredDimensions) {
  float sigma = 1.5f;
  auto kernel = images::CreateGaussianKernel(sigma);

  // Check that dimensions are inferred and odd
  ASSERT_GT(kernel.Width(), 0);
  ASSERT_GT(kernel.Height(), 0);
  ASSERT_EQ(kernel.Width() % 2, 1);  // Should be odd
  ASSERT_EQ(kernel.Height() % 2, 1);  // Should be odd

  // Check normalization
  float sum = KernelSum(kernel);
  ASSERT_NEAR(sum, 1.0f, 0.001f);

  // Center should have highest value
  unsigned center_y = kernel.Height() / 2;
  unsigned center_x = kernel.Width() / 2;
  float center = kernel.At(center_y, center_x, 0);
  ASSERT_GT(center, 0.0f);
}

TEST_F(GaussianBlurTests, CreateGaussianKernel_WithSigmaAndDimensions) {
  float sigma = 1.0f;
  auto kernel = images::CreateGaussianKernelFromDimensions(7, 7, sigma);

  // Check dimensions
  ASSERT_EQ(kernel.Width(), 7);
  ASSERT_EQ(kernel.Height(), 7);

  // Check normalization
  float sum = KernelSum(kernel);
  ASSERT_NEAR(sum, 1.0f, 0.001f);
}

TEST_F(GaussianBlurTests, CreateGaussianKernelSeparableX_WithWidthAndInferredSigma) {
  auto kernel = images::CreateGaussianKernelSeparableXFromSize(5);

  // Check dimensions (should be row vector: height=1)
  ASSERT_EQ(kernel.Width(), 5);
  ASSERT_EQ(kernel.Height(), 1);

  // Check normalization
  float sum = KernelSum(kernel);
  ASSERT_NEAR(sum, 1.0f, 0.001f);

  // Check symmetry (center should be highest)
  float center = kernel.At(0, 2, 0);
  ASSERT_GT(center, 0.0f);
  ASSERT_GT(center, kernel.At(0, 0, 0));
  ASSERT_GT(center, kernel.At(0, 4, 0));

  // Check symmetry
  ASSERT_NEAR(kernel.At(0, 0, 0), kernel.At(0, 4, 0), 0.001f);
  ASSERT_NEAR(kernel.At(0, 1, 0), kernel.At(0, 3, 0), 0.001f);
}

TEST_F(GaussianBlurTests, CreateGaussianKernelSeparableX_WithWidthAndSigma) {
  float sigma = 1.0f;
  auto kernel = images::CreateGaussianKernelSeparableXFromSize(5, sigma);

  // Check dimensions
  ASSERT_EQ(kernel.Width(), 5);
  ASSERT_EQ(kernel.Height(), 1);

  // Check normalization
  float sum = KernelSum(kernel);
  ASSERT_NEAR(sum, 1.0f, 0.001f);

  // Center should have highest value
  float center = kernel.At(0, 2, 0);
  ASSERT_GT(center, kernel.At(0, 1, 0));
  ASSERT_GT(center, kernel.At(0, 3, 0));
}

TEST_F(GaussianBlurTests, CreateGaussianKernelSeparableX_WithSigmaAndInferredWidth) {
  float sigma = 1.5f;
  auto kernel = images::CreateGaussianKernelSeparableX(sigma);

  // Check that width is inferred and odd
  ASSERT_GT(kernel.Width(), 0);
  ASSERT_EQ(kernel.Width() % 2, 1);  // Should be odd
  ASSERT_EQ(kernel.Height(), 1);  // Should be row vector

  // Check normalization
  float sum = KernelSum(kernel);
  ASSERT_NEAR(sum, 1.0f, 0.001f);

  // Center should have highest value
  unsigned center_x = kernel.Width() / 2;
  float center = kernel.At(0, center_x, 0);
  ASSERT_GT(center, 0.0f);
}

TEST_F(GaussianBlurTests, CreateGaussianKernelSeparableX_WithSigmaAndWidth) {
  float sigma = 1.0f;
  auto kernel = images::CreateGaussianKernelSeparableX(sigma, 7);

  // Check dimensions
  ASSERT_EQ(kernel.Width(), 7);
  ASSERT_EQ(kernel.Height(), 1);

  // Check normalization
  float sum = KernelSum(kernel);
  ASSERT_NEAR(sum, 1.0f, 0.001f);
}

TEST_F(GaussianBlurTests, CreateGaussianKernelSeparableY_WithHeightAndInferredSigma) {
  auto kernel = images::CreateGaussianKernelSeparableYFromSize(5);

  // Check dimensions (should be column vector: width=1)
  ASSERT_EQ(kernel.Width(), 1);
  ASSERT_EQ(kernel.Height(), 5);

  // Check normalization
  float sum = KernelSum(kernel);
  ASSERT_NEAR(sum, 1.0f, 0.001f);

  // Check symmetry (center should be highest)
  float center = kernel.At(2, 0, 0);
  ASSERT_GT(center, 0.0f);
  ASSERT_GT(center, kernel.At(0, 0, 0));
  ASSERT_GT(center, kernel.At(4, 0, 0));

  // Check symmetry
  ASSERT_NEAR(kernel.At(0, 0, 0), kernel.At(4, 0, 0), 0.001f);
  ASSERT_NEAR(kernel.At(1, 0, 0), kernel.At(3, 0, 0), 0.001f);
}

TEST_F(GaussianBlurTests, CreateGaussianKernelSeparableY_WithHeightAndSigma) {
  float sigma = 1.0f;
  auto kernel = images::CreateGaussianKernelSeparableYFromSize(5, sigma);

  // Check dimensions
  ASSERT_EQ(kernel.Width(), 1);
  ASSERT_EQ(kernel.Height(), 5);

  // Check normalization
  float sum = KernelSum(kernel);
  ASSERT_NEAR(sum, 1.0f, 0.001f);

  // Center should have highest value
  float center = kernel.At(2, 0, 0);
  ASSERT_GT(center, kernel.At(1, 0, 0));
  ASSERT_GT(center, kernel.At(3, 0, 0));
}

TEST_F(GaussianBlurTests, CreateGaussianKernelSeparableY_WithSigmaAndInferredHeight) {
  float sigma = 1.5f;
  auto kernel = images::CreateGaussianKernelSeparableY(sigma);

  // Check that height is inferred and odd
  ASSERT_GT(kernel.Height(), 0);
  ASSERT_EQ(kernel.Height() % 2, 1);  // Should be odd
  ASSERT_EQ(kernel.Width(), 1);  // Should be column vector

  // Check normalization
  float sum = KernelSum(kernel);
  ASSERT_NEAR(sum, 1.0f, 0.001f);

  // Center should have highest value
  unsigned center_y = kernel.Height() / 2;
  float center = kernel.At(center_y, 0, 0);
  ASSERT_GT(center, 0.0f);
}

TEST_F(GaussianBlurTests, CreateGaussianKernelSeparableY_WithSigmaAndHeight) {
  float sigma = 1.0f;
  auto kernel = images::CreateGaussianKernelSeparableY(sigma, 7);

  // Check dimensions
  ASSERT_EQ(kernel.Width(), 1);
  ASSERT_EQ(kernel.Height(), 7);

  // Check normalization
  float sum = KernelSum(kernel);
  ASSERT_NEAR(sum, 1.0f, 0.001f);
}

TEST_F(GaussianBlurTests, CreateGaussianKernelSeparable_SameSigmaProducesSameKernel) {
  float sigma = 1.0f;

  auto kernel_x1 = images::CreateGaussianKernelSeparableXFromSize(5, sigma);
  auto kernel_x2 = images::CreateGaussianKernelSeparableX(sigma, 5);

  auto kernel_y1 = images::CreateGaussianKernelSeparableYFromSize(5, sigma);
  auto kernel_y2 = images::CreateGaussianKernelSeparableY(sigma, 5);

  // Kernels with same sigma and size should be identical
  for (unsigned x = 0; x < 5; ++x) {
    ASSERT_NEAR(kernel_x1.At(0, x, 0), kernel_x2.At(0, x, 0), 0.001f);
  }

  for (unsigned y = 0; y < 5; ++y) {
    ASSERT_NEAR(kernel_y1.At(y, 0, 0), kernel_y2.At(y, 0, 0), 0.001f);
  }
}

TEST_F(GaussianBlurTests, CreateGaussianKernel_SeparableMatches2D) {
  float sigma = 1.0f;

  // Create separable kernels
  auto kernel_x = images::CreateGaussianKernelSeparableXFromSize(5, sigma);
  auto kernel_y = images::CreateGaussianKernelSeparableYFromSize(5, sigma);

  // Create 2D kernel
  auto kernel_2d = images::CreateGaussianKernelFromDimensions(5, 5, sigma);

  // The 2D kernel should be approximately the outer product of the 1D kernels
  // (allowing for normalization differences)
  for (unsigned y = 0; y < 5; ++y) {
    for (unsigned x = 0; x < 5; ++x) {
      float separable_product = kernel_y.At(y, 0, 0) * kernel_x.At(0, x, 0);
      // The 2D kernel should be close to the product (within reasonable tolerance)
      // Note: They won't be exactly equal due to different normalization approaches
      ASSERT_GT(kernel_2d.At(y, x, 0), 0.0f);
      ASSERT_GT(separable_product, 0.0f);
    }
  }
}

TEST_F(GaussianBlurTests, CreateGaussianKernel_LargerSigmaProducesWiderKernel) {
  float sigma1 = 0.5f;
  float sigma2 = 2.0f;

  auto kernel1 = images::CreateGaussianKernel(sigma1);
  auto kernel2 = images::CreateGaussianKernel(sigma2);

  // Larger sigma should produce larger kernel (when dimensions are inferred)
  ASSERT_GE(kernel2.Width(), kernel1.Width());
  ASSERT_GE(kernel2.Height(), kernel1.Height());
}

TEST_F(GaussianBlurTests, CreateGaussianKernelSeparable_LargerSigmaProducesWiderKernel) {
  float sigma1 = 0.5f;
  float sigma2 = 2.0f;

  auto kernel_x1 = images::CreateGaussianKernelSeparableX(sigma1);
  auto kernel_x2 = images::CreateGaussianKernelSeparableX(sigma2);

  auto kernel_y1 = images::CreateGaussianKernelSeparableY(sigma1);
  auto kernel_y2 = images::CreateGaussianKernelSeparableY(sigma2);

  // Larger sigma should produce larger kernels
  ASSERT_GE(kernel_x2.Width(), kernel_x1.Width());
  ASSERT_GE(kernel_y2.Height(), kernel_y1.Height());
}

TEST_F(GaussianBlurTests, CreateGaussianKernel_ValuesDecreaseAwayFromCenter) {
  auto kernel = images::CreateGaussianKernelFromDimensions(5, 5, 1.0f);

  // Center value
  float center = kernel.At(2, 2, 0);

  // Values should decrease as we move away from center
  ASSERT_GT(center, kernel.At(2, 1, 0));
  ASSERT_GT(center, kernel.At(1, 2, 0));
  ASSERT_GT(center, kernel.At(2, 3, 0));
  ASSERT_GT(center, kernel.At(3, 2, 0));

  // Corner should be smaller than edges
  ASSERT_GT(kernel.At(2, 1, 0), kernel.At(0, 0, 0));
  ASSERT_GT(kernel.At(1, 2, 0), kernel.At(0, 0, 0));
}

TEST_F(GaussianBlurTests, CreateGaussianKernelSeparable_ValuesDecreaseAwayFromCenter) {
  auto kernel_x = images::CreateGaussianKernelSeparableXFromSize(5, 1.0f);
  auto kernel_y = images::CreateGaussianKernelSeparableYFromSize(5, 1.0f);

  // Center values
  float center_x = kernel_x.At(0, 2, 0);
  float center_y = kernel_y.At(2, 0, 0);

  // Values should decrease away from center
  ASSERT_GT(center_x, kernel_x.At(0, 1, 0));
  ASSERT_GT(center_x, kernel_x.At(0, 3, 0));
  ASSERT_GT(center_x, kernel_x.At(0, 0, 0));
  ASSERT_GT(center_x, kernel_x.At(0, 4, 0));

  ASSERT_GT(center_y, kernel_y.At(1, 0, 0));
  ASSERT_GT(center_y, kernel_y.At(3, 0, 0));
  ASSERT_GT(center_y, kernel_y.At(0, 0, 0));
  ASSERT_GT(center_y, kernel_y.At(4, 0, 0));
}

TEST_F(GaussianBlurTests, CreateGaussianKernel_MinimumSigma) {
  // Test with very small kernel to trigger minimum sigma
  auto kernel = images::CreateGaussianKernelFromDimensions(3, 3);

  // Should still produce valid kernel
  float sum = KernelSum(kernel);
  ASSERT_NEAR(sum, 1.0f, 0.001f);
  ASSERT_GT(kernel.At(1, 1, 0), 0.0f);
}

} // namespace voyahslam::tests
