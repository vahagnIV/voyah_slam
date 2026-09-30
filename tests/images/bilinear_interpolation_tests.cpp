#include "bilinear_interpolation_tests.h"

#include "voyahslam/images/bilinear_interpolation.h"

namespace voyahslam::tests {

TEST_F(BilinearInterpolationTests, Interpolates) {
  images::Image<float, 1> image(3, 3);
  images::BilinearInterpolation<float, 1> interpolation;
  image.Set(0,0,0, 1);
  image.Set(1,0,0, 1);
  image.Set(2,0,0, 1);
  image.Set(0,1,0, 2);
  image.Set(1,1,0, 2);
  image.Set(2,1,0, 2);
  image.Set(0,2,0, 1);
  image.Set(1,2,0, 1);
  image.Set(2,2,0, 1);

  float result = interpolation.Sample(image, 0.5,0.5,0);
  ASSERT_DOUBLE_EQ(result, 1.5);

  result = interpolation.Sample(image, 1.7,1.4,0);
  ASSERT_NEAR(result, 1.6, 1e-6);
}

} // namespace voyahslam::tests
