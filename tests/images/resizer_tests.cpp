#include "resizer_tests.h"

#include "voyahslam/images/resizer.h"

namespace voyahslam::tests {

TEST_F(ResizerTests, ResizesImage) {

  images::Image<uint8_t, 1> image(3, 3);
  image.Set(0, 0, 0, 1);
  image.Set(1, 0, 0, 1);
  image.Set(2, 0, 0, 1);
  image.Set(0, 1, 0, 2);
  image.Set(1, 1, 0, 2);
  image.Set(2, 1, 0, 2);
  image.Set(0, 2, 0, 1);
  image.Set(1, 2, 0, 1);
  image.Set(2, 2, 0, 1);

  images::Resizer<uint8_t, 1> resizer;
  auto resized = resizer.Resize(image, 2, 2);
  ASSERT_DOUBLE_EQ(resized.At(0, 0, 0), 1);
  ASSERT_DOUBLE_EQ(resized.At(1, 0, 0), 1);
  ASSERT_DOUBLE_EQ(resized.At(0, 1, 0), 1);
  ASSERT_DOUBLE_EQ(resized.At(1, 1, 0), 1);
}

} // namespace voyahslam::tests
