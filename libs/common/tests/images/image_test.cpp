#include <voyahslam/images/image.h>

#include <gtest/gtest.h>

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <new>

namespace voyahslam::images {
namespace {

// A parent whose every (y, x, c) holds a distinct value (mod 251, a prime
// below 256, so uint8 values do not repeat along a row).
Image<uint8_t, 3> MakeParent(unsigned width, unsigned height) {
  Image<uint8_t, 3> parent(width, height);
  for (unsigned y = 0; y < height; ++y) {
    for (unsigned x = 0; x < width; ++x) {
      for (unsigned c = 0; c < 3; ++c) {
        parent.Set(y, x, c, static_cast<uint8_t>((y * width * 3 + x * 3 + c) % 251));
      }
    }
  }
  return parent;
}

TEST(ImageTest, DefaultIsEmpty) {
  // Default-initialise on a buffer full of garbage, so members without an
  // initialiser show up as non-zero instead of being zeroed by luck.
  alignas(Image<uint8_t, 3>) unsigned char storage[sizeof(Image<uint8_t, 3>)];
  std::memset(storage, 0xAB, sizeof(storage));
  auto *img = new (storage) Image<uint8_t, 3>;  // default-init, not value-init
  EXPECT_EQ(img->Width(), 0u);
  EXPECT_EQ(img->Height(), 0u);
  EXPECT_EQ(img->Stride(), 0u);
  EXPECT_EQ(img->Offset(), 0u);
  EXPECT_EQ(img->GetData(), nullptr);
  img->~Image();
}

TEST(ImageTest, ConvertOfViewReadsOnlyTheView) {
  auto parent = MakeParent(8, 6);
  Image<uint8_t, 3> view(parent, 2, 1, 4, 3);
  Image<float, 3> converted = view.Convert<float>();

  ASSERT_EQ(converted.Width(), 4u);
  ASSERT_EQ(converted.Height(), 3u);
  EXPECT_EQ(converted.Stride(), 4u);
  for (unsigned y = 0; y < 3; ++y) {
    for (unsigned x = 0; x < 4; ++x) {
      for (unsigned c = 0; c < 3; ++c) {
        EXPECT_FLOAT_EQ(converted.At(y, x, c), static_cast<float>(view.At(y, x, c)))
            << "y=" << y << " x=" << x << " c=" << c;
        EXPECT_FLOAT_EQ(converted.At(y, x, c), static_cast<float>(parent.At(y + 1, x + 2, c)));
      }
    }
  }
}

TEST(ImageTest, ConvertOfBorderViewIncludesLastRowAndColumn) {
  auto parent = MakeParent(8, 6);
  Image<uint8_t, 3> view(parent, 5, 4, 3, 2);  // touches the right and bottom borders
  Image<float, 3> converted = view.Convert<float>();

  ASSERT_EQ(converted.Width(), 3u);
  ASSERT_EQ(converted.Height(), 2u);
  for (unsigned y = 0; y < 2; ++y) {
    for (unsigned x = 0; x < 3; ++x) {
      for (unsigned c = 0; c < 3; ++c) {
        EXPECT_FLOAT_EQ(converted.At(y, x, c), static_cast<float>(parent.At(y + 4, x + 5, c)));
      }
    }
  }
}

TEST(ImageTest, ConvertOfEmptyIsEmpty) {
  Image<uint8_t, 3> empty;
  Image<float, 3> converted = empty.Convert<float>();
  EXPECT_EQ(converted.Width(), 0u);
  EXPECT_EQ(converted.Height(), 0u);
}

}  // namespace
}  // namespace voyahslam::images
