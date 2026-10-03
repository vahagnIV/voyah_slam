#include <voyahviz/color_image.h>

#include <gtest/gtest.h>

namespace voyahviz {
namespace {

using voyahslam::images::TImageU8Gray;

TEST(ColorImageTest, ToColorCopiesGrayIntoAllChannels) {
    TImageU8Gray gray(3, 2);
    for (unsigned y = 0; y < 2; ++y)
        for (unsigned x = 0; x < 3; ++x) gray.Set(y, x, 0, static_cast<uint8_t>(10 * y + x));

    TColorImage color = ToColor(gray);
    ASSERT_EQ(color.Width(), 3u);
    ASSERT_EQ(color.Height(), 2u);
    for (unsigned c = 0; c < 3; ++c) EXPECT_EQ(color.At(1, 2, c), 12);
}

TEST(ColorImageTest, HorizontalConcatPlacesImagesSideBySide) {
    TColorImage left(2, 2), right(3, 2);
    for (unsigned y = 0; y < 2; ++y) {
        for (unsigned c = 0; c < 3; ++c) {
            for (unsigned x = 0; x < 2; ++x) left.Set(y, x, c, 1);
            for (unsigned x = 0; x < 3; ++x) right.Set(y, x, c, 2);
        }
    }
    TColorImage both = HorizontalConcat(left, right);
    ASSERT_EQ(both.Width(), 5u);
    ASSERT_EQ(both.Height(), 2u);
    EXPECT_EQ(both.At(1, 1, 0), 1);
    EXPECT_EQ(both.At(1, 2, 0), 2);
    EXPECT_EQ(both.At(0, 4, 2), 2);
}

} // namespace
} // namespace voyahviz
