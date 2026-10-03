#include <voyahviz/image_conversion.h>

#include <gtest/gtest.h>

namespace voyahviz {
namespace {

using voyahslam::images::TImageF32Gray;
using voyahslam::images::TImageU8Gray;

TEST(ImageConversionTest, WrapsEightBitImageWithoutCopying) {
    TImageU8Gray image(4, 3);
    for (unsigned y = 0; y < 3; ++y)
        for (unsigned x = 0; x < 4; ++x) image.Set(y, x, 0, static_cast<uint8_t>(10 * y + x));

    cv::Mat mat = ToCvMat(image);
    ASSERT_EQ(mat.rows, 3);
    ASSERT_EQ(mat.cols, 4);
    EXPECT_EQ(mat.type(), CV_8UC1);
    EXPECT_EQ(mat.at<uint8_t>(2, 3), 23);
    image.Set(1, 1, 0, 99);  // shared buffer
    EXPECT_EQ(mat.at<uint8_t>(1, 1), 99);
}

TEST(ImageConversionTest, ConvertsUnitRangeFloatToEightBit) {
    TImageF32Gray image(2, 1);
    image.Set(0, 0, 0, 0.0f);
    image.Set(0, 1, 0, 1.0f);

    cv::Mat mat = ToCvMat(image);
    EXPECT_EQ(mat.type(), CV_8UC1);
    EXPECT_EQ(mat.at<uint8_t>(0, 0), 0);
    EXPECT_EQ(mat.at<uint8_t>(0, 1), 255);
}

} // namespace
} // namespace voyahviz
