// FAST port (features::DetectFast) against live cv::FAST(img, kps, t, nms),
// TYPE_9_16, on synthetic images: every runnable SIMD path must match
// OpenCV exactly (same corners, same order, same responses).

#include <gtest/gtest.h>

#include "fast_test_support.h"

#include <voyahslam/features/fast.h>
#include <voyahslam/images/image.h>

#include <cstdint>
#include <random>
#include <vector>

using namespace voyahslam;
using namespace voyahslam::features;

namespace {

// Row-major 8-bit image with an explicit stride (>= width).
struct Buffer {
    int width, height;
    std::ptrdiff_t stride;
    std::vector<uint8_t> pixels;

    Buffer(int w, int h, std::ptrdiff_t s, uint8_t fill = 0)
        : width(w), height(h), stride(s), pixels(static_cast<size_t>(s) * h, fill) {}
    uint8_t& At(int x, int y) { return pixels[static_cast<size_t>(y) * stride + x]; }
    const uint8_t* Data() const { return pixels.data(); }
};

// Blocky noise with full-range values: strong corners, plateaus (equal
// scores, exercising strict NMS) and saturated neighbours near 0 and 255.
Buffer MakeNoise(int width, int height, std::ptrdiff_t stride, uint32_t seed, int cell = 3) {
    Buffer buffer(width, height, stride, 77);  // padding bytes stay 77
    std::mt19937 rng(seed);
    std::uniform_int_distribution<int> value(0, 255);
    const int cells_x = (width + cell - 1) / cell, cells_y = (height + cell - 1) / cell;
    std::vector<uint8_t> cells(static_cast<size_t>(cells_x) * cells_y);
    for (auto& c : cells) c = static_cast<uint8_t>(value(rng));
    for (int y = 0; y < height; ++y)
        for (int x = 0; x < width; ++x) {
            uint8_t v = cells[(y / cell) * cells_x + x / cell];
            if (rng() % 7 == 0) v = static_cast<uint8_t>(value(rng));  // speckle
            buffer.At(x, y) = v;
        }
    return buffer;
}

void ExpectAllPathsMatchOpenCv(const uint8_t* data, int width, int height, std::ptrdiff_t stride,
                               int threshold, bool nonmax_suppression) {
    const auto expected = test::CvFast(data, width, height, stride, threshold, nonmax_suppression);
    for (auto simd : test::RunnableFastPaths()) {
        const auto actual =
            DetectFast(data, width, height, stride, threshold, nonmax_suppression, simd);
        ASSERT_EQ(actual, expected) << test::FastSimdName(simd) << " " << width << "x" << height
                                    << " stride " << stride << " t=" << threshold
                                    << " nms=" << nonmax_suppression;
    }
    // kAuto is one of the paths above.
    EXPECT_EQ(DetectFast(data, width, height, stride, threshold, nonmax_suppression), expected);
}

}  // namespace

TEST(FastTest, ScalarAndSse2OrNeonAreAvailable) {
    EXPECT_TRUE(IsFastSimdAvailable(FastSimd::kScalar));
    EXPECT_TRUE(IsFastSimdAvailable(FastSimd::kAuto));
#if defined(__x86_64__) || defined(_M_X64)
    EXPECT_TRUE(IsFastSimdAvailable(FastSimd::kSse2));
    EXPECT_FALSE(IsFastSimdAvailable(FastSimd::kNeon));
#elif defined(__aarch64__)
    EXPECT_TRUE(IsFastSimdAvailable(FastSimd::kNeon));
    EXPECT_FALSE(IsFastSimdAvailable(FastSimd::kSse2));
#endif
    EXPECT_NE(ResolveFastSimd(FastSimd::kAuto), FastSimd::kAuto);
    EXPECT_TRUE(IsFastSimdAvailable(ResolveFastSimd(FastSimd::kAuto)));
}

TEST(FastTest, NoiseMatchesOpenCvAcrossWidthsAndThresholds) {
    // Widths straddle the 16- and 32-pixel SIMD blocks and the scalar tail.
    for (int width : {7, 8, 15, 19, 20, 22, 23, 34, 35, 36, 38, 39, 51, 52, 67, 68, 70, 71, 100, 131}) {
        for (int height : {7, 9, 23}) {
            const auto image = MakeNoise(width, height, width, 1000u + width * 31 + height);
            for (int threshold : {0, 1, 7, 20, 40, 100, 254, 255}) {
                for (bool nms : {true, false}) {
                    ExpectAllPathsMatchOpenCv(image.Data(), width, height, width, threshold, nms);
                }
            }
        }
    }
}

TEST(FastTest, SmoothNoiseMatchesOpenCv) {
    // Larger cells: sparse corners, so SIMD blocks skip (the j -= 8 / 16 rewind).
    for (int cell : {5, 9, 17}) {
        const auto image = MakeNoise(160, 120, 160, 7u + cell, cell);
        for (int threshold : {7, 20}) {
            for (bool nms : {true, false}) {
                ExpectAllPathsMatchOpenCv(image.Data(), 160, 120, 160, threshold, nms);
            }
        }
    }
}

TEST(FastTest, StrideLargerThanWidth) {
    // A 50x40 view inside a 97-wide buffer, offset (5, 4): padding is noise too.
    const auto big = MakeNoise(97, 60, 97, 99u);
    const uint8_t* view = big.Data() + 4 * 97 + 5;
    for (int threshold : {7, 20}) {
        for (bool nms : {true, false}) {
            ExpectAllPathsMatchOpenCv(view, 50, 40, 97, threshold, nms);
        }
    }
    // Same pixels, compact copy: identical corners.
    Buffer compact(50, 40, 50);
    for (int y = 0; y < 40; ++y)
        for (int x = 0; x < 50; ++x) compact.At(x, y) = view[y * 97 + x];
    for (auto simd : test::RunnableFastPaths()) {
        EXPECT_EQ(DetectFast(view, 50, 40, 97, 20, true, simd),
                  DetectFast(compact.Data(), 50, 40, 50, 20, true, simd))
            << test::FastSimdName(simd);
    }
}

TEST(FastTest, FlatImageHasNoCorners) {
    for (uint8_t level : {uint8_t{0}, uint8_t{128}, uint8_t{255}}) {
        Buffer flat(64, 48, 64, level);
        for (int threshold : {0, 20}) {
            for (bool nms : {true, false}) {
                for (auto simd : test::RunnableFastPaths()) {
                    EXPECT_TRUE(DetectFast(flat.Data(), 64, 48, 64, threshold, nms, simd).empty());
                }
                ExpectAllPathsMatchOpenCv(flat.Data(), 64, 48, 64, threshold, nms);
            }
        }
    }
}

TEST(FastTest, ImagesSmallerThan7x7HaveNoCorners) {
    for (int width = 1; width <= 8; ++width) {
        for (int height = 1; height <= 8; ++height) {
            const auto image = MakeNoise(width, height, width + 3, 5u + width * 8 + height, 1);
            for (bool nms : {true, false}) {
                if (width < 7 || height < 7) {
                    for (auto simd : test::RunnableFastPaths()) {
                        EXPECT_TRUE(DetectFast(image.Data(), width, height, width + 3, 0, nms, simd)
                                        .empty())
                            << width << "x" << height;
                    }
                }
                ExpectAllPathsMatchOpenCv(image.Data(), width, height, width + 3, 0, nms);
            }
        }
    }
}

TEST(FastTest, CornersAtTheThreePixelBorder) {
    // A bright pixel on a dark background is a corner (all 16 circle pixels
    // darker). Place one at each extreme detectable position: x, y in {3, n-4}.
    constexpr int kWidth = 40, kHeight = 30;
    for (int threshold : {0, 20, 254}) {
        Buffer image(kWidth, kHeight, kWidth, 0);
        const int xs[] = {3, kWidth - 4}, ys[] = {3, kHeight - 4};
        for (int y : ys)
            for (int x : xs) image.At(x, y) = 255;
        // One pixel outside the border band: never reported.
        image.At(2, 15) = 255;
        image.At(15, kHeight - 3) = 255;
        for (auto simd : test::RunnableFastPaths()) {
            const auto corners = DetectFast(image.Data(), kWidth, kHeight, kWidth, threshold, true, simd);
            const std::vector<FastCorner> expected = {
                {3, 3, 254}, {kWidth - 4, 3, 254}, {3, kHeight - 4, 254}, {kWidth - 4, kHeight - 4, 254}};
            EXPECT_EQ(corners, expected) << test::FastSimdName(simd) << " t=" << threshold;
        }
        ExpectAllPathsMatchOpenCv(image.Data(), kWidth, kHeight, kWidth, threshold, true);
        ExpectAllPathsMatchOpenCv(image.Data(), kWidth, kHeight, kWidth, threshold, false);
    }
}

TEST(FastTest, Threshold255FindsNothingAndThreshold0FindsMost) {
    const auto image = MakeNoise(64, 64, 64, 4242u, 1);
    for (auto simd : test::RunnableFastPaths()) {
        EXPECT_TRUE(DetectFast(image.Data(), 64, 64, 64, 255, false, simd).empty());
        EXPECT_GT(DetectFast(image.Data(), 64, 64, 64, 0, false, simd).size(), 100u);
    }
    ExpectAllPathsMatchOpenCv(image.Data(), 64, 64, 64, 0, true);
    ExpectAllPathsMatchOpenCv(image.Data(), 64, 64, 64, 255, true);
}

TEST(FastTest, WorkingImageOverloadMatchesPointerOverload) {
    const auto noise = MakeNoise(80, 60, 80, 17u);
    images::TWorkingImage image(80, 60);
    for (int y = 0; y < 60; ++y)
        for (int x = 0; x < 80; ++x) image.Set(y, x, 0, noise.pixels[y * 80 + x]);
    EXPECT_EQ(DetectFast(image, 20, true), DetectFast(noise.Data(), 80, 60, 80, 20, true));
    // A sub-view keeps its offset and the parent's stride.
    images::TWorkingImage sub(image, 7, 5, 40, 30);
    EXPECT_EQ(DetectFast(sub, 20, true), DetectFast(noise.Data() + 5 * 80 + 7, 40, 30, 80, 20, true));
}

TEST(FastTest, OutParameterOverloadReplacesContents) {
    const auto image = MakeNoise(64, 64, 64, 8u);
    std::vector<FastCorner> corners = {{1, 2, 3}};
    DetectFast(image.Data(), 64, 64, 64, 20, true, corners);
    EXPECT_EQ(corners, DetectFast(image.Data(), 64, 64, 64, 20, true));
}
