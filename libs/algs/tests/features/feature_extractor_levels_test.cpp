#include <gtest/gtest.h>

#include <cmath>
#include <cstring>
#include <limits>
#include <random>
#include <set>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "voyahslam/features/orb_feature_extractor.h"
#include "voyahslam/images/filter.h"
#include "voyahslam/images/gaussian_blur.h"

using namespace voyahslam;
using namespace voyahslam::features;

namespace {

// Seeded multi-scale blocky noise (random cells of 4 and 16 px): corners and
// blobs at several scales.
images::TWorkingImage MakeNoiseImage(unsigned size) {
    std::mt19937 rng(42);
    std::uniform_int_distribution<int> dist(0, 127);
    const unsigned fine = size / 4, coarse = size / 16;
    std::vector<int> fine_cells(fine * fine), coarse_cells(coarse * coarse);
    for (int& v : fine_cells) v = dist(rng);
    for (int& v : coarse_cells) v = dist(rng);
    images::TWorkingImage image(size, size);
    for (unsigned y = 0; y < size; ++y)
        for (unsigned x = 0; x < size; ++x)
            image.Set(y, x, 0,
                      static_cast<uint8_t>(fine_cells[(y / 4) * fine + x / 4] / 2 +
                                           coarse_cells[(y / 16) * coarse + x / 16]));
    return image;
}

// The same noise for any width and height (the cell grids are sized with
// rounded-up divisions, so every pixel indexes a cell).
images::TWorkingImage MakeNoiseImage(unsigned width, unsigned height) {
    std::mt19937 rng(42);
    std::uniform_int_distribution<int> dist(0, 127);
    const unsigned fine_w = (width + 3) / 4, fine_h = (height + 3) / 4;
    const unsigned coarse_w = (width + 15) / 16, coarse_h = (height + 15) / 16;
    std::vector<int> fine_cells(fine_w * fine_h), coarse_cells(coarse_w * coarse_h);
    for (int& v : fine_cells) v = dist(rng);
    for (int& v : coarse_cells) v = dist(rng);
    images::TWorkingImage image(width, height);
    for (unsigned y = 0; y < height; ++y)
        for (unsigned x = 0; x < width; ++x)
            image.Set(y, x, 0,
                      static_cast<uint8_t>(fine_cells[(y / 4) * fine_w + x / 4] / 2 +
                                           coarse_cells[(y / 16) * coarse_w + x / 16]));
    return image;
}

using Features = OrbFeatureExtractor::TFeatures;

// Field-by-field, bit-exact comparison (the same fields the golden test digests).
::testing::AssertionResult SameFeatures(const Features& a, const Features& b) {
    if (a.points().cols() != b.points().cols())
        return ::testing::AssertionFailure()
               << "count " << a.points().cols() << " vs " << b.points().cols();
    const auto same_bytes = [](const auto& x, const auto& y) {
        return x.rows() == y.rows() && x.cols() == y.cols() &&
               (x.size() == 0 || std::memcmp(x.data(), y.data(), sizeof(x(0)) * x.size()) == 0);
    };
    if (!same_bytes(a.points(), b.points())) return ::testing::AssertionFailure() << "points differ";
    if (!same_bytes(a.strengths(), b.strengths()))
        return ::testing::AssertionFailure() << "strengths differ";
    if (!same_bytes(a.angles(), b.angles())) return ::testing::AssertionFailure() << "angles differ";
    if (!same_bytes(a.levels(), b.levels())) return ::testing::AssertionFailure() << "levels differ";
    if (!same_bytes(a.descriptors(), b.descriptors()))
        return ::testing::AssertionFailure() << "descriptors differ";
    return ::testing::AssertionSuccess();
}

// True when `call` throws std::invalid_argument whose message contains `part`.
template <typename F> bool ThrowsInvalidArgumentContaining(F&& call, const std::string& part) {
    try {
        call();
    } catch (const std::invalid_argument& e) {
        return std::string(e.what()).find(part) != std::string::npos;
    } catch (...) {
        return false;
    }
    return false;
}

}  // namespace

TEST(OrbLevelsTest, LevelsInRangeAndScaleFactor) {
    OrbFeatureExtractor extractor;
    auto image = MakeNoiseImage(512);
    auto features = extractor.DetectAndExtract(image, 1000);

    ASSERT_GT(features.points().cols(), 0);
    ASSERT_EQ(features.levels().cols(), features.points().cols());
    EXPECT_EQ(extractor.num_levels(), 8u);
    std::set<int> distinct;
    for (Eigen::Index i = 0; i < features.levels().cols(); ++i) {
        EXPECT_LT(features.levels()(i), 8);
        distinct.insert(features.levels()(i));
    }
    EXPECT_GE(distinct.size(), 2u);
    EXPECT_DOUBLE_EQ(extractor.ScaleFactor(0), 1.0);
    EXPECT_NEAR(extractor.ScaleFactor(3), std::pow(1.2, 3), 1e-12);
}

// ORB-SLAM3 rounds the rotated BRIEF offsets (cvRound); truncation picks other
// pixels. Pattern point 1 is (9, 5): at 30 degrees it rotates to (5.29, 8.83).
TEST(OrbDescriptorTest, RotatedOffsetsAreRounded) {
    const float angle = static_cast<float>(M_PI / 6.0);
    const auto offset = OrbFeatureExtractor::RotatedPatternOffset(1, angle);
    EXPECT_EQ(offset.x(), 5);
    EXPECT_EQ(offset.y(), 9);  // truncation would give 8

    // Rotated by the ORB-SLAM3 formula over all 512 points.
    // (x', y') = (x*a - y*b, x*b + y*a), a = cos, b = sin.
    const float a = std::cos(angle), b = std::sin(angle);
    int differing_from_truncation = 0;
    for (int k = 0; k < 512; ++k) {
        const auto o = OrbFeatureExtractor::RotatedPatternOffset(k, angle);
        const auto base = OrbFeatureExtractor::RotatedPatternOffset(k, 0.f);
        const float x = static_cast<float>(base.x()), y = static_cast<float>(base.y());
        EXPECT_EQ(o.x(), std::lround(x * a - y * b));
        EXPECT_EQ(o.y(), std::lround(x * b + y * a));
        if (o.x() != static_cast<int>(x * a - y * b) || o.y() != static_cast<int>(x * b + y * a))
            ++differing_from_truncation;
    }
    EXPECT_GT(differing_from_truncation, 0);
}

TEST(OrbDescriptorTest, DescriptorBitsUseRoundedOffsets) {
    OrbFeatureExtractor extractor;
    auto image = MakeNoiseImage(128);
    const float angle = static_cast<float>(M_PI / 6.0);
    geometry::TPoints2DF points(2, 1);
    points << 64.f, 64.f;
    geometry::TRowVectorF angles(1);
    angles << angle;
    auto descriptors = extractor.ComputeDescriptors(image, points, angles);

    auto blurred = images::Filter<images::TWorkingImage::TScalar, 1, float>::ApplySeparable(
        images::CreateGaussianKernelSeparableXFromSize(7, 2.),
        images::CreateGaussianKernelSeparableYFromSize(7, 2.), image);
    // Pair k uses pattern points 2k and 2k+1; bit j of byte i is pair 8i+j.
    for (int byte = 0; byte < OrbFeatureExtractor::DescriptorSize; ++byte) {
        uint8_t expected = 0;
        for (int j = 0; j < 8; ++j) {
            const int k = 8 * byte + j;
            const auto p0 = OrbFeatureExtractor::RotatedPatternOffset(2 * k, angle);
            const auto p1 = OrbFeatureExtractor::RotatedPatternOffset(2 * k + 1, angle);
            const float t0 = blurred.At(64 + p0.y(), 64 + p0.x(), 0);
            const float t1 = blurred.At(64 + p1.y(), 64 + p1.x(), 0);
            expected |= static_cast<uint8_t>((t0 < t1) << j);
        }
        EXPECT_EQ(descriptors(byte, 0), expected) << "byte " << byte;
    }
}

// Review CR-01: an image whose smallest pyramid level is below 66 px (PATCH_SIZE 31
// plus one 35-px FAST cell) divided by zero in release builds; the extractor now
// throws std::invalid_argument in every build, before it touches the workspace.
TEST(OrbInputTest, ImageTooSmallForThePyramidThrows) {
    const OrbFeatureExtractor extractor;  // 1.2, 8 levels: smallest level is image / 3.58
    for (const auto& [w, h] : {std::pair{224u, 224u}, std::pair{234u, 480u}, std::pair{480u, 234u}}) {
        const auto image = MakeNoiseImage(w, h);
        OrbFeatureExtractor::Workspace workspace;
        EXPECT_TRUE(ThrowsInvalidArgumentContaining(
            [&] { (void)extractor.DetectAndExtract(image, 500, workspace); }, "too small"))
            << w << "x" << h << " (workspace overload)";
        EXPECT_TRUE(ThrowsInvalidArgumentContaining(
            [&] { (void)extractor.DetectAndExtract(image, 500); }, "too small"))
            << w << "x" << h << " (allocating overload)";
    }
    const images::TWorkingImage empty;
    OrbFeatureExtractor::Workspace workspace;
    EXPECT_TRUE(ThrowsInvalidArgumentContaining(
        [&] { (void)extractor.DetectAndExtract(empty, 500, workspace); }, "too small"));
    EXPECT_TRUE(ThrowsInvalidArgumentContaining(
        [&] { (void)extractor.DetectAndExtract(empty, 500); }, "too small"));
}

TEST(OrbInputTest, SmallestAcceptedImageExtracts) {
    const OrbFeatureExtractor extractor;
    // 235 / 1.2^7 rounds to 66, the smallest level the grid logic can handle.
    const auto image = MakeNoiseImage(235, 235);
    EXPECT_NO_THROW((void)extractor.DetectAndExtract(image, 500));

    const OrbFeatureExtractor one_level(1.2, 1);
    EXPECT_NO_THROW((void)one_level.DetectAndExtract(MakeNoiseImage(66, 66), 100));
    EXPECT_THROW((void)one_level.DetectAndExtract(MakeNoiseImage(65, 66), 100), std::invalid_argument);
    EXPECT_THROW((void)one_level.DetectAndExtract(MakeNoiseImage(66, 65), 100), std::invalid_argument);
}

TEST(OrbInputTest, RejectedImageLeavesTheWorkspaceUsable) {
    const OrbFeatureExtractor extractor;
    OrbFeatureExtractor::Workspace workspace;
    const auto good = MakeNoiseImage(512, 512);
    const Features first = extractor.DetectAndExtract(good, 800, workspace);
    ASSERT_GT(first.points().cols(), 0);

    EXPECT_THROW((void)extractor.DetectAndExtract(MakeNoiseImage(224, 224), 800, workspace),
                 std::invalid_argument);

    const Features again = extractor.DetectAndExtract(good, 800, workspace);
    EXPECT_TRUE(SameFeatures(first, again));
}

TEST(OrbInputTest, InvalidConstructorArgumentsThrow) {
    EXPECT_THROW(OrbFeatureExtractor(1.2, 0), std::invalid_argument);
    constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();
    constexpr double kInf = std::numeric_limits<double>::infinity();
    for (const double scale : {1.0, 0.5, 0.0, -1.2, kNaN, kInf}) {
        EXPECT_THROW(OrbFeatureExtractor(scale, 8), std::invalid_argument) << "scale " << scale;
    }
    EXPECT_NO_THROW(OrbFeatureExtractor(1.2, 1));
    EXPECT_NO_THROW(OrbFeatureExtractor());
}
