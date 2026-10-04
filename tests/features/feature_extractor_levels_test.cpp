#include <gtest/gtest.h>

#include <cmath>
#include <random>
#include <set>
#include <vector>

#include "voyahslam/features/orb_feature_extractor.h"
#include "voyahslam/features/sift_feature_extractor.h"
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

TEST(SiftLevelsTest, LevelFromOctave) {
    auto pack = [](int octave, int layer) { return (layer << 8) | (octave & 0xFF); };
    EXPECT_EQ(SIFTFeatureExtractor::LevelFromOctave(pack(-1, 1)), 1);
    EXPECT_EQ(SIFTFeatureExtractor::LevelFromOctave(pack(0, 2)), 5);
    EXPECT_EQ(SIFTFeatureExtractor::LevelFromOctave(pack(2, 3)), 12);

    SIFTFeatureExtractor extractor;
    EXPECT_DOUBLE_EQ(extractor.ScaleFactor(0), 0.5);
    EXPECT_DOUBLE_EQ(extractor.ScaleFactor(3), 1.0);
    EXPECT_DOUBLE_EQ(extractor.ScaleFactor(6), 2.0);
}

TEST(SiftLevelsTest, DetectAndExtractRespectsCountAndRadians) {
    SIFTFeatureExtractor extractor;
    auto image = MakeNoiseImage(512);
    constexpr size_t kRequested = 50;
    auto features = extractor.DetectAndExtract(image, kRequested);

    // The image yields far more than 50 keypoints, so exactly 50 are kept.
    ASSERT_EQ(features.points().cols(), static_cast<Eigen::Index>(kRequested));
    EXPECT_EQ(features.strengths().cols(), static_cast<Eigen::Index>(kRequested));
    EXPECT_EQ(features.angles().cols(), static_cast<Eigen::Index>(kRequested));
    EXPECT_EQ(features.descriptors().cols(), static_cast<Eigen::Index>(kRequested));
    EXPECT_EQ(features.levels().cols(), static_cast<Eigen::Index>(kRequested));
    for (Eigen::Index i = 0; i < features.angles().cols(); ++i) {
        EXPECT_GE(features.angles()(i), 0.0F);
        EXPECT_LT(features.angles()(i), 2.0F * static_cast<float>(M_PI));
        if (i > 0) EXPECT_LE(features.strengths()(i), features.strengths()(i - 1));
    }
}
