#include <gtest/gtest.h>

#include <cstring>
#include <random>
#include <thread>
#include <vector>

#include "voyahslam/features/orb_feature_extractor.h"

using namespace voyahslam;
using namespace voyahslam::features;

namespace {

using Features = OrbFeatureExtractor::TFeatures;

// Seeded multi-scale blocky noise (random cells of 4 and 16 px): corners and blobs at several
// scales. `seed` makes distinct frames of the same size.
images::TWorkingImage MakeNoiseImage(unsigned size, unsigned seed = 42) {
    std::mt19937 rng(seed);
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

constexpr size_t kFeatures = 800;

}  // namespace

TEST(OrbExtractorWorkspaceTest, ConcurrentCallsEqualSingleThreadResult) {
    const OrbFeatureExtractor extractor;
    std::vector<images::TWorkingImage> images;
    for (unsigned i = 0; i < 20; ++i) images.push_back(MakeNoiseImage(i % 2 ? 256 : 384, 100 + i));

    // Reference: single thread, a fresh workspace per image.
    std::vector<Features> reference;
    for (const auto& image : images) {
        OrbFeatureExtractor::Workspace workspace;
        reference.push_back(extractor.DetectAndExtract(image, kFeatures, workspace));
    }

    // Two threads share the one extractor; each owns its Workspace and takes every image.
    std::vector<std::vector<Features>> results(2);
    auto work = [&](size_t t) {
        OrbFeatureExtractor::Workspace workspace;
        for (const auto& image : images)
            results[t].push_back(extractor.DetectAndExtract(image, kFeatures, workspace));
    };
    std::thread first(work, 0), second(work, 1);
    first.join();
    second.join();

    for (size_t t = 0; t < 2; ++t) {
        ASSERT_EQ(results[t].size(), reference.size());
        for (size_t i = 0; i < reference.size(); ++i)
            EXPECT_TRUE(SameFeatures(results[t][i], reference[i])) << "thread " << t << " image " << i;
    }
}

TEST(OrbExtractorWorkspaceTest, AllocatingOverloadEqualsWorkspaceOverload) {
    const OrbFeatureExtractor extractor;
    const auto image = MakeNoiseImage(384);
    OrbFeatureExtractor::Workspace workspace;
    EXPECT_TRUE(SameFeatures(extractor.DetectAndExtract(image, kFeatures),
                             extractor.DetectAndExtract(image, kFeatures, workspace)));
}

TEST(OrbExtractorWorkspaceTest, ReusedWorkspaceEqualsFreshWorkspace) {
    const OrbFeatureExtractor extractor;
    OrbFeatureExtractor::Workspace reused;
    for (unsigned frame = 0; frame < 5; ++frame) {
        const auto image = MakeNoiseImage(320, 7 + frame);
        const auto expected = extractor.DetectAndExtract(image, kFeatures);
        EXPECT_TRUE(SameFeatures(extractor.DetectAndExtract(image, kFeatures, reused), expected))
            << "frame " << frame;
    }
}

TEST(OrbExtractorWorkspaceTest, WorkspaceReinitialisesOnNewImageSize) {
    const OrbFeatureExtractor extractor;
    OrbFeatureExtractor::Workspace workspace;
    for (unsigned size : {512u, 256u, 512u, 384u}) {
        const auto image = MakeNoiseImage(size, size);
        const auto expected = extractor.DetectAndExtract(image, kFeatures);
        EXPECT_TRUE(SameFeatures(extractor.DetectAndExtract(image, kFeatures, workspace), expected))
            << "size " << size;
    }
}

TEST(OrbExtractorWorkspaceTest, WorkspaceReinitialisesForAnotherLevelCount) {
    const OrbFeatureExtractor eight_levels(1.2, 8);
    const OrbFeatureExtractor four_levels(1.2, 4);
    const auto image = MakeNoiseImage(384);
    OrbFeatureExtractor::Workspace workspace;
    for (const OrbFeatureExtractor* extractor : {&eight_levels, &four_levels, &eight_levels}) {
        const auto expected = extractor->DetectAndExtract(image, kFeatures);
        EXPECT_TRUE(SameFeatures(extractor->DetectAndExtract(image, kFeatures, workspace), expected))
            << extractor->num_levels() << " levels";
    }
}
