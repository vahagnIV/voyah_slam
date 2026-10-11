// Regression: the ORB extractor's output on real corridor1 frames must stay
// byte-identical (keypoints, levels, responses, angles, descriptors).
//
// The golden digests in data/orb_corridor1_golden.txt were recorded with the
// OpenCV-based extractor (cv::FAST per grid cell) before FAST was ported, so
// this test pins the port to the old behaviour. To re-record after an
// intended change, run with VOYAHSLAM_RECORD_ORB_GOLDEN=1.

#include <gtest/gtest.h>

#include "corridor1_frames.h"
#include "voyahslam/features/orb_feature_extractor.h"

#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <map>
#include <sstream>
#include <string>

using namespace voyahslam;
using namespace voyahslam::features;

namespace {

constexpr size_t kNumFeatures = 1500;  // examples/full_slam kNumFeatures
constexpr size_t kNumFrames = 5;

uint64_t Fnv1a(const void* data, size_t bytes, uint64_t hash = 1469598103934665603ull) {
    const auto* p = static_cast<const uint8_t*>(data);
    for (size_t i = 0; i < bytes; ++i) {
        hash ^= p[i];
        hash *= 1099511628211ull;
    }
    return hash;
}

template <typename T>
uint64_t HashValue(const T& value, uint64_t hash) {
    return Fnv1a(&value, sizeof(T), hash);
}

// The digests hash raw float bytes, so they pin x86-64 with this compiler. On
// another compiler or on arm64 (FMA contraction) they may need re-recording
// with VOYAHSLAM_RECORD_ORB_GOLDEN=1 for reasons unrelated to FAST.
// One line per (frame, level): count and digests of each output array,
// restricted to that level's columns (the extractor emits levels in order).
std::string Digest(const std::string& frame, const OrbFeatureExtractor::TFeatures& features,
                   size_t num_levels) {
    std::ostringstream out;
    for (size_t level = 0; level < num_levels; ++level) {
        uint64_t points = 1469598103934665603ull, strengths = points, angles = points,
                 descriptors = points;
        size_t count = 0;
        for (Eigen::Index i = 0; i < features.levels().cols(); ++i) {
            if (features.levels()(i) != level) continue;
            ++count;
            points = HashValue(features.points()(0, i), points);
            points = HashValue(features.points()(1, i), points);
            strengths = HashValue(features.strengths()(i), strengths);
            angles = HashValue(features.angles()(i), angles);
            for (Eigen::Index b = 0; b < features.descriptors().rows(); ++b)
                descriptors = HashValue(features.descriptors()(b, i), descriptors);
        }
        out << frame << " level " << level << " count " << count << " points " << std::hex
            << points << " strengths " << strengths << " angles " << angles << " descriptors "
            << descriptors << std::dec << "\n";
    }
    return out.str();
}

std::string GoldenPath() { return std::string(VOYAH_FEATURES_DATA_DIR) + "/orb_corridor1_golden.txt"; }

}  // namespace

TEST(OrbExtractorGoldenTest, Corridor1OutputUnchanged) {
    const auto frames = test::Corridor1SampleFrames(kNumFrames);
    if (!test::Corridor1Present()) GTEST_SKIP() << "corridor1 dataset not found";
    ASSERT_GE(frames.size(), kNumFrames)
        << "corridor1 dataset at " << test::Corridor1Cam0Dir() << " is present but has too few frames";

    OrbFeatureExtractor extractor;  // defaults: 1.2, 8 levels, FAST 20 / 7
    std::string actual;
    for (const auto& path : frames) {
        const auto image = test::ReadFrame(path);
        const auto features = extractor.DetectAndExtract(image, kNumFeatures);
        // Levels are emitted in order, so per-level digests cover everything.
        for (Eigen::Index i = 1; i < features.levels().cols(); ++i)
            ASSERT_LE(features.levels()(i - 1), features.levels()(i));
        actual += Digest(path.filename().string(), features, extractor.num_levels());
    }

    if (const char* record = std::getenv("VOYAHSLAM_RECORD_ORB_GOLDEN"); record && *record) {
        std::ofstream(GoldenPath()) << actual;
        GTEST_SKIP() << "recorded " << GoldenPath();
    }

    std::ifstream file(GoldenPath());
    ASSERT_TRUE(file) << "missing " << GoldenPath();
    std::stringstream expected;
    expected << file.rdbuf();

    // Line by line, so a failure names the frame and level.
    std::istringstream expected_lines(expected.str()), actual_lines(actual);
    std::string expected_line, actual_line;
    size_t lines = 0;
    while (std::getline(expected_lines, expected_line)) {
        ASSERT_TRUE(std::getline(actual_lines, actual_line)) << "fewer lines than the golden";
        EXPECT_EQ(actual_line, expected_line);
        ++lines;
    }
    EXPECT_FALSE(std::getline(actual_lines, actual_line)) << "more lines than the golden";
    EXPECT_EQ(lines, kNumFrames * extractor.num_levels());
}
