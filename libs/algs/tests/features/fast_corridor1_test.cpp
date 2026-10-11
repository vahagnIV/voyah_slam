// FAST port against live cv::FAST on real TUM-VI corridor1 frames: every
// pyramid level the ORB extractor builds, both of its thresholds, the whole
// level and every grid cell it hands to FAST (stride-aware sub-views).

#include <gtest/gtest.h>

#include "corridor1_frames.h"
#include "fast_test_support.h"

#include <voyahslam/features/fast.h>
#include <voyahslam/features/orb/image_pyramid.h>

#include <algorithm>
#include <cmath>
#include <vector>

using namespace voyahslam;
using namespace voyahslam::features;

namespace {

constexpr size_t kNumFrames = 3;
constexpr int kLevels = 8;         // OrbFeatureExtractor defaults
constexpr double kScale = 1.2;
constexpr int kThresholds[] = {20, 7};  // init_threshold_FAST, min_threshold_FAST

// The ORB extractor's grid (OrbFeatureExtractor::ComputePointsPerLevel).
struct Cell {
    int x, y, width, height;
};

std::vector<Cell> OrbCells(int width, int height) {
    constexpr int kW = 35, kPatch = 31, kHalfPatch = 15;
    const int n_cols = (width - kPatch) / kW, n_rows = (height - kPatch) / kW;
    const int cell_width = (width - kPatch) / n_cols, cell_height = (height - kPatch) / n_rows;
    std::vector<Cell> cells;
    for (int i = 0; i < n_rows; ++i) {
        const int y0 = i * cell_height + kHalfPatch, y1 = std::min(y0 + cell_height, height);
        for (int j = 0; j < n_cols; ++j) {
            const int x0 = j * cell_width + kHalfPatch, x1 = std::min(x0 + cell_width, width);
            cells.push_back({x0, y0, x1 - x0, y1 - y0});
        }
    }
    return cells;
}

std::vector<precision_t> ScaleFactors() {
    std::vector<precision_t> factors(kLevels, 1.0);
    for (int l = 1; l < kLevels; ++l) factors[l] = factors[l - 1] * kScale;
    return factors;
}

}  // namespace

TEST(FastCorridor1Test, EveryLevelThresholdAndCellMatchesOpenCv) {
    const auto frames = test::Corridor1SampleFrames(kNumFrames);
    if (frames.size() < kNumFrames) GTEST_SKIP() << "corridor1 dataset not found";
    const auto paths = test::RunnableFastPaths();

    size_t total_corners = 0, total_cells = 0;
    for (const auto& path : frames) {
        const auto image = test::ReadFrame(path);
        orb::ImagePyramid pyramid;
        pyramid.Initialize(image.Width(), image.Height(), ScaleFactors());
        pyramid.Build(image);
        for (int level = 0; level < kLevels; ++level) {
            const auto& view = pyramid[level];
            const uint8_t* data = &view.At(0, 0, 0);
            const int width = static_cast<int>(view.Width()), height = static_cast<int>(view.Height());
            const std::ptrdiff_t stride = view.Stride();
            for (int threshold : kThresholds) {
                for (bool nms : {true, false}) {
                    const auto expected = test::CvFast(data, width, height, stride, threshold, nms);
                    total_corners += expected.size();
                    for (auto simd : paths) {
                        ASSERT_EQ(DetectFast(data, width, height, stride, threshold, nms, simd), expected)
                            << path.filename() << " level " << level << " t=" << threshold
                            << " nms=" << nms << " " << test::FastSimdName(simd);
                    }
                }
                for (const Cell& cell : OrbCells(width, height)) {
                    const uint8_t* cell_data = data + cell.y * stride + cell.x;
                    const auto expected =
                        test::CvFast(cell_data, cell.width, cell.height, stride, threshold, true);
                    ++total_cells;
                    for (auto simd : paths) {
                        ASSERT_EQ(DetectFast(cell_data, cell.width, cell.height, stride, threshold, true, simd),
                                  expected)
                            << path.filename() << " level " << level << " t=" << threshold << " cell ("
                            << cell.x << ", " << cell.y << ") " << test::FastSimdName(simd);
                    }
                }
            }
        }
    }
    // Sanity: the comparison covered real corners.
    EXPECT_GT(total_corners, 10000u);
    EXPECT_GT(total_cells, 1000u);
}
