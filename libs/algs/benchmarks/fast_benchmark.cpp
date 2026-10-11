// FAST benchmark: features::DetectFast (each SIMD path) against cv::FAST on
// TUM-VI corridor1 frames, at every ORB pyramid level and both extractor
// thresholds, single thread. Run from a Release build:
//
//   fast_benchmark [cam0_data_dir] [frames=20] [repetitions=30]
//
// Two workloads per level and threshold:
//   level - one DetectFast / cv::FAST call on the whole pyramid level;
//   cells - the ORB extractor's workload: one call per 35-px grid cell (a
//           stride-aware sub-view), retried at threshold 7 when a cell has
//           no corner at 20 (only for the 20 row).
// Prints the median time per frame (microseconds) and ours / OpenCV.
// Every timed call's output is also checked against cv::FAST.

#include <voyahslam/features/fast.h>
#include <voyahslam/features/orb/image_pyramid.h>
#include <voyahslam/io/read_image.h>

#include <opencv2/core.hpp>
#include <opencv2/features.hpp>

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <string>
#include <vector>

using namespace voyahslam;
using namespace voyahslam::features;

namespace {

constexpr int kLevels = 8;
constexpr double kScale = 1.2;
constexpr int kInitThreshold = 20, kMinThreshold = 7;

struct View {
    const uint8_t* data;
    int width, height;
    std::ptrdiff_t stride;
};

std::vector<View> OrbCells(const View& level) {
    constexpr int kW = 35, kPatch = 31, kHalfPatch = 15;
    const int n_cols = (level.width - kPatch) / kW, n_rows = (level.height - kPatch) / kW;
    const int cell_width = (level.width - kPatch) / n_cols;
    const int cell_height = (level.height - kPatch) / n_rows;
    std::vector<View> cells;
    for (int i = 0; i < n_rows; ++i) {
        const int y0 = i * cell_height + kHalfPatch, y1 = std::min(y0 + cell_height, level.height);
        for (int j = 0; j < n_cols; ++j) {
            const int x0 = j * cell_width + kHalfPatch, x1 = std::min(x0 + cell_width, level.width);
            cells.push_back({level.data + y0 * level.stride + x0, x1 - x0, y1 - y0, level.stride});
        }
    }
    return cells;
}

// Converted for checking only (outside the timed region).
std::vector<FastCorner> ToCorners(const std::vector<cv::KeyPoint>& keypoints) {
    std::vector<FastCorner> corners;
    corners.reserve(keypoints.size());
    for (const auto& kp : keypoints)
        corners.push_back({static_cast<int>(kp.pt.x), static_cast<int>(kp.pt.y),
                           static_cast<int>(kp.response)});
    return corners;
}

using Clock = std::chrono::steady_clock;

double Median(std::vector<double> values) {
    std::sort(values.begin(), values.end());
    const size_t n = values.size();
    return n % 2 ? values[n / 2] : 0.5 * (values[n / 2 - 1] + values[n / 2]);
}

const char* Name(FastSimd simd) {
    switch (simd) {
        case FastSimd::kScalar: return "scalar";
        case FastSimd::kSse2: return "sse2";
        case FastSimd::kAvx2: return "avx2";
        case FastSimd::kNeon: return "neon";
        default: return "auto";
    }
}

}  // namespace

int main(int argc, char** argv) {
    std::filesystem::path dir;
    if (argc > 1 && *argv[1]) {
        dir = argv[1];
    } else if (const char* env = std::getenv("VOYAHSLAM_CORRIDOR1"); env && *env) {
        dir = std::filesystem::path(env) / "mav0/cam0/data";
    } else {
        const char* home = std::getenv("HOME");
        dir = std::filesystem::path(home ? home : ".") / "Downloads/dataset-corridor1_1024_16/mav0/cam0/data";
    }
    const size_t num_frames = argc > 2 ? std::stoul(argv[2]) : 20;
    const int repetitions = argc > 3 ? std::stoi(argv[3]) : 30;

    std::vector<std::filesystem::path> files;
    for (const auto& entry : std::filesystem::directory_iterator(dir))
        if (entry.path().extension() == ".png") files.push_back(entry.path());
    std::sort(files.begin(), files.end());
    if (files.empty()) {
        std::fprintf(stderr, "no frames in %s\n", dir.c_str());
        return 1;
    }
    const size_t step = std::max<size_t>(1, files.size() / num_frames);

    cv::setNumThreads(1);
    std::vector<precision_t> factors(kLevels, 1.0);
    for (int l = 1; l < kLevels; ++l) factors[l] = factors[l - 1] * kScale;

    // pyramids[frame] holds all levels.
    std::vector<orb::ImagePyramid> pyramids;
    pyramids.reserve(num_frames);
    for (size_t f = 0; f < files.size() && pyramids.size() < num_frames; f += step) {
        std::ifstream file(files[f], std::ios::binary);
        const auto image = io::ReadImageFromStream(file);
        pyramids.emplace_back();
        pyramids.back().Initialize(image.Width(), image.Height(), factors);
        pyramids.back().Build(image);
    }

    std::vector<FastSimd> paths;
    for (auto simd : {FastSimd::kScalar, FastSimd::kSse2, FastSimd::kAvx2, FastSimd::kNeon})
        if (IsFastSimdAvailable(simd)) paths.push_back(simd);
    const FastSimd best = ResolveFastSimd(FastSimd::kAuto);

    std::printf("FAST benchmark: %zu corridor1 frames, %d repetitions, single thread; auto = %s\n",
                pyramids.size(), repetitions, Name(best));
    std::printf("median time per frame in microseconds; ratio = ours / OpenCV (< 1: ours faster)\n\n");
    std::printf("| workload | level | size | t | OpenCV |");
    for (auto simd : paths) std::printf(" %s | ratio |", Name(simd));
    std::printf("\n|---|---|---|---|---|");
    for (size_t p = 0; p < paths.size(); ++p) std::printf("---|---|");
    std::printf("\n");

    // Totals of the medians, per workload: [0] OpenCV, [1 + p] path p.
    std::vector<double> totals_level(1 + paths.size(), 0.0), totals_cells(1 + paths.size(), 0.0);
    std::vector<FastCorner> ours;
    std::vector<cv::KeyPoint> keypoints;
    bool all_equal = true;

    for (int cells_mode = 0; cells_mode < 2; ++cells_mode) {
        for (int level = 0; level < kLevels; ++level) {
            for (int threshold : {kInitThreshold, kMinThreshold}) {
                std::vector<std::vector<double>> samples(1 + paths.size());
                for (int rep = 0; rep < repetitions; ++rep) {
                    for (auto& pyramid : pyramids) {
                        const auto& image = pyramid[level];
                        const View view{&image.At(0, 0, 0), static_cast<int>(image.Width()),
                                        static_cast<int>(image.Height()),
                                        static_cast<std::ptrdiff_t>(image.Stride())};
                        const std::vector<View> views =
                            cells_mode ? OrbCells(view) : std::vector<View>{view};

                        // OpenCV.
                        auto start = Clock::now();
                        size_t cv_count = 0;
                        for (const View& v : views) {
                            const cv::Mat mat(v.height, v.width, CV_8U, const_cast<uint8_t*>(v.data),
                                              static_cast<size_t>(v.stride));
                            cv::FAST(mat, keypoints, threshold, true);
                            if (cells_mode && keypoints.empty() && threshold == kInitThreshold)
                                cv::FAST(mat, keypoints, kMinThreshold, true);
                            cv_count += keypoints.size();
                        }
                        samples[0].push_back(std::chrono::duration<double, std::micro>(Clock::now() - start).count());

                        for (size_t p = 0; p < paths.size(); ++p) {
                            start = Clock::now();
                            size_t count = 0;
                            for (const View& v : views) {
                                DetectFast(v.data, v.width, v.height, v.stride, threshold, true, ours, paths[p]);
                                if (cells_mode && ours.empty() && threshold == kInitThreshold)
                                    DetectFast(v.data, v.width, v.height, v.stride, kMinThreshold, true, ours,
                                               paths[p]);
                                count += ours.size();
                            }
                            samples[1 + p].push_back(
                                std::chrono::duration<double, std::micro>(Clock::now() - start).count());
                            if (count != cv_count) all_equal = false;
                        }

                        // Exactness check on the first repetition (untimed).
                        if (rep == 0 && !cells_mode) {
                            const cv::Mat mat(view.height, view.width, CV_8U, const_cast<uint8_t*>(view.data),
                                              static_cast<size_t>(view.stride));
                            cv::FAST(mat, keypoints, threshold, true);
                            const auto expected = ToCorners(keypoints);
                            for (auto simd : paths)
                                if (DetectFast(view.data, view.width, view.height, view.stride, threshold, true,
                                               simd) != expected)
                                    all_equal = false;
                        }
                    }
                }
                const auto& image = pyramids[0][level];
                const double cv_median = Median(samples[0]);
                auto& totals = cells_mode ? totals_cells : totals_level;
                totals[0] += cv_median;
                std::printf("| %s | %d | %ux%u | %d | %.1f |", cells_mode ? "cells" : "level", level,
                            image.Width(), image.Height(), threshold, cv_median);
                for (size_t p = 0; p < paths.size(); ++p) {
                    const double m = Median(samples[1 + p]);
                    totals[1 + p] += m;
                    std::printf(" %.1f | %.2f |", m, m / cv_median);
                }
                std::printf("\n");
            }
        }
    }

    for (int cells_mode = 0; cells_mode < 2; ++cells_mode) {
        const auto& totals = cells_mode ? totals_cells : totals_level;
        std::printf("| %s total | all | | both | %.1f |", cells_mode ? "cells" : "level", totals[0]);
        for (size_t p = 0; p < paths.size(); ++p)
            std::printf(" %.1f | %.2f |", totals[1 + p], totals[1 + p] / totals[0]);
        std::printf("\n");
    }
    std::printf("\noutputs identical to cv::FAST: %s\n", all_equal ? "yes" : "NO");
    return all_equal ? 0 : 2;
}
