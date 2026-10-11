#ifndef VOYAHSLAM_FEATURES_FAST_H
#define VOYAHSLAM_FEATURES_FAST_H

// FAST corner detector (9-of-16 segment test), ported from OpenCV 5
// (modules/features/src/fast.cpp, fast_score.cpp, fast.avx2.cpp; Apache-2.0,
// original algorithm and code by Edward Rosten, BSD). Bit-exact with
// cv::FAST(image, keypoints, threshold, nonmax_suppression) for TYPE_9_16:
// the same corners, in the same order (row-major), with the same responses.
// See DESIGN.md "FAST corner detector".

#include <voyahslam/images/image.h>

#include <cstddef>
#include <cstdint>
#include <vector>

namespace voyahslam::features {

// A detected corner: pixel position in the view, and its FAST score (the
// largest threshold at which it would still be a corner; 0 without
// non-maximum suppression, as cv::FAST).
struct FastCorner {
    int x;
    int y;
    int response;

    bool operator==(const FastCorner&) const = default;
};

// Implementation used by DetectFast. Every path gives identical output;
// kAuto picks the fastest one this CPU supports (AVX2 > SSE2 on x86-64,
// NEON on arm64, else scalar). The others exist for tests and benchmarks.
enum class FastSimd { kAuto, kScalar, kSse2, kAvx2, kNeon };

// Whether `simd` is compiled in and supported by this CPU (kAuto: always).
[[nodiscard]] bool IsFastSimdAvailable(FastSimd simd);
// The concrete path `simd` runs (kAuto resolves to the best available).
[[nodiscard]] FastSimd ResolveFastSimd(FastSimd simd);

// Detects FAST corners in an 8-bit grayscale view: `data` points at pixel
// (0, 0), rows are `stride` bytes apart (stride >= width), and only pixels
// inside width x height are read, so a sub-view of a larger image needs no
// copy. Corners lie at least 3 px from the view border; views smaller than
// 7x7 have none. `threshold` is clamped to [0, 255]. `out_corners` is
// replaced. Requesting an unavailable `simd` path is a programming error
// (asserted); release builds fall back to kAuto.
void DetectFast(const uint8_t* data, int width, int height, std::ptrdiff_t stride, int threshold,
                bool nonmax_suppression, std::vector<FastCorner>& out_corners,
                FastSimd simd = FastSimd::kAuto);

[[nodiscard]] std::vector<FastCorner> DetectFast(const uint8_t* data, int width, int height,
                                                 std::ptrdiff_t stride, int threshold,
                                                 bool nonmax_suppression,
                                                 FastSimd simd = FastSimd::kAuto);

// Whole image (or Image sub-view: its offset and stride are honoured).
[[nodiscard]] std::vector<FastCorner> DetectFast(const images::TWorkingImage& image, int threshold,
                                                 bool nonmax_suppression,
                                                 FastSimd simd = FastSimd::kAuto);

}  // namespace voyahslam::features

#endif  // VOYAHSLAM_FEATURES_FAST_H
