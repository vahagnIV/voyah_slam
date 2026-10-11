// Internal: FAST row kernels shared by the scalar driver (fast.cpp) and the
// SIMD translation units. Licence: see fast.cpp (OpenCV, Apache-2.0; Edward
// Rosten, BSD).

#ifndef VOYAHSLAM_SRC_FEATURES_FAST_FAST_KERNELS_H
#define VOYAHSLAM_SRC_FEATURES_FAST_FAST_KERNELS_H

#include <cstdint>

namespace voyahslam::features::fast_internal {

// 9-of-16: a corner needs more than K = 8 contiguous circle pixels all
// brighter or all darker; the circle is walked N = 25 steps (16 + 9) so an
// arc wrapping past pixel 15 is counted.
constexpr int kPatternSize = 16;
constexpr int kK = kPatternSize / 2;
constexpr int kN = kPatternSize + kK + 1;

struct RowArgs {
    const int* pixel;  // 25 circle offsets (makeOffsets), in bytes
    int width;         // view width
    int threshold;     // clamped to [0, 255]
    bool nonmax_suppression;
};

// A SIMD row kernel: starting at column `j` (pointer `ptr` at that pixel),
// tests whole vector blocks while one fits before `width - 3`, appends corner
// columns to `cornerpos` (ascending) and, with non-maximum suppression, writes
// their scores to `curr`. Leaves `j`/`ptr` at the first column it did not
// test; the caller finishes the row with the scalar loop.
using RowKernel = void (*)(int& j, const uint8_t*& ptr, uint8_t* curr, int* cornerpos,
                           int& ncorners, const RowArgs& args);

// x86-64 (fast_sse2.cpp: 16 px per block; fast_avx2.cpp: 32 px, needs AVX2).
void ProcessRowSse2(int& j, const uint8_t*& ptr, uint8_t* curr, int* cornerpos, int& ncorners,
                    const RowArgs& args);
void ProcessRowAvx2(int& j, const uint8_t*& ptr, uint8_t* curr, int* cornerpos, int& ncorners,
                    const RowArgs& args);
// arm64 (fast_neon.cpp: 16 px per block).
void ProcessRowNeon(int& j, const uint8_t*& ptr, uint8_t* curr, int* cornerpos, int& ncorners,
                    const RowArgs& args);

}  // namespace voyahslam::features::fast_internal

#endif  // VOYAHSLAM_SRC_FEATURES_FAST_FAST_KERNELS_H
