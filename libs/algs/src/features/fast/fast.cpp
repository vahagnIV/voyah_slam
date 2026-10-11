// FAST corner detector (9-of-16 segment test with non-maximum suppression).
//
// Ported from OpenCV 5: github.com/opencv/opencv, branch 5.x at commit
// 20e367198c7adde8f1efc0f525256b1e15798024 (version 5.1.0-dev, the OpenCV
// installed for voyahslam's tests; FAST_t and cornerScore are unchanged since
// tag 5.0.0), files modules/features/src/fast.cpp (FAST_t<16>) and
// fast_score.cpp (makeOffsets, cornerScore<16>). The SIMD row kernels are in
// fast_sse2.cpp, fast_avx2.cpp and fast_neon.cpp.
//
// Changes from the original: OpenCV types removed (raw 8-bit view with an
// explicit stride, FastCorner output); only the 16-pixel pattern; universal
// intrinsics replaced by SSE2 / AVX2 / NEON intrinsics in separate files,
// selected at run time; scratch rows kept per thread. The output is
// bit-exact with cv::FAST(img, keypoints, threshold, nonmax_suppression)
// for TYPE_9_16.
//
// ---------------------------------------------------------------------------
// OpenCV licence notice
//
// This file is part of OpenCV project. It is subject to the license terms in
// the LICENSE file found in the top-level directory of this distribution and
// at http://opencv.org/license.html.
//
// Copyright (C) 2000-2022, Intel Corporation, all rights reserved.
// Copyright (C) 2009-2011, Willow Garage Inc., all rights reserved.
// Copyright (C) 2009-2016, NVIDIA Corporation, all rights reserved.
// Copyright (C) 2010-2013, Advanced Micro Devices, Inc., all rights reserved.
// Copyright (C) 2015-2023, OpenCV Foundation, all rights reserved.
// Copyright (C) 2008-2016, Itseez Inc., all rights reserved.
// Copyright (C) 2019-2023, Xperience AI, all rights reserved.
// Copyright (C) 2019-2022, Shenzhen Institute of Artificial Intelligence and
//                          Robotics for Society, all rights reserved.
// Copyright (C) 2022-2023, Southern University of Science And Technology,
//                          all rights reserved.
// Copyright (C) 2023-2025, OpenCV AI, all rights reserved.
// Copyright (C) 2024, Bigvision LLC, all rights reserved.
// Third party copyrights are property of their respective owners.
//
// Licensed under the Apache License, Version 2.0 (the "License"); you may not
// use this file except in compliance with the License. You may obtain a copy
// of the License at http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
// WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
// License for the specific language governing permissions and limitations
// under the License.
//
// ---------------------------------------------------------------------------
// Original notice of the FAST code, contributed to OpenCV by its author:
//
// Copyright (c) 2006, 2008 Edward Rosten
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//
//     *Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//     *Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//     *Neither the name of the University of Cambridge nor the names of
//      its contributors may be used to endorse or promote products derived
//      from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER
// OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
// EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
// PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
// PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
// LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
// NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
// References:
//  * Machine learning for high-speed corner detection,
//    E. Rosten and T. Drummond, ECCV 2006
//  * Faster and better: A machine learning approach to corner detection
//    E. Rosten, R. Porter and T. Drummond, PAMI, 2009

#include "voyahslam/features/fast.h"

#include "fast_kernels.h"

#include <algorithm>
#include <cassert>
#include <cstring>

namespace voyahslam::features {

namespace {

using fast_internal::kK;
using fast_internal::kN;
using fast_internal::kPatternSize;
using fast_internal::RowArgs;
using fast_internal::RowKernel;

// makeOffsets (fast_score.cpp), pattern 16: circle of radius 3, then the
// first 9 offsets repeated so a contiguous arc can wrap.
void MakeOffsets(int pixel[25], int row_stride) {
    static const int kOffsets16[][2] = {
        {0, 3},  {1, 3},   {2, 2},   {3, 1},   {3, 0},  {3, -1}, {2, -2}, {1, -3},
        {0, -3}, {-1, -3}, {-2, -2}, {-3, -1}, {-3, 0}, {-3, 1}, {-2, 2}, {-1, 3}};
    int k = 0;
    for (; k < kPatternSize; k++) pixel[k] = kOffsets16[k][0] + kOffsets16[k][1] * row_stride;
    for (; k < 25; k++) pixel[k] = pixel[k - kPatternSize];
}

// cornerScore<16> (fast_score.cpp), scalar branch: the largest threshold at
// which `ptr` is still a corner. (OpenCV builds with CV_SIMD128 run a vector
// branch that drops the `threshold` seed; for a detected corner both give the
// same value, since its score is at least `threshold`.)
int CornerScore(const uint8_t* ptr, const int pixel[], int threshold) {
    int k, v = ptr[0];
    short d[kN];
    for (k = 0; k < kN; k++) d[k] = (short)(v - ptr[pixel[k]]);

    int a0 = threshold;
    for (k = 0; k < 16; k += 2) {
        int a = std::min((int)d[k + 1], (int)d[k + 2]);
        a = std::min(a, (int)d[k + 3]);
        if (a <= a0) continue;
        a = std::min(a, (int)d[k + 4]);
        a = std::min(a, (int)d[k + 5]);
        a = std::min(a, (int)d[k + 6]);
        a = std::min(a, (int)d[k + 7]);
        a = std::min(a, (int)d[k + 8]);
        a0 = std::max(a0, std::min(a, (int)d[k]));
        a0 = std::max(a0, std::min(a, (int)d[k + 9]));
    }

    int b0 = -a0;
    for (k = 0; k < 16; k += 2) {
        int b = std::max((int)d[k + 1], (int)d[k + 2]);
        b = std::max(b, (int)d[k + 3]);
        b = std::max(b, (int)d[k + 4]);
        b = std::max(b, (int)d[k + 5]);
        if (b >= b0) continue;
        b = std::max(b, (int)d[k + 6]);
        b = std::max(b, (int)d[k + 7]);
        b = std::max(b, (int)d[k + 8]);

        b0 = std::min(b0, std::max(b, (int)d[k]));
        b0 = std::min(b0, std::max(b, (int)d[k + 9]));
    }

    return -b0 - 1;
}

bool CpuHasAvx2() {
#if defined(VOYAHSLAM_FAST_AVX2)
    static const bool has_avx2 = [] {
        __builtin_cpu_init();
        return __builtin_cpu_supports("avx2") != 0;
    }();
    return has_avx2;
#else
    return false;
#endif
}

// Per-thread scratch rows (three score rows, three corner-column rows), grown
// on demand, so per-cell calls in the ORB extractor do not allocate.
struct Scratch {
    std::vector<uint8_t> scores;
    std::vector<int> corner_positions;
};

Scratch& ThreadScratch() {
    thread_local Scratch scratch;
    return scratch;
}

}  // namespace

bool IsFastSimdAvailable(FastSimd simd) {
    switch (simd) {
        case FastSimd::kAuto:
        case FastSimd::kScalar:
            return true;
        case FastSimd::kSse2:
#if defined(VOYAHSLAM_FAST_SSE2)
            return true;
#else
            return false;
#endif
        case FastSimd::kAvx2:
            return CpuHasAvx2();
        case FastSimd::kNeon:
#if defined(VOYAHSLAM_FAST_NEON)
            return true;
#else
            return false;
#endif
    }
    return false;
}

FastSimd ResolveFastSimd(FastSimd simd) {
    if (simd != FastSimd::kAuto) return simd;
    if (IsFastSimdAvailable(FastSimd::kAvx2)) return FastSimd::kAvx2;
    if (IsFastSimdAvailable(FastSimd::kSse2)) return FastSimd::kSse2;
    if (IsFastSimdAvailable(FastSimd::kNeon)) return FastSimd::kNeon;
    return FastSimd::kScalar;
}

// FAST_t<16> (fast.cpp).
void DetectFast(const uint8_t* data, int width, int height, std::ptrdiff_t stride, int threshold,
                bool nonmax_suppression, std::vector<FastCorner>& out_corners, FastSimd simd) {
    out_corners.clear();
    if (width <= 0 || height <= 0) return;
    assert(stride >= width);

    assert(IsFastSimdAvailable(simd));
    if (!IsFastSimdAvailable(simd)) simd = FastSimd::kAuto;
    simd = ResolveFastSimd(simd);

    // Up to two vector kernels, wide first (AVX2 32 px, then 16 px), as
    // OpenCV runs its AVX2 dispatch before the 128-bit loop.
    RowKernel kernels[2] = {nullptr, nullptr};
    switch (simd) {
#if defined(VOYAHSLAM_FAST_AVX2)
        case FastSimd::kAvx2:
            kernels[0] = fast_internal::ProcessRowAvx2;
            kernels[1] = fast_internal::ProcessRowSse2;
            break;
#endif
#if defined(VOYAHSLAM_FAST_SSE2)
        case FastSimd::kSse2:
            kernels[0] = fast_internal::ProcessRowSse2;
            break;
#endif
#if defined(VOYAHSLAM_FAST_NEON)
        case FastSimd::kNeon:
            kernels[0] = fast_internal::ProcessRowNeon;
            break;
#endif
        default:
            break;
    }

    int i, j, k, pixel[25];
    MakeOffsets(pixel, static_cast<int>(stride));

    threshold = std::min(std::max(threshold, 0), 255);
    const RowArgs args{pixel, width, threshold, nonmax_suppression};

    uint8_t threshold_tab[512];
    for (i = -255; i <= 255; i++)
        threshold_tab[i + 255] = (uint8_t)(i < -threshold ? 1 : i > threshold ? 2 : 0);

    Scratch& scratch = ThreadScratch();
    const size_t score_row = static_cast<size_t>(width);
    const size_t corner_row = static_cast<size_t>(width) + 1;
    if (scratch.scores.size() < 3 * score_row) scratch.scores.resize(3 * score_row);
    if (scratch.corner_positions.size() < 3 * corner_row)
        scratch.corner_positions.resize(3 * corner_row);
    uint8_t* buf[3];
    int* cpbuf[3];
    for (unsigned idx = 0; idx < 3; ++idx) {
        buf[idx] = scratch.scores.data() + idx * score_row;
        cpbuf[idx] = scratch.corner_positions.data() + idx * corner_row;
        std::memset(buf[idx], 0, score_row);
    }

    for (i = 3; i < height - 2; i++) {
        const uint8_t* ptr = data + i * stride + 3;
        uint8_t* curr = buf[(i - 3) % 3];
        int* cornerpos = cpbuf[(i - 3) % 3] + 1;  // cornerpos[-1] holds the count
        std::memset(curr, 0, score_row);
        int ncorners = 0;

        if (i < height - 3) {
            j = 3;
            for (RowKernel kernel : kernels) {
                if (kernel) kernel(j, ptr, curr, cornerpos, ncorners, args);
            }
            for (; j < width - 3; j++, ptr++) {
                int v = ptr[0];
                const uint8_t* tab = &threshold_tab[0] - v + 255;
                int d = tab[ptr[pixel[0]]] | tab[ptr[pixel[8]]];

                if (d == 0) continue;

                d &= tab[ptr[pixel[2]]] | tab[ptr[pixel[10]]];
                d &= tab[ptr[pixel[4]]] | tab[ptr[pixel[12]]];
                d &= tab[ptr[pixel[6]]] | tab[ptr[pixel[14]]];

                if (d == 0) continue;

                d &= tab[ptr[pixel[1]]] | tab[ptr[pixel[9]]];
                d &= tab[ptr[pixel[3]]] | tab[ptr[pixel[11]]];
                d &= tab[ptr[pixel[5]]] | tab[ptr[pixel[13]]];
                d &= tab[ptr[pixel[7]]] | tab[ptr[pixel[15]]];

                if (d & 1) {
                    int vt = v - threshold, count = 0;

                    for (k = 0; k < kN; k++) {
                        int x = ptr[pixel[k]];
                        if (x < vt) {
                            if (++count > kK) {
                                cornerpos[ncorners++] = j;
                                if (nonmax_suppression)
                                    curr[j] = (uint8_t)CornerScore(ptr, pixel, threshold);
                                break;
                            }
                        } else
                            count = 0;
                    }
                }

                if (d & 2) {
                    int vt = v + threshold, count = 0;

                    for (k = 0; k < kN; k++) {
                        int x = ptr[pixel[k]];
                        if (x > vt) {
                            if (++count > kK) {
                                cornerpos[ncorners++] = j;
                                if (nonmax_suppression)
                                    curr[j] = (uint8_t)CornerScore(ptr, pixel, threshold);
                                break;
                            }
                        } else
                            count = 0;
                    }
                }
            }
        }

        cornerpos[-1] = ncorners;

        if (i == 3) continue;

        const uint8_t* prev = buf[(i - 4 + 3) % 3];
        const uint8_t* pprev = buf[(i - 5 + 3) % 3];
        cornerpos = cpbuf[(i - 4 + 3) % 3] + 1;
        ncorners = cornerpos[-1];

        for (k = 0; k < ncorners; k++) {
            j = cornerpos[k];
            int score = prev[j];
            if (!nonmax_suppression ||
                (score > prev[j + 1] && score > prev[j - 1] && score > pprev[j - 1] &&
                 score > pprev[j] && score > pprev[j + 1] && score > curr[j - 1] &&
                 score > curr[j] && score > curr[j + 1])) {
                out_corners.push_back({j, i - 1, score});
            }
        }
    }
}

std::vector<FastCorner> DetectFast(const uint8_t* data, int width, int height,
                                   std::ptrdiff_t stride, int threshold, bool nonmax_suppression,
                                   FastSimd simd) {
    std::vector<FastCorner> corners;
    DetectFast(data, width, height, stride, threshold, nonmax_suppression, corners, simd);
    return corners;
}

std::vector<FastCorner> DetectFast(const images::TWorkingImage& image, int threshold,
                                   bool nonmax_suppression, FastSimd simd) {
    if (image.Width() == 0 || image.Height() == 0) return {};
    return DetectFast(&image.At(0, 0, 0), static_cast<int>(image.Width()),
                      static_cast<int>(image.Height()), image.Stride(), threshold,
                      nonmax_suppression, simd);
}

}  // namespace voyahslam::features
