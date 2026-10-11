// FAST row kernel, AVX2, 32 pixels per block. Compiled with -mavx2 and run
// only when the CPU reports AVX2 (fast.cpp).
//
// Ported from OpenCV 5 modules/features/src/fast.avx2.cpp
// (FAST_t_patternSize16_AVX2_Impl::process), commit 20e367198c (5.1.0-dev).
// Copyright (C) 2000-2025 the OpenCV authors (see fast.cpp for the full
// list); licensed under the Apache License, Version 2.0
// (http://www.apache.org/licenses/LICENSE-2.0). Original FAST code
// Copyright (c) 2006, 2008 Edward Rosten, BSD licence (full text in
// fast.cpp). Changes: OpenCV types and universal intrinsics removed (the
// 128-bit score uses SSE2), no virtual dispatch.

#include "fast_kernels.h"

#include <immintrin.h>

namespace voyahslam::features::fast_internal {

namespace {

inline short ReduceMaxS16(__m128i v) {
    v = _mm_max_epi16(v, _mm_srli_si128(v, 8));
    v = _mm_max_epi16(v, _mm_srli_si128(v, 4));
    v = _mm_max_epi16(v, _mm_srli_si128(v, 2));
    return static_cast<short>(_mm_cvtsi128_si32(v));
}

inline __m128i Load128(const short* p) { return _mm_loadu_si128(reinterpret_cast<const __m128i*>(p)); }

inline __m256i Load256(const uint8_t* p) {
    return _mm256_loadu_si256(reinterpret_cast<const __m256i*>(p));
}

// The corner score (cornerScore<16>, CV_SIMD128 branch): best 9-arc minimum
// of d (darker) or negated maximum (brighter), minus one.
inline int Score(const uint8_t* p, const int* pixel) {
    short d[25];
    for (int q = 0; q < 25; q++) d[q] = (short)(p[0] - p[pixel[q]]);
    __m128i q0 = _mm_set1_epi16(-1000), q1 = _mm_set1_epi16(1000);
    for (int q = 0; q < 16; q += 8) {
        __m128i v0 = Load128(d + q + 1);
        __m128i v1 = Load128(d + q + 2);
        __m128i a = _mm_min_epi16(v0, v1);
        __m128i b = _mm_max_epi16(v0, v1);
        for (int r = 3; r <= 8; ++r) {
            v0 = Load128(d + q + r);
            a = _mm_min_epi16(a, v0);
            b = _mm_max_epi16(b, v0);
        }
        v0 = Load128(d + q);
        q0 = _mm_max_epi16(q0, _mm_min_epi16(a, v0));
        q1 = _mm_min_epi16(q1, _mm_max_epi16(b, v0));
        v0 = Load128(d + q + 9);
        q0 = _mm_max_epi16(q0, _mm_min_epi16(a, v0));
        q1 = _mm_min_epi16(q1, _mm_max_epi16(b, v0));
    }
    q0 = _mm_max_epi16(q0, _mm_subs_epi16(_mm_setzero_si128(), q1));
    return ReduceMaxS16(q0) - 1;
}

}  // namespace

void ProcessRowAvx2(int& j, const uint8_t*& ptr, uint8_t* curr, int* cornerpos, int& ncorners,
                    const RowArgs& args) {
    const int* pixel = args.pixel;
    const __m256i delta256 = _mm256_set1_epi8((char)(-128));
    const __m256i k16_256 = _mm256_set1_epi8((char)kK);
    const __m256i t256 = _mm256_set1_epi8((char)args.threshold);

    for (; j < args.width - 32 - 3; j += 32, ptr += 32) {
        __m256i m0, m1;
        __m256i v0 = Load256(ptr);

        __m256i v1 = _mm256_xor_si256(_mm256_subs_epu8(v0, t256), delta256);
        v0 = _mm256_xor_si256(_mm256_adds_epu8(v0, t256), delta256);

        __m256i x0 = _mm256_sub_epi8(Load256(ptr + pixel[0]), delta256);
        __m256i x1 = _mm256_sub_epi8(Load256(ptr + pixel[4]), delta256);
        __m256i x2 = _mm256_sub_epi8(Load256(ptr + pixel[8]), delta256);
        __m256i x3 = _mm256_sub_epi8(Load256(ptr + pixel[12]), delta256);

        m0 = _mm256_and_si256(_mm256_cmpgt_epi8(x0, v0), _mm256_cmpgt_epi8(x1, v0));
        m1 = _mm256_and_si256(_mm256_cmpgt_epi8(v1, x0), _mm256_cmpgt_epi8(v1, x1));
        m0 = _mm256_or_si256(m0, _mm256_and_si256(_mm256_cmpgt_epi8(x1, v0), _mm256_cmpgt_epi8(x2, v0)));
        m1 = _mm256_or_si256(m1, _mm256_and_si256(_mm256_cmpgt_epi8(v1, x1), _mm256_cmpgt_epi8(v1, x2)));
        m0 = _mm256_or_si256(m0, _mm256_and_si256(_mm256_cmpgt_epi8(x2, v0), _mm256_cmpgt_epi8(x3, v0)));
        m1 = _mm256_or_si256(m1, _mm256_and_si256(_mm256_cmpgt_epi8(v1, x2), _mm256_cmpgt_epi8(v1, x3)));
        m0 = _mm256_or_si256(m0, _mm256_and_si256(_mm256_cmpgt_epi8(x3, v0), _mm256_cmpgt_epi8(x0, v0)));
        m1 = _mm256_or_si256(m1, _mm256_and_si256(_mm256_cmpgt_epi8(v1, x3), _mm256_cmpgt_epi8(v1, x0)));
        m0 = _mm256_or_si256(m0, m1);

        unsigned int mask = static_cast<unsigned int>(_mm256_movemask_epi8(m0));
        if (mask == 0) continue;
        if ((mask & 0xffff) == 0) {
            // Only the upper half has candidates: restart the block there.
            j -= 16;
            ptr -= 16;
            continue;
        }

        __m256i c0 = _mm256_setzero_si256(), c1 = c0, max0 = c0, max1 = c0;
        for (int k = 0; k < kN; k++) {
            __m256i x = _mm256_xor_si256(Load256(ptr + pixel[k]), delta256);
            m0 = _mm256_cmpgt_epi8(x, v0);
            m1 = _mm256_cmpgt_epi8(v1, x);

            c0 = _mm256_and_si256(_mm256_sub_epi8(c0, m0), m0);
            c1 = _mm256_and_si256(_mm256_sub_epi8(c1, m1), m1);

            max0 = _mm256_max_epu8(max0, c0);
            max1 = _mm256_max_epu8(max1, c1);
        }

        max0 = _mm256_max_epu8(max0, max1);
        unsigned int m = static_cast<unsigned int>(_mm256_movemask_epi8(_mm256_cmpgt_epi8(max0, k16_256)));

        for (int k = 0; m > 0 && k < 32; k++, m >>= 1) {
            if (m & 1) {
                cornerpos[ncorners++] = j + k;
                if (args.nonmax_suppression) curr[j + k] = (uint8_t)Score(ptr + k, pixel);
            }
        }
    }
    _mm256_zeroupper();
}

}  // namespace voyahslam::features::fast_internal
