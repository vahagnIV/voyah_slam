// FAST row kernel, SSE2 (x86-64 baseline), 16 pixels per block.
//
// Ported from OpenCV 5 modules/features/src/fast.cpp, the CV_SIMD128 block
// of FAST_t<16> (universal intrinsics written out as SSE2), commit
// 20e367198c (5.1.0-dev). Copyright (C) 2000-2025 the OpenCV authors (see
// fast.cpp for the full list); licensed under the Apache License, Version
// 2.0 (http://www.apache.org/licenses/LICENSE-2.0). Original FAST code
// Copyright (c) 2006, 2008 Edward Rosten, BSD licence (full text in
// fast.cpp). Changes: OpenCV types and universal intrinsics removed.

#include "fast_kernels.h"

#include <emmintrin.h>

namespace voyahslam::features::fast_internal {

namespace {

inline short ReduceMaxS16(__m128i v) {
    v = _mm_max_epi16(v, _mm_srli_si128(v, 8));
    v = _mm_max_epi16(v, _mm_srli_si128(v, 4));
    v = _mm_max_epi16(v, _mm_srli_si128(v, 2));
    return static_cast<short>(_mm_cvtsi128_si32(v));
}

inline __m128i Load(const void* p) { return _mm_loadu_si128(static_cast<const __m128i*>(p)); }

}  // namespace

void ProcessRowSse2(int& j, const uint8_t*& ptr, uint8_t* curr, int* cornerpos, int& ncorners,
                    const RowArgs& args) {
    const int* pixel = args.pixel;
    const int quarter = kPatternSize / 4;
    const __m128i delta = _mm_set1_epi8((char)0x80);
    const __m128i t = _mm_set1_epi8((char)args.threshold);
    const __m128i k16 = _mm_set1_epi8((char)kK);

    for (; j < args.width - 16 - 3; j += 16, ptr += 16) {
        const __m128i v = Load(ptr);
        // v_add / v_sub on u8 saturate; xor 0x80 maps u8 order to s8 order.
        const __m128i v0 = _mm_xor_si128(_mm_adds_epu8(v, t), delta);
        const __m128i v1 = _mm_xor_si128(_mm_subs_epu8(v, t), delta);

        const __m128i x0 = _mm_sub_epi8(Load(ptr + pixel[0]), delta);
        const __m128i x1 = _mm_sub_epi8(Load(ptr + pixel[quarter]), delta);
        const __m128i x2 = _mm_sub_epi8(Load(ptr + pixel[2 * quarter]), delta);
        const __m128i x3 = _mm_sub_epi8(Load(ptr + pixel[3 * quarter]), delta);

        __m128i m0, m1;
        m0 = _mm_and_si128(_mm_cmplt_epi8(v0, x0), _mm_cmplt_epi8(v0, x1));
        m1 = _mm_and_si128(_mm_cmplt_epi8(x0, v1), _mm_cmplt_epi8(x1, v1));
        m0 = _mm_or_si128(m0, _mm_and_si128(_mm_cmplt_epi8(v0, x1), _mm_cmplt_epi8(v0, x2)));
        m1 = _mm_or_si128(m1, _mm_and_si128(_mm_cmplt_epi8(x1, v1), _mm_cmplt_epi8(x2, v1)));
        m0 = _mm_or_si128(m0, _mm_and_si128(_mm_cmplt_epi8(v0, x2), _mm_cmplt_epi8(v0, x3)));
        m1 = _mm_or_si128(m1, _mm_and_si128(_mm_cmplt_epi8(x2, v1), _mm_cmplt_epi8(x3, v1)));
        m0 = _mm_or_si128(m0, _mm_and_si128(_mm_cmplt_epi8(v0, x3), _mm_cmplt_epi8(v0, x0)));
        m1 = _mm_or_si128(m1, _mm_and_si128(_mm_cmplt_epi8(x3, v1), _mm_cmplt_epi8(x0, v1)));
        m0 = _mm_or_si128(m0, m1);

        const int candidates = _mm_movemask_epi8(m0);
        if (candidates == 0) continue;
        if ((candidates & 0xFF) == 0) {
            // Only the upper half has candidates: restart the block there.
            j -= 8;
            ptr -= 8;
            continue;
        }

        __m128i c0 = _mm_setzero_si128();
        __m128i c1 = _mm_setzero_si128();
        __m128i max0 = _mm_setzero_si128();
        __m128i max1 = _mm_setzero_si128();
        for (int k = 0; k < kN; k++) {
            const __m128i x = _mm_xor_si128(Load(ptr + pixel[k]), delta);
            m0 = _mm_cmplt_epi8(v0, x);
            m1 = _mm_cmplt_epi8(x, v1);

            // Run lengths of brighter (c0) / darker (c1) pixels; reset on a miss.
            c0 = _mm_and_si128(_mm_sub_epi8(c0, m0), m0);
            c1 = _mm_and_si128(_mm_sub_epi8(c1, m1), m1);

            max0 = _mm_max_epu8(max0, c0);
            max1 = _mm_max_epu8(max1, c1);
        }

        max0 = _mm_cmpgt_epi8(_mm_max_epu8(max0, max1), k16);  // runs <= 25: signed is fine
        unsigned int m = static_cast<unsigned int>(_mm_movemask_epi8(max0));

        for (int k = 0; m > 0 && k < 16; k++, m >>= 1) {
            if (m & 1) {
                cornerpos[ncorners++] = j + k;
                if (args.nonmax_suppression) {
                    short d[25];
                    for (int q = 0; q < 25; q++) d[q] = (short)(ptr[k] - ptr[k + pixel[q]]);

                    // Lane l of a0/b0: min/max over the 9-arc starting at l;
                    // a1/b1: arcs starting at 8 + l.
                    __m128i a0, b0, a1, b1;
                    a0 = b0 = a1 = b1 = Load(d + 8);
                    for (int shift = 0; shift < 8; ++shift) {
                        __m128i v_nms = Load(d + shift);
                        a0 = _mm_min_epi16(a0, v_nms);
                        b0 = _mm_max_epi16(b0, v_nms);
                        v_nms = Load(d + 9 + shift);
                        a1 = _mm_min_epi16(a1, v_nms);
                        b1 = _mm_max_epi16(b1, v_nms);
                    }
                    const __m128i best = _mm_max_epi16(
                        _mm_max_epi16(a0, a1),
                        _mm_subs_epi16(_mm_setzero_si128(), _mm_min_epi16(b0, b1)));
                    curr[j + k] = (uint8_t)(ReduceMaxS16(best) - 1);
                }
            }
        }
    }
}

}  // namespace voyahslam::features::fast_internal
