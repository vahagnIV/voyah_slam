// FAST row kernel, NEON (arm64 baseline), 16 pixels per block. Built only
// for arm64 targets (libs/algs/CMakeLists.txt).
//
// Ported from OpenCV 5 modules/features/src/fast.cpp, the CV_SIMD128 block
// of FAST_t<16>, as OpenCV's universal intrinsics lower it to NEON; commit
// 20e367198c (5.1.0-dev). Copyright (C) 2000-2025 the OpenCV authors (see
// fast.cpp for the full list); licensed under the Apache License, Version
// 2.0 (http://www.apache.org/licenses/LICENSE-2.0). Original FAST code
// Copyright (c) 2006, 2008 Edward Rosten, BSD licence (full text in
// fast.cpp). Changes: OpenCV types and universal intrinsics removed.
//
// Not yet run on arm64 hardware (docs/TODO.md); verified bit-exact only
// through SIMDe's NEON emulation on x86-64.

#include "fast_kernels.h"

#include <arm_neon.h>

namespace voyahslam::features::fast_internal {

namespace {

// One bit per lane (lane 0 = bit 0) from an all-ones / all-zeros mask, as
// OpenCV's v_signmask.
inline unsigned int SignMask(uint8x16_t mask) {
    static const uint8_t kBits[16] = {1, 2, 4, 8, 16, 32, 64, 128, 1, 2, 4, 8, 16, 32, 64, 128};
    const uint8x16_t bits = vandq_u8(mask, vld1q_u8(kBits));
    return static_cast<unsigned int>(vaddv_u8(vget_low_u8(bits))) |
           (static_cast<unsigned int>(vaddv_u8(vget_high_u8(bits))) << 8);
}

}  // namespace

void ProcessRowNeon(int& j, const uint8_t*& ptr, uint8_t* curr, int* cornerpos, int& ncorners,
                    const RowArgs& args) {
    const int* pixel = args.pixel;
    const int quarter = kPatternSize / 4;
    const uint8x16_t delta = vdupq_n_u8(0x80);
    const uint8x16_t t = vdupq_n_u8((uint8_t)args.threshold);
    const uint8x16_t k16 = vdupq_n_u8((uint8_t)kK);

    for (; j < args.width - 16 - 3; j += 16, ptr += 16) {
        const uint8x16_t v = vld1q_u8(ptr);
        // Saturating add / sub, then xor 0x80 maps u8 order to s8 order.
        const int8x16_t v0 = vreinterpretq_s8_u8(veorq_u8(vqaddq_u8(v, t), delta));
        const int8x16_t v1 = vreinterpretq_s8_u8(veorq_u8(vqsubq_u8(v, t), delta));

        const int8x16_t x0 = vreinterpretq_s8_u8(vsubq_u8(vld1q_u8(ptr + pixel[0]), delta));
        const int8x16_t x1 = vreinterpretq_s8_u8(vsubq_u8(vld1q_u8(ptr + pixel[quarter]), delta));
        const int8x16_t x2 = vreinterpretq_s8_u8(vsubq_u8(vld1q_u8(ptr + pixel[2 * quarter]), delta));
        const int8x16_t x3 = vreinterpretq_s8_u8(vsubq_u8(vld1q_u8(ptr + pixel[3 * quarter]), delta));

        uint8x16_t m0, m1;
        m0 = vandq_u8(vcltq_s8(v0, x0), vcltq_s8(v0, x1));
        m1 = vandq_u8(vcltq_s8(x0, v1), vcltq_s8(x1, v1));
        m0 = vorrq_u8(m0, vandq_u8(vcltq_s8(v0, x1), vcltq_s8(v0, x2)));
        m1 = vorrq_u8(m1, vandq_u8(vcltq_s8(x1, v1), vcltq_s8(x2, v1)));
        m0 = vorrq_u8(m0, vandq_u8(vcltq_s8(v0, x2), vcltq_s8(v0, x3)));
        m1 = vorrq_u8(m1, vandq_u8(vcltq_s8(x2, v1), vcltq_s8(x3, v1)));
        m0 = vorrq_u8(m0, vandq_u8(vcltq_s8(v0, x3), vcltq_s8(v0, x0)));
        m1 = vorrq_u8(m1, vandq_u8(vcltq_s8(x3, v1), vcltq_s8(x0, v1)));
        m0 = vorrq_u8(m0, m1);

        if (vmaxvq_u8(m0) == 0) continue;
        if (vmaxv_u8(vget_low_u8(m0)) == 0) {
            // Only the upper half has candidates: restart the block there.
            j -= 8;
            ptr -= 8;
            continue;
        }

        uint8x16_t c0 = vdupq_n_u8(0);
        uint8x16_t c1 = vdupq_n_u8(0);
        uint8x16_t max0 = vdupq_n_u8(0);
        uint8x16_t max1 = vdupq_n_u8(0);
        for (int k = 0; k < kN; k++) {
            const int8x16_t x = vreinterpretq_s8_u8(veorq_u8(vld1q_u8(ptr + pixel[k]), delta));
            m0 = vcltq_s8(v0, x);
            m1 = vcltq_s8(x, v1);

            // Run lengths of brighter (c0) / darker (c1) pixels; reset on a miss.
            c0 = vandq_u8(vsubq_u8(c0, m0), m0);
            c1 = vandq_u8(vsubq_u8(c1, m1), m1);

            max0 = vmaxq_u8(max0, c0);
            max1 = vmaxq_u8(max1, c1);
        }

        unsigned int m = SignMask(vcgtq_u8(vmaxq_u8(max0, max1), k16));

        for (int k = 0; m > 0 && k < 16; k++, m >>= 1) {
            if (m & 1) {
                cornerpos[ncorners++] = j + k;
                if (args.nonmax_suppression) {
                    short d[25];
                    for (int q = 0; q < 25; q++) d[q] = (short)(ptr[k] - ptr[k + pixel[q]]);

                    // Lane l of a0/b0: min/max over the 9-arc starting at l;
                    // a1/b1: arcs starting at 8 + l.
                    int16x8_t a0, b0, a1, b1;
                    a0 = b0 = a1 = b1 = vld1q_s16(d + 8);
                    for (int shift = 0; shift < 8; ++shift) {
                        int16x8_t v_nms = vld1q_s16(d + shift);
                        a0 = vminq_s16(a0, v_nms);
                        b0 = vmaxq_s16(b0, v_nms);
                        v_nms = vld1q_s16(d + 9 + shift);
                        a1 = vminq_s16(a1, v_nms);
                        b1 = vmaxq_s16(b1, v_nms);
                    }
                    const int16x8_t best = vmaxq_s16(
                        vmaxq_s16(a0, a1), vqsubq_s16(vdupq_n_s16(0), vminq_s16(b0, b1)));
                    curr[j + k] = (uint8_t)(vmaxvq_s16(best) - 1);
                }
            }
        }
    }
}

}  // namespace voyahslam::features::fast_internal
