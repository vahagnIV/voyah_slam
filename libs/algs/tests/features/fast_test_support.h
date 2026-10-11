#ifndef VOYAHSLAM_ALGS_TESTS_FEATURES_FAST_TEST_SUPPORT_H
#define VOYAHSLAM_ALGS_TESTS_FEATURES_FAST_TEST_SUPPORT_H

// Reference for the FAST port: live cv::FAST (TYPE_9_16) on the same view,
// converted to FastCorner. OpenCV is a test-only dependency.

#include <voyahslam/features/fast.h>

#include <opencv2/core.hpp>
#include <opencv2/features.hpp>

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <ostream>
#include <string>
#include <vector>

namespace voyahslam::features {

inline void PrintTo(const FastCorner& corner, std::ostream* out) {
    *out << "(" << corner.x << ", " << corner.y << "; " << corner.response << ")";
}

}  // namespace voyahslam::features

namespace voyahslam::test {

inline std::vector<features::FastCorner> CvFast(const uint8_t* data, int width, int height,
                                                std::ptrdiff_t stride, int threshold,
                                                bool nonmax_suppression) {
    const cv::Mat view(height, width, CV_8U, const_cast<uint8_t*>(data),
                       static_cast<size_t>(stride));
    std::vector<cv::KeyPoint> keypoints;
    cv::FAST(view, keypoints, threshold, nonmax_suppression,
             cv::FastFeatureDetector::TYPE_9_16);
    std::vector<features::FastCorner> corners;
    corners.reserve(keypoints.size());
    for (const auto& kp : keypoints) {
        // cv::FAST emits integer positions and integer scores as floats.
        corners.push_back({static_cast<int>(kp.pt.x), static_cast<int>(kp.pt.y),
                           static_cast<int>(kp.response)});
        if (std::floor(kp.pt.x) != kp.pt.x || std::floor(kp.pt.y) != kp.pt.y ||
            std::floor(kp.response) != kp.response) {
            corners.push_back({-1, -1, -1});  // forces a mismatch
        }
    }
    return corners;
}

// Every SIMD path this machine can run (kScalar first).
inline std::vector<features::FastSimd> RunnableFastPaths() {
    std::vector<features::FastSimd> paths;
    for (auto simd : {features::FastSimd::kScalar, features::FastSimd::kSse2,
                      features::FastSimd::kAvx2, features::FastSimd::kNeon}) {
        if (features::IsFastSimdAvailable(simd)) paths.push_back(simd);
    }
    return paths;
}

inline std::string FastSimdName(features::FastSimd simd) {
    switch (simd) {
        case features::FastSimd::kAuto: return "auto";
        case features::FastSimd::kScalar: return "scalar";
        case features::FastSimd::kSse2: return "sse2";
        case features::FastSimd::kAvx2: return "avx2";
        case features::FastSimd::kNeon: return "neon";
    }
    return "?";
}

}  // namespace voyahslam::test

#endif  // VOYAHSLAM_ALGS_TESTS_FEATURES_FAST_TEST_SUPPORT_H
