#include "voyahslam/features/sift_feature_extractor.h"
#include <opencv2/imgproc.hpp>
#include <opencv2/features2d.hpp>
#include <algorithm>
#include <cmath>
#include <numbers>
#include <numeric>
#include <vector>

namespace voyahslam::features {

SIFTFeatureExtractor::TFeatures SIFTFeatureExtractor::DetectAndExtract(const images::TWorkingImage &image, size_t number_of_features) const {
    cv::Mat cv_image(image.Height(), image.Width(), CV_8U,
                     const_cast<uint8_t*>(&image.At(0, 0, 0)), image.Stride());

    std::vector<cv::KeyPoint> keypoints;
    cv::Mat descriptors;
    cv::Ptr<cv::SIFT> sift = cv::SIFT::create();
    sift->detectAndCompute(cv_image, cv::Mat(), keypoints, descriptors);

    // Keep the strongest `number_of_features` keypoints, strongest first.
    std::vector<size_t> order(keypoints.size());
    std::iota(order.begin(), order.end(), size_t{0});
    std::stable_sort(order.begin(), order.end(), [&keypoints](size_t a, size_t b) {
        return keypoints[a].response > keypoints[b].response;
    });
    const size_t num_features = std::min(number_of_features, keypoints.size());
    order.resize(num_features);

    geometry::TPoints2DF points(2, num_features);
    geometry::TRowVectorF strengths(1, num_features);
    geometry::TRowVectorF angles(1, num_features);
    TDescriptors descs(DescriptorSize, num_features);
    TLevels levels(num_features);

    constexpr double kDegToRad = std::numbers::pi / 180.0;
    for (size_t i = 0; i < num_features; ++i) {
        const cv::KeyPoint& keypoint = keypoints[order[i]];
        points(0, i) = keypoint.pt.x;
        points(1, i) = keypoint.pt.y;
        strengths(0, i) = keypoint.response;
        // OpenCV angles are degrees in [0, 360); guard the upper bound against rounding.
        double radians = keypoint.angle * kDegToRad;
        if (radians < 0.0 || radians >= 2.0 * std::numbers::pi) radians = 0.0;
        angles(0, i) = radians;
        levels(i) = LevelFromOctave(keypoint.octave);
        for (int d = 0; d < DescriptorSize; ++d)
            descs(d, i) = descriptors.at<TDescriptorScalarType>(static_cast<int>(order[i]), d);
    }

    return TFeatures(
        std::move(points),
        std::move(strengths),
        std::move(angles),
        std::move(descs),
        std::move(levels));
}

uint8_t SIFTFeatureExtractor::LevelFromOctave(int packed_octave) {
    const int octave = static_cast<int8_t>(packed_octave & 0xFF);
    const int layer = (packed_octave >> 8) & 0xFF;
    return static_cast<uint8_t>((octave + 1) * 3 + layer);
}

double SIFTFeatureExtractor::ScaleFactor(uint8_t level) const {
    return std::exp2(static_cast<double>(level) / 3.0) / 2.0;
}

SIFTFeatureExtractor::TDescriptorScalarType SIFTFeatureExtractor::Distance(
    const TDescriptor &desc1,
    const TDescriptor &desc2) {
    return (desc1 - desc2).norm();
}

} // namespace voyahslam::features
