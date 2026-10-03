#include "voyahslam/features/sift_feature_extractor.h"
#include <opencv2/imgproc.hpp>
#include <opencv2/features2d.hpp>

namespace voyahslam::features {

SIFTFeatureExtractor::TFeatures SIFTFeatureExtractor::DetectAndExtract(const images::TWorkingImage &image, size_t number_of_features) const {
    cv::Mat cv_image(image.Height(), image.Width(), CV_8U,
                     const_cast<uint8_t*>(&image.At(0, 0, 0)), image.Stride());

    std::vector<cv::KeyPoint> keypoints;
    cv::Mat descriptors;
    cv::Ptr<cv::SIFT> sift = cv::SIFT::create();
    sift->detectAndCompute(cv_image, cv::Mat(), keypoints, descriptors);

    const size_t num_features = keypoints.size();
    geometry::TPoints2DF points(2, num_features);
    geometry::TRowVectorF strengths(1, num_features);
    geometry::TRowVectorF angles(1, num_features);
    TDescriptors descs(DescriptorSize, num_features);

    for (size_t i = 0; i < std::min(number_of_features, num_features); ++i) {
        points(0, i) = keypoints[i].pt.x;
        points(1, i) = keypoints[i].pt.y;
        strengths(0, i) = keypoints[i].response;
        angles(0, i) = keypoints[i].angle;
    }

    Eigen::Map<const Eigen::Matrix<TDescriptorScalarType, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>
        cv_desc_map(descriptors.ptr<TDescriptorScalarType>(), descriptors.rows, descriptors.cols);
    descs = cv_desc_map.transpose();

    return TFeatures(
        std::move(points),
        std::move(strengths),
        std::move(angles),
        std::move(descs));
}

SIFTFeatureExtractor::TDescriptorScalarType SIFTFeatureExtractor::Distance(
    const TDescriptor &desc1,
    const TDescriptor &desc2) {
    return (desc1 - desc2).norm();
}

} // namespace voyahslam::features
