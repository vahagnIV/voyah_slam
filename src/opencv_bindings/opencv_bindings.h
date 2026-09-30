#ifndef VOYAHSLAM_SRC_OPENCV_BINDINGS_OPENCV_BINDINGS_H
#define VOYAHSLAM_SRC_OPENCV_BINDINGS_OPENCV_BINDINGS_H

#include <opencv2/opencv.hpp>
#include <voyahslam/images/image.h>

namespace voyahslam {

template<typename TImageType>
struct ImageTypeConverter {};

template<>
struct ImageTypeConverter<images::TImageF32Gray> {
  static cv::Mat ConvertToCvMatUint8Gray(const images::TImageF32Gray &image) {
    cv::Mat cv_32f(image.Height(), image.Width(), CV_32F, (void *)image.GetData());
    cv::Mat result;
    cv_32f.convertTo(result, CV_8U, 255.0);
    return result;
  }
};

template<>
struct ImageTypeConverter<images::TImageU8Gray> {
  static cv::Mat ConvertToCvMatUint8Gray(const images::TWorkingImage &image) {
    return cv::Mat(image.Height(), image.Width(), CV_8U, (void *)image.GetData());
  }
};

} // namespace voyahslam

#endif // VOYAHSLAM_SRC_OPENCV_BINDINGS_OPENCV_BINDINGS_H
