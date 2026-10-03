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
    // Wraps without copying; honours the view's offset and stride.
    return cv::Mat(image.Height(), image.Width(), CV_8U,
                   const_cast<uint8_t *>(&image.At(0, 0, 0)), image.Stride());
  }
};

} // namespace voyahslam

#endif // VOYAHSLAM_SRC_OPENCV_BINDINGS_OPENCV_BINDINGS_H
