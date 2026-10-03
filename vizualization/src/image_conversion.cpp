#include <voyahviz/image_conversion.h>

namespace voyahviz {

cv::Mat ToCvMat(const voyahslam::images::TImageU8Gray& image) {
    return cv::Mat(static_cast<int>(image.Height()), static_cast<int>(image.Width()), CV_8UC1,
                   const_cast<uint8_t*>(&image.At(0, 0, 0)), image.Stride());
}

cv::Mat ToCvMat(const voyahslam::images::TImageF32Gray& image) {
    cv::Mat as_float(static_cast<int>(image.Height()), static_cast<int>(image.Width()), CV_32FC1,
                     const_cast<float*>(&image.At(0, 0, 0)), image.Stride() * sizeof(float));
    cv::Mat result;
    as_float.convertTo(result, CV_8U, 255.0);
    return result;
}

} // namespace voyahviz
