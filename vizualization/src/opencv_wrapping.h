#ifndef VOYAHVIZ_SRC_OPENCV_WRAPPING_H
#define VOYAHVIZ_SRC_OPENCV_WRAPPING_H

// Private: OpenCV must not appear in voyahviz's public headers.

#include <opencv2/core.hpp>
#include <voyahslam/images/image.h>

namespace voyahviz::internal {

// Zero-copy views of our images (interleaved channels, stride in pixels). The
// result is only valid while the image's buffer is alive.
template <typename T, int C>
cv::Mat WrapAsCvMat(const voyahslam::images::Image<T, C>& image) {
    static_assert(std::is_same_v<T, uint8_t>, "only 8-bit images are wrapped");
    return cv::Mat(static_cast<int>(image.Height()), static_cast<int>(image.Width()), CV_8UC(C),
                   const_cast<T*>(&image.At(0, 0, 0)), static_cast<size_t>(image.Stride()) * C * sizeof(T));
}

} // namespace voyahviz::internal

#endif // VOYAHVIZ_SRC_OPENCV_WRAPPING_H
