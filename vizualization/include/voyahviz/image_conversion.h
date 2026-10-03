#ifndef VOYAHVIZ_IMAGE_CONVERSION_H
#define VOYAHVIZ_IMAGE_CONVERSION_H

#include <opencv2/core.hpp>
#include <voyahslam/images/image.h>

namespace voyahviz {

// Wraps an 8-bit image without copying; the result is only valid while
// `image` (its buffer) is alive.
[[nodiscard]] cv::Mat ToCvMat(const voyahslam::images::TImageU8Gray& image);

// Converts a float image with values in [0, 1] to a new 8-bit cv::Mat.
[[nodiscard]] cv::Mat ToCvMat(const voyahslam::images::TImageF32Gray& image);

} // namespace voyahviz

#endif // VOYAHVIZ_IMAGE_CONVERSION_H
