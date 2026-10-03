#ifndef VOYAHVIZ_COLOR_IMAGE_H
#define VOYAHVIZ_COLOR_IMAGE_H

#include <cstdint>
#include <voyahslam/images/image.h>

namespace voyahviz {

// 8-bit, 3-channel image, channels interleaved in B, G, R order (the order
// display backends expect).
using TColorImage = voyahslam::images::Image<uint8_t, 3>;

struct Color {
    uint8_t red = 0;
    uint8_t green = 0;
    uint8_t blue = 0;
};

// Gray → color (all three channels equal).
[[nodiscard]] TColorImage ToColor(const voyahslam::images::TImageU8Gray& image);

// `left` and `right` side by side. Heights must match.
[[nodiscard]] TColorImage HorizontalConcat(const TColorImage& left, const TColorImage& right);

} // namespace voyahviz

#endif // VOYAHVIZ_COLOR_IMAGE_H
