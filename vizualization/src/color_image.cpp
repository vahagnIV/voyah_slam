#include <voyahviz/color_image.h>

#include <cassert>
#include <cstring>

namespace voyahviz {

TColorImage ToColor(const voyahslam::images::TImageU8Gray& image) {
    TColorImage color(image.Width(), image.Height());
    for (unsigned y = 0; y < image.Height(); ++y) {
        const uint8_t* src = &image.At(y, 0, 0);
        uint8_t* dst = &const_cast<uint8_t&>(color.At(y, 0, 0));
        for (unsigned x = 0; x < image.Width(); ++x) {
            dst[3 * x] = dst[3 * x + 1] = dst[3 * x + 2] = src[x];
        }
    }
    return color;
}

TColorImage HorizontalConcat(const TColorImage& left, const TColorImage& right) {
    assert(left.Height() == right.Height() && "HorizontalConcat: heights differ");
    TColorImage result(left.Width() + right.Width(), left.Height());
    for (unsigned y = 0; y < left.Height(); ++y) {
        uint8_t* dst = &const_cast<uint8_t&>(result.At(y, 0, 0));
        std::memcpy(dst, &left.At(y, 0, 0), left.Width() * 3);
        std::memcpy(dst + left.Width() * 3, &right.At(y, 0, 0), right.Width() * 3);
    }
    return result;
}

} // namespace voyahviz
