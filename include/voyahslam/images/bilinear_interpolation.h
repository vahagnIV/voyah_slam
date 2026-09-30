#ifndef VOYAHSLAM_IMAGES_BILINEAR_INTERPOLATION_H
#define VOYAHSLAM_IMAGES_BILINEAR_INTERPOLATION_H

#include <cmath>
#include <voyahslam/images/image.h>

namespace voyahslam::images {

template <typename T, int C> class BilinearInterpolation {
public:
  T Sample(const Image<T, C> &image, float y, float x, size_t c) const {
    size_t rx = std::min(static_cast<unsigned>(std::floor(std::max(0.f, x))),
                         image.Width() - 1);
    size_t ty = std::min(static_cast<unsigned>(std::floor(std::max(0.f, y))),
                         image.Height() - 1);

    size_t lx = rx == image.Width() - 1 ? rx : rx + 1;
    size_t by = ty == image.Height() - 1 ? ty : ty + 1;

    double value_at_top = InterpolateOnZeroOneInterval(
        image.At(ty, rx, c), image.At(ty, lx, c), x - rx);
    double value_at_bottom = InterpolateOnZeroOneInterval(
        image.At(by, rx, c), image.At(by, lx, c), x - rx);

    double result =
        InterpolateOnZeroOneInterval(value_at_top, value_at_bottom, y - ty);

    result = std::max(
        static_cast<double>(std::numeric_limits<T>::min()),
        std::min(static_cast<double>(std::numeric_limits<T>::max()), result));
    return static_cast<T>(result);
  }

private:
  T InterpolateOnZeroOneInterval(double value_at_zero, double value_at_one,
                                 double x) const {
    return (value_at_one - value_at_zero) * x + value_at_zero;
  }
};

} // namespace voyahslam::images

#endif // VOYAHSLAM_IMAGES_BILINEAR_INTERPOLATION_H
