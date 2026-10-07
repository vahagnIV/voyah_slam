#ifndef VOYAHSLAM_IMAGES_RESIZER_H
#define VOYAHSLAM_IMAGES_RESIZER_H

#include <voyahslam/images/bilinear_interpolation.h>

namespace voyahslam::images {

template <typename T, int C,
          template <typename, int> class Sampler = BilinearInterpolation>
class Resizer {
  Sampler<T, C> sampler_;

public:
  Image<T, C> Resize(const Image<T, C> &src, size_t new_width, size_t new_height) const {
    Image<T, C> resizedImage(new_width, new_height);
    Resize(src, resizedImage);
    return resizedImage;
  }

  void Resize(const Image<T, C> &src, Image<T, C> &image_to) const {
    double scale_x = static_cast<double>(src.Width()) / image_to.Width();
    double scale_y = static_cast<double>(src.Height()) / image_to.Height();
    for (size_t y = 0; y < image_to.Height(); ++y) {
      float y_o = scale_y * y;
      for (size_t x = 0; x < image_to.Width(); ++x) {
        float x_o = scale_x * x;
        for (size_t c = 0; c < image_to.Channels(); ++c) {
          image_to.Set(y, x, c, sampler_.Sample(src, y_o, x_o, c));
        }
      }
    }
  }
};

} // namespace voyahslam::images

#endif // VOYAHSLAM_IMAGES_RESIZER_H
