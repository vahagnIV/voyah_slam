#ifndef VOYAHSLAM_IMAGES_FILTER_H
#define VOYAHSLAM_IMAGES_FILTER_H

#include <voyahslam/images/image.h>
#include <cassert>
#include <cmath>
#include <limits>

namespace voyahslam::images {

template <typename T, int C = 1, typename K = T> class Filter {
public:
  static Image<decltype(std::declval<T>() * std::declval<K>()), C>
  Apply(const Image<K, 1> &kernel, const Image<T, C> &image) {
    using ResultType = decltype(std::declval<T>() * std::declval<K>());
    Image<ResultType, C> result(image.Width(), image.Height());

    assert(kernel.Width() > 1 && kernel.Height() > 1 &&
           "Kernel must be a 2D rectangular kernel (width > 1 AND height > 1)");
    assert(kernel.Width() <= image.Width() && kernel.Height() <= image.Height() &&
           "Kernel size must be smaller than or equal to image size");

    const size_t kernel_width = kernel.Width();
    const size_t kernel_height = kernel.Height();
    const size_t kernel_center_x = kernel_width / 2;
    const size_t kernel_center_y = kernel_height / 2;

    for (size_t y = 0; y < image.Height(); ++y) {
      for (size_t x = 0; x < image.Width(); ++x) {
        for (size_t c = 0; c < image.Channels(); ++c) {
          ResultType sum = ResultType(0);
          K kernel_sum = K(0);

          for (size_t ky = 0; ky < kernel_height; ++ky) {
            for (size_t kx = 0; kx < kernel_width; ++kx) {
              int image_x = static_cast<int>(x) + static_cast<int>(kx) - static_cast<int>(kernel_center_x);
              int image_y = static_cast<int>(y) + static_cast<int>(ky) - static_cast<int>(kernel_center_y);

              int clamped_x = image_x;
              int clamped_y = image_y;
              if (clamped_x < 0) clamped_x = -clamped_x;
              if (clamped_x >= image.Width()) clamped_x = 2 * image.Width() - clamped_x - 1;
              if (clamped_y < 0) clamped_y = -clamped_y;
              if (clamped_y >= image.Height()) clamped_y = 2 * image.Height() - clamped_y - 1;

              K kernel_val = kernel.At(ky, kx, 0);
              sum += static_cast<ResultType>(image.At(clamped_y, clamped_x, c)) * kernel_val;
              kernel_sum += kernel_val;
            }
          }

          if (kernel_sum != K(0)) {
            result.Set(y, x, c, sum / kernel_sum);
          } else {
            result.Set(y, x, c, ResultType(0));
          }
        }
      }
    }

    return result;
  }

  static Image<decltype(std::declval<T>() * std::declval<K>()), C>
  ApplySeparable(const Image<K, 1> &kernel_x, const Image<K, 1> &kernel_y,
                 const Image<T, C> &image) {
    using ResultType = decltype(std::declval<T>() * std::declval<K>());

    assert(kernel_x.Height() == 1 && "kernel_x must be a row vector (height=1)");
    assert(kernel_y.Width() == 1 && "kernel_y must be a column vector (width=1)");

    Image<ResultType, C> intermediate = ApplyHorizontal(kernel_x, image);
    return ApplyVertical(kernel_y, intermediate);
  }

private:
  // Templated on the input pixel type: in the separable filter the second pass
  // reads the first pass's (wider) result type.
  template <typename TIn>
  static Image<decltype(std::declval<TIn>() * std::declval<K>()), C>
  ApplyHorizontal(const Image<K, 1> &kernel, const Image<TIn, C> &image) {
    using ResultType = decltype(std::declval<TIn>() * std::declval<K>());
    Image<ResultType, C> result(image.Width(), image.Height());
    const size_t kernel_size = kernel.Width();
    const size_t kernel_center = kernel_size / 2;

    assert(kernel_size <= image.Width() &&
           "Kernel width must be smaller than or equal to image width");

    for (size_t y = 0; y < image.Height(); ++y) {
      for (size_t x = 0; x < image.Width(); ++x) {
        for (size_t c = 0; c < image.Channels(); ++c) {
          ResultType sum = ResultType(0);
          K kernel_sum = K(0);

          for (size_t k = 0; k < kernel_size; ++k) {
            int image_x = static_cast<int>(x) + static_cast<int>(k) - static_cast<int>(kernel_center);

            int clamped_x = image_x;
            if (clamped_x < 0) clamped_x = -clamped_x;
            if (clamped_x >= image.Width()) clamped_x = 2 * image.Width() - clamped_x - 1;

            K kernel_val = kernel.At(0, k, 0);
            sum += static_cast<ResultType>(image.At(y, clamped_x, c)) * kernel_val;
            kernel_sum += kernel_val;
          }

          if (kernel_sum != K(0)) {
            result.Set(y, x, c, sum / kernel_sum);
          } else {
            result.Set(y, x, c, ResultType(0));
          }
        }
      }
    }

    return result;
  }

  // Templated on the input pixel type: in the separable filter the second pass
  // reads the first pass's (wider) result type.
  template <typename TIn>
  static Image<decltype(std::declval<TIn>() * std::declval<K>()), C>
  ApplyVertical(const Image<K, 1> &kernel, const Image<TIn, C> &image) {
    using ResultType = decltype(std::declval<TIn>() * std::declval<K>());
    Image<ResultType, C> result(image.Width(), image.Height());
    const size_t kernel_size = kernel.Height();
    const size_t kernel_center = kernel_size / 2;

    assert(kernel_size <= image.Height() &&
           "Kernel height must be smaller than or equal to image height");

    for (size_t y = 0; y < image.Height(); ++y) {
      for (size_t x = 0; x < image.Width(); ++x) {
        for (size_t c = 0; c < image.Channels(); ++c) {
          ResultType sum = ResultType(0);
          K kernel_sum = K(0);

          for (size_t k = 0; k < kernel_size; ++k) {
            int image_y = static_cast<int>(y) + static_cast<int>(k) - static_cast<int>(kernel_center);

            int clamped_y = image_y;
            if (clamped_y < 0) clamped_y = -clamped_y;
            if (clamped_y >= image.Height()) clamped_y = 2 * image.Height() - clamped_y - 1;

            K kernel_val = kernel.At(k, 0, 0);
            sum += static_cast<ResultType>(image.At(clamped_y, x, c)) * kernel_val;
            kernel_sum += kernel_val;
          }

          if (kernel_sum != K(0)) {
            result.Set(y, x, c, sum / kernel_sum);
          } else {
            result.Set(y, x, c, ResultType(0));
          }
        }
      }
    }

    return result;
  }
};

} // namespace voyahslam::images

#endif // VOYAHSLAM_IMAGES_FILTER_H
