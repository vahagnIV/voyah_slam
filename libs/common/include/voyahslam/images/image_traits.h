#ifndef VOYAHSLAM_IMAGES_IMAGE_TRAITS_H
#define VOYAHSLAM_IMAGES_IMAGE_TRAITS_H

#include <voyahslam/images/image.h>

namespace voyahslam::images {

template<typename ImageType>
struct ImageTraits{};

template<>
struct ImageTraits<TImageU8Gray> {
  constexpr static const int MaxValue = 255;
  constexpr static const int MinValue = 0;
};

template<>
struct ImageTraits<TImageF32Gray> {
  constexpr static const float MaxValue = 1.;
  constexpr static const float MinValue = 0.;
};

} // namespace voyahslam::images

#endif // VOYAHSLAM_IMAGES_IMAGE_TRAITS_H
