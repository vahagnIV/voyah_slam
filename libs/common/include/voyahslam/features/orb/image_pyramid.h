#ifndef VOYAHSLAM_FEATURES_ORB_IMAGE_PYRAMID_H
#define VOYAHSLAM_FEATURES_ORB_IMAGE_PYRAMID_H

#include <voyahslam/defs.h>
#include <voyahslam/images/image.h>

namespace voyahslam::features::orb {

class ImagePyramid {
public:
  ImagePyramid() = default;
  void Initialize(size_t image_width, size_t image_height,
                  const std::vector<precision_t> &scale_factors);
  [[nodiscard]] size_t GetDepth() const;
  void Build(const images::TWorkingImage &image);
  const images::TWorkingImage &operator[](size_t i) const {
    return image_pyramid_[i];
  }
  // Size of level 0 (the full-resolution image the pyramid was initialised for).
  [[nodiscard]] unsigned Width() const { return image_width_; }
  [[nodiscard]] unsigned Height() const { return image_height_; }

  // True if Initialize() was called with exactly this image size and these scale factors, so
  // Build() of an image of that size needs no re-initialisation.
  [[nodiscard]] bool Fits(size_t image_width, size_t image_height,
                          const std::vector<precision_t> &scale_factors) const;

  [[nodiscard]] bool Initialized() const { return initialized_; };

  // Width or height of the level with the given scale factor for a full-resolution
  // width or height of `size`: the arithmetic Initialize() uses (multiply by
  // 1 / scale_factor, round, cast to unsigned). Public so that a size check made
  // before Initialize() (OrbFeatureExtractor::DetectAndExtract) cannot disagree
  // with the levels Initialize() creates.
  [[nodiscard]] static unsigned LevelSize(size_t size, precision_t scale_factor);

private:
  unsigned image_width_{0};
  unsigned image_height_{0};
  bool initialized_{false};
  std::vector<precision_t> inv_scale_factors_;
  std::vector<images::TWorkingImage> image_pyramid_;
};

} // namespace voyahslam::features::orb

#endif // VOYAHSLAM_FEATURES_ORB_IMAGE_PYRAMID_H
