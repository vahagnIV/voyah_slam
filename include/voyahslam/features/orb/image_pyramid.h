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
  const images::TWorkingImage &operator[](size_t i) {
    return image_pyramid_[i];
  }

  [[nodiscard]] bool Initialized() const { return initialized_; };

private:
  unsigned image_width_;
  unsigned image_height_;
  bool initialized_{false};
  std::vector<precision_t> inv_scale_factors_;
  std::vector<images::TWorkingImage> image_pyramid_;
};

} // namespace voyahslam::features::orb

#endif // VOYAHSLAM_FEATURES_ORB_IMAGE_PYRAMID_H
