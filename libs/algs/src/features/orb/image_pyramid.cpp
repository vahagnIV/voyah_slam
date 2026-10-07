#include "voyahslam/features/orb/image_pyramid.h"

#include "voyahslam/images/resizer.h"

namespace voyahslam::features::orb {

void ImagePyramid::Initialize(size_t image_width, size_t image_height,
                           const std::vector<precision_t> &scale_factors) {
  initialized_ = true;
  image_width_ = image_width;
  image_height_ = image_height;
  inv_scale_factors_.reserve(scale_factors.size());
  for (auto s : scale_factors) {
    assert(s > 0);
    inv_scale_factors_.push_back(1. / s);
  }
  for (double scale : inv_scale_factors_) {
    auto width = static_cast<unsigned>(std::round(image_width_ * scale));
    auto height = static_cast<unsigned>(std::round(image_height_ * scale));
    image_pyramid_.push_back(images::TWorkingImage(width, height));
  }
}

size_t ImagePyramid::GetDepth() const { return image_pyramid_.size(); }

void ImagePyramid::Build(const images::TWorkingImage &image) {
  assert(image_pyramid_[0].Width() == image.Width());
  assert(image_pyramid_[0].Height() == image.Height());
  images::Resizer<images::TWorkingImage::TScalar,
                  images::TWorkingImage::Channel>
      resizer{};
  image.CopyTo(image_pyramid_[0]);
  for (size_t level = 1; level < inv_scale_factors_.size(); ++level) {
    resizer.Resize(image_pyramid_[level - 1], image_pyramid_[level]);
  }
}

} // namespace voyahslam::features::orb
