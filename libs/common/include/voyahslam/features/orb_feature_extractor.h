#ifndef VOYAHSLAM_FEATURES_ORB_FEATURE_EXTRACTOR_H
#define VOYAHSLAM_FEATURES_ORB_FEATURE_EXTRACTOR_H

#include <voyahslam/features/image_features.h>
#include <voyahslam/features/orb/image_pyramid.h>
#include <voyahslam/images/image.h>

namespace voyahslam::features {

class OrbFeatureExtractor {
public:
  static constexpr int DescriptorSize = 32;
  typedef uint8_t TDescriptorScalarType;
  typedef ImageFeatures<TDescriptorScalarType, DescriptorSize> TFeatures;
  typedef TFeatures::TDescriptor TDescriptor;
  typedef TFeatures::TDescriptors TDescriptors;
  typedef TDescriptorScalarType (*distance_f)(const TDescriptor &, const TDescriptor &);

  OrbFeatureExtractor(precision_t scale_factor = 1.2, size_t levels = 8,
                      unsigned init_threshold_FAST = 20,
                      unsigned min_threshold_FAST = 7);
  [[nodiscard]] TFeatures DetectAndExtract(const images::TWorkingImage &image, size_t number_of_features) const;
  static TDescriptorScalarType Distance(const TDescriptor& desc1, const TDescriptor& desc2);

  // Scale of pyramid `level` relative to the full-resolution image.
  [[nodiscard]] double ScaleFactor(uint8_t level) const;
  [[nodiscard]] size_t num_levels() const;

  // Pattern point `index` (0..511) rotated by the angle with the given cosine
  // and sine, rounded per component (ORB-SLAM3 cvRound). Public for tests.
  static geometry::TPoint2DI RotatedPatternOffset(int index, float cos_a, float sin_a);
  // Convenience overload; computes sin/cos per call, not for hot loops.
  static geometry::TPoint2DI RotatedPatternOffset(int index, float angle);

  // Public for tests.
  TDescriptors ComputeDescriptors(const images::TWorkingImage &image,
                                  const geometry::TPoints2DF &points,
                                  const geometry::TRowVectorF &angles) const;

private:
  precision_t scale_factor_;
  precision_t log_scale_factor_;
  unsigned init_threshold_FAST_;
  unsigned min_threshold_FAST_;

  std::vector<precision_t> scale_factors_;
  mutable orb::ImagePyramid pyramid_;

  std::vector<precision_t> level_sigma2_;
  std::vector<precision_t> inv_level_sigma2_;

  void Initialize();
  static decltype(scale_factors_) ComputeScaleFactors(precision_t scale, size_t levels);
  void ComputeLevels();
  TFeatures ComputeKeyPointsOctTree(size_t features) const;
  std::tuple<geometry::TPoints2DF, geometry::TRowVectorF>
  ComputePointsPerLevel(size_t level, int n_features) const;
  std::vector<size_t> ComputeFeaturesPerLevel(size_t number_of_features) const;

  geometry::TRowVectorF
  ComputeOrientation(const images::TWorkingImage &image,
                     const geometry::TPoints2DF &points) const;
  static float ComputeOrientationSinglePoint(const images::TWorkingImage &image,
                                             const geometry::TPoint2DF &point);

  images::Image<float, 1> gaussian_filter_x_, gaussian_filter_y_;
};

} // namespace voyahslam::features

#endif // VOYAHSLAM_FEATURES_ORB_FEATURE_EXTRACTOR_H
