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

  // Throws std::invalid_argument (in every build) when `levels` is 0 or `scale_factor` is not a
  // finite number greater than 1; nothing is computed from an invalid value.
  OrbFeatureExtractor(precision_t scale_factor = 1.2, size_t levels = 8,
                      unsigned init_threshold_FAST = 20,
                      unsigned min_threshold_FAST = 7);
  // Per-thread scratch for DetectAndExtract (the image pyramid). NOT thread-safe: keep one per
  // extraction thread (stereo: two if left and right are extracted in parallel). Reusing it
  // across frames avoids the per-frame pyramid allocation; it re-initialises itself when the
  // image size or the extractor's level count changes.
  class Workspace {
  public:
    Workspace() = default;

  private:
    friend class OrbFeatureExtractor;
    orb::ImagePyramid pyramid_;
  };

  // Nothing in the extractor changes after construction, so one extractor may be shared by several
  // threads as long as each passes its own Workspace.
  //
  // Both overloads throw std::invalid_argument (in every build, message containing "too small")
  // when any pyramid level would be narrower or shorter than MinLevelSize() = 66 px (the 31 px
  // ORB patch plus one 35 px FAST cell), and for an empty image. With the defaults (scale 1.2,
  // 8 levels) the smallest level is the image divided by 1.2^7 = 3.58, so the smallest accepted
  // image is 235x235 px. The check runs before the workspace is touched: a rejected call leaves
  // the Workspace as it was.
  [[nodiscard]] TFeatures DetectAndExtract(const images::TWorkingImage &image, size_t number_of_features,
                                           Workspace &out_workspace) const;
  // Convenience: builds a temporary Workspace per call (allocates the pyramid); fine for tests
  // and one-offs. A loop over frames should reuse a Workspace instead.
  [[nodiscard]] TFeatures DetectAndExtract(const images::TWorkingImage &image, size_t number_of_features) const;
  static TDescriptorScalarType Distance(const TDescriptor& desc1, const TDescriptor& desc2);

  // Smallest width and height, in pixels, of every pyramid level that DetectAndExtract accepts.
  [[nodiscard]] static constexpr unsigned MinLevelSize() { return 66; }

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

  std::vector<precision_t> level_sigma2_;
  std::vector<precision_t> inv_level_sigma2_;

  // Returns `scale_factor` after checking the configuration (std::invalid_argument); used in the
  // first member initializer so no member is computed from an invalid value.
  static precision_t ValidatedScaleFactor(precision_t scale_factor, size_t levels);
  // Throws std::invalid_argument when a pyramid level of a width x height image would be below
  // MinLevelSize().
  void CheckImageSize(size_t width, size_t height) const;

  void Initialize();
  static decltype(scale_factors_) ComputeScaleFactors(precision_t scale, size_t levels);
  void ComputeLevels();
  TFeatures ComputeKeyPointsOctTree(const orb::ImagePyramid &pyramid, size_t features) const;
  std::tuple<geometry::TPoints2DF, geometry::TRowVectorF>
  ComputePointsPerLevel(const orb::ImagePyramid &pyramid, size_t level, int n_features) const;
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
