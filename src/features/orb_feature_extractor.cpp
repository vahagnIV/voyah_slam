#include "voyahslam/features/orb_feature_extractor.h"
#include "opencv_bindings/opencv_bindings.h"
#include "voyahslam/images/filter.h"
#include "voyahslam/images/gaussian_blur.h"
#include "orb/extract_node.h"
#include "orb/quad_tree_distributor.h"

#include <numeric>
#if defined(AVX2) || defined(__AVX2__)
#include <immintrin.h>
#endif

namespace voyahslam::features {

static constexpr int HALF_PATCH_SIZE = 15;
static constexpr int PATCH_SIZE = 31;
static const int UMax[HALF_PATCH_SIZE + 1] = {15, 15, 15, 15, 14, 14, 14, 13,
                                              13, 12, 11, 10, 9,  8,  6,  3};

static const geometry::TPoint2DI PATTERN[512] = {
    geometry::TPoint2DI(8, -3),    geometry::TPoint2DI(9, 5),
    geometry::TPoint2DI(4, 2),     geometry::TPoint2DI(7, -12),
    geometry::TPoint2DI(-11, 9),   geometry::TPoint2DI(-8, 2),
    geometry::TPoint2DI(7, -12),   geometry::TPoint2DI(12, -13),
    geometry::TPoint2DI(2, -13),   geometry::TPoint2DI(2, 12),
    geometry::TPoint2DI(1, -7),    geometry::TPoint2DI(1, 6),
    geometry::TPoint2DI(-2, -10),  geometry::TPoint2DI(-2, -4),
    geometry::TPoint2DI(-13, -13), geometry::TPoint2DI(-11, -8),
    geometry::TPoint2DI(-13, -3),  geometry::TPoint2DI(-12, -9),
    geometry::TPoint2DI(10, 4),    geometry::TPoint2DI(11, 9),
    geometry::TPoint2DI(-13, -8),  geometry::TPoint2DI(-8, -9),
    geometry::TPoint2DI(-11, 7),   geometry::TPoint2DI(-9, 12),
    geometry::TPoint2DI(7, 7),     geometry::TPoint2DI(12, 6),
    geometry::TPoint2DI(-4, -5),   geometry::TPoint2DI(-3, 0),
    geometry::TPoint2DI(-13, 2),   geometry::TPoint2DI(-12, -3),
    geometry::TPoint2DI(-9, 0),    geometry::TPoint2DI(-7, 5),
    geometry::TPoint2DI(12, -6),   geometry::TPoint2DI(12, -1),
    geometry::TPoint2DI(-3, 6),    geometry::TPoint2DI(-2, 12),
    geometry::TPoint2DI(-6, -13),  geometry::TPoint2DI(-4, -8),
    geometry::TPoint2DI(11, -13),  geometry::TPoint2DI(12, -8),
    geometry::TPoint2DI(4, 7),     geometry::TPoint2DI(5, 1),
    geometry::TPoint2DI(5, -3),    geometry::TPoint2DI(10, -3),
    geometry::TPoint2DI(3, -7),    geometry::TPoint2DI(6, 12),
    geometry::TPoint2DI(-8, -7),   geometry::TPoint2DI(-6, -2),
    geometry::TPoint2DI(-2, 11),   geometry::TPoint2DI(-1, -10),
    geometry::TPoint2DI(-13, 12),  geometry::TPoint2DI(-8, 10),
    geometry::TPoint2DI(-7, 3),    geometry::TPoint2DI(-5, -3),
    geometry::TPoint2DI(-4, 2),    geometry::TPoint2DI(-3, 7),
    geometry::TPoint2DI(-10, -12), geometry::TPoint2DI(-6, 11),
    geometry::TPoint2DI(5, -12),   geometry::TPoint2DI(6, -7),
    geometry::TPoint2DI(5, -6),    geometry::TPoint2DI(7, -1),
    geometry::TPoint2DI(1, 0),     geometry::TPoint2DI(4, -5),
    geometry::TPoint2DI(9, 11),    geometry::TPoint2DI(11, -13),
    geometry::TPoint2DI(4, 7),     geometry::TPoint2DI(4, 12),
    geometry::TPoint2DI(2, -1),    geometry::TPoint2DI(4, 4),
    geometry::TPoint2DI(-4, -12),  geometry::TPoint2DI(-2, 7),
    geometry::TPoint2DI(-8, -5),   geometry::TPoint2DI(-7, -10),
    geometry::TPoint2DI(4, 11),    geometry::TPoint2DI(9, 12),
    geometry::TPoint2DI(0, -8),    geometry::TPoint2DI(1, -13),
    geometry::TPoint2DI(-13, -2),  geometry::TPoint2DI(-8, 2),
    geometry::TPoint2DI(-3, -2),   geometry::TPoint2DI(-2, 3),
    geometry::TPoint2DI(-6, 9),    geometry::TPoint2DI(-4, -9),
    geometry::TPoint2DI(8, 12),    geometry::TPoint2DI(10, 7),
    geometry::TPoint2DI(0, 9),     geometry::TPoint2DI(1, 3),
    geometry::TPoint2DI(7, -5),    geometry::TPoint2DI(11, -10),
    geometry::TPoint2DI(-13, -6),  geometry::TPoint2DI(-11, 0),
    geometry::TPoint2DI(10, 7),    geometry::TPoint2DI(12, 1),
    geometry::TPoint2DI(-6, -3),   geometry::TPoint2DI(-6, 12),
    geometry::TPoint2DI(10, -9),   geometry::TPoint2DI(12, -4),
    geometry::TPoint2DI(-13, 8),   geometry::TPoint2DI(-8, -12),
    geometry::TPoint2DI(-13, 0),   geometry::TPoint2DI(-8, -4),
    geometry::TPoint2DI(3, 3),     geometry::TPoint2DI(7, 8),
    geometry::TPoint2DI(5, 7),     geometry::TPoint2DI(10, -7),
    geometry::TPoint2DI(-1, 7),    geometry::TPoint2DI(1, -12),
    geometry::TPoint2DI(3, -10),   geometry::TPoint2DI(5, 6),
    geometry::TPoint2DI(2, -4),    geometry::TPoint2DI(3, -10),
    geometry::TPoint2DI(-13, 0),   geometry::TPoint2DI(-13, 5),
    geometry::TPoint2DI(-13, -7),  geometry::TPoint2DI(-12, 12),
    geometry::TPoint2DI(-13, 3),   geometry::TPoint2DI(-11, 8),
    geometry::TPoint2DI(-7, 12),   geometry::TPoint2DI(-4, 7),
    geometry::TPoint2DI(6, -10),   geometry::TPoint2DI(12, 8),
    geometry::TPoint2DI(-9, -1),   geometry::TPoint2DI(-7, -6),
    geometry::TPoint2DI(-2, -5),   geometry::TPoint2DI(0, 12),
    geometry::TPoint2DI(-12, 5),   geometry::TPoint2DI(-7, 5),
    geometry::TPoint2DI(3, -10),   geometry::TPoint2DI(8, -13),
    geometry::TPoint2DI(-7, -7),   geometry::TPoint2DI(-4, 5),
    geometry::TPoint2DI(-3, -2),   geometry::TPoint2DI(-1, -7),
    geometry::TPoint2DI(2, 9),     geometry::TPoint2DI(5, -11),
    geometry::TPoint2DI(-11, -13), geometry::TPoint2DI(-5, -13),
    geometry::TPoint2DI(-1, 6),    geometry::TPoint2DI(0, -1),
    geometry::TPoint2DI(5, -3),    geometry::TPoint2DI(5, 2),
    geometry::TPoint2DI(-4, -13),  geometry::TPoint2DI(-4, 12),
    geometry::TPoint2DI(-9, -6),   geometry::TPoint2DI(-9, 6),
    geometry::TPoint2DI(-12, -10), geometry::TPoint2DI(-8, -4),
    geometry::TPoint2DI(10, 2),    geometry::TPoint2DI(12, -3),
    geometry::TPoint2DI(7, 12),    geometry::TPoint2DI(12, 12),
    geometry::TPoint2DI(-7, -13),  geometry::TPoint2DI(-6, 5),
    geometry::TPoint2DI(-4, 9),    geometry::TPoint2DI(-3, 4),
    geometry::TPoint2DI(7, -1),    geometry::TPoint2DI(12, 2),
    geometry::TPoint2DI(-7, 6),    geometry::TPoint2DI(-5, 1),
    geometry::TPoint2DI(-13, 11),  geometry::TPoint2DI(-12, 5),
    geometry::TPoint2DI(-3, 7),    geometry::TPoint2DI(-2, -6),
    geometry::TPoint2DI(7, -8),    geometry::TPoint2DI(12, -7),
    geometry::TPoint2DI(-13, -7),  geometry::TPoint2DI(-11, -12),
    geometry::TPoint2DI(1, -3),    geometry::TPoint2DI(12, 12),
    geometry::TPoint2DI(2, -6),    geometry::TPoint2DI(3, 0),
    geometry::TPoint2DI(-4, 3),    geometry::TPoint2DI(-2, -13),
    geometry::TPoint2DI(-1, -13),  geometry::TPoint2DI(1, 9),
    geometry::TPoint2DI(7, 1),     geometry::TPoint2DI(8, -6),
    geometry::TPoint2DI(1, -1),    geometry::TPoint2DI(3, 12),
    geometry::TPoint2DI(9, 1),     geometry::TPoint2DI(12, 6),
    geometry::TPoint2DI(-1, -9),   geometry::TPoint2DI(-1, 3),
    geometry::TPoint2DI(-13, -13), geometry::TPoint2DI(-10, 5),
    geometry::TPoint2DI(7, 7),     geometry::TPoint2DI(10, 12),
    geometry::TPoint2DI(12, -5),   geometry::TPoint2DI(12, 9),
    geometry::TPoint2DI(6, 3),     geometry::TPoint2DI(7, 11),
    geometry::TPoint2DI(5, -13),   geometry::TPoint2DI(6, 10),
    geometry::TPoint2DI(2, -12),   geometry::TPoint2DI(2, 3),
    geometry::TPoint2DI(3, 8),     geometry::TPoint2DI(4, -6),
    geometry::TPoint2DI(2, 6),     geometry::TPoint2DI(12, -13),
    geometry::TPoint2DI(9, -12),   geometry::TPoint2DI(10, 3),
    geometry::TPoint2DI(-8, 4),    geometry::TPoint2DI(-7, 9),
    geometry::TPoint2DI(-11, 12),  geometry::TPoint2DI(-4, -6),
    geometry::TPoint2DI(1, 12),    geometry::TPoint2DI(2, -8),
    geometry::TPoint2DI(6, -9),    geometry::TPoint2DI(7, -4),
    geometry::TPoint2DI(2, 3),     geometry::TPoint2DI(3, -2),
    geometry::TPoint2DI(6, 3),     geometry::TPoint2DI(11, 0),
    geometry::TPoint2DI(3, -3),    geometry::TPoint2DI(8, -8),
    geometry::TPoint2DI(7, 8),     geometry::TPoint2DI(9, 3),
    geometry::TPoint2DI(-11, -5),  geometry::TPoint2DI(-6, -4),
    geometry::TPoint2DI(-10, 11),  geometry::TPoint2DI(-5, 10),
    geometry::TPoint2DI(-5, -8),   geometry::TPoint2DI(-3, 12),
    geometry::TPoint2DI(-10, 5),   geometry::TPoint2DI(-9, 0),
    geometry::TPoint2DI(8, -1),    geometry::TPoint2DI(12, -6),
    geometry::TPoint2DI(4, -6),    geometry::TPoint2DI(6, -11),
    geometry::TPoint2DI(-10, 12),  geometry::TPoint2DI(-8, 7),
    geometry::TPoint2DI(4, -2),    geometry::TPoint2DI(6, 7),
    geometry::TPoint2DI(-2, 0),    geometry::TPoint2DI(-2, 12),
    geometry::TPoint2DI(-5, -8),   geometry::TPoint2DI(-5, 2),
    geometry::TPoint2DI(7, -6),    geometry::TPoint2DI(10, 12),
    geometry::TPoint2DI(-9, -13),  geometry::TPoint2DI(-8, -8),
    geometry::TPoint2DI(-5, -13),  geometry::TPoint2DI(-5, -2),
    geometry::TPoint2DI(8, -8),    geometry::TPoint2DI(9, -13),
    geometry::TPoint2DI(-9, -11),  geometry::TPoint2DI(-9, 0),
    geometry::TPoint2DI(1, -8),    geometry::TPoint2DI(1, -2),
    geometry::TPoint2DI(7, -4),    geometry::TPoint2DI(9, 1),
    geometry::TPoint2DI(-2, 1),    geometry::TPoint2DI(-1, -4),
    geometry::TPoint2DI(11, -6),   geometry::TPoint2DI(12, -11),
    geometry::TPoint2DI(-12, -9),  geometry::TPoint2DI(-6, 4),
    geometry::TPoint2DI(3, 7),     geometry::TPoint2DI(7, 12),
    geometry::TPoint2DI(5, 5),     geometry::TPoint2DI(10, 8),
    geometry::TPoint2DI(0, -4),    geometry::TPoint2DI(2, 8),
    geometry::TPoint2DI(-9, 12),   geometry::TPoint2DI(-5, -13),
    geometry::TPoint2DI(0, 7),     geometry::TPoint2DI(2, 12),
    geometry::TPoint2DI(-1, 2),    geometry::TPoint2DI(1, 7),
    geometry::TPoint2DI(5, 11),    geometry::TPoint2DI(7, -9),
    geometry::TPoint2DI(3, 5),     geometry::TPoint2DI(6, -8),
    geometry::TPoint2DI(-13, -4),  geometry::TPoint2DI(-8, 9),
    geometry::TPoint2DI(-5, 9),    geometry::TPoint2DI(-3, -3),
    geometry::TPoint2DI(-4, -7),   geometry::TPoint2DI(-3, -12),
    geometry::TPoint2DI(6, 5),     geometry::TPoint2DI(8, 0),
    geometry::TPoint2DI(-7, 6),    geometry::TPoint2DI(-6, 12),
    geometry::TPoint2DI(-13, 6),   geometry::TPoint2DI(-5, -2),
    geometry::TPoint2DI(1, -10),   geometry::TPoint2DI(3, 10),
    geometry::TPoint2DI(4, 1),     geometry::TPoint2DI(8, -4),
    geometry::TPoint2DI(-2, -2),   geometry::TPoint2DI(2, -13),
    geometry::TPoint2DI(2, -12),   geometry::TPoint2DI(12, 12),
    geometry::TPoint2DI(-2, -13),  geometry::TPoint2DI(0, -6),
    geometry::TPoint2DI(4, 1),     geometry::TPoint2DI(9, 3),
    geometry::TPoint2DI(-6, -10),  geometry::TPoint2DI(-3, -5),
    geometry::TPoint2DI(-3, -13),  geometry::TPoint2DI(-1, 1),
    geometry::TPoint2DI(7, 5),     geometry::TPoint2DI(12, -11),
    geometry::TPoint2DI(4, -2),    geometry::TPoint2DI(5, -7),
    geometry::TPoint2DI(-13, 9),   geometry::TPoint2DI(-9, -5),
    geometry::TPoint2DI(7, 1),     geometry::TPoint2DI(8, 6),
    geometry::TPoint2DI(7, -8),    geometry::TPoint2DI(7, 6),
    geometry::TPoint2DI(-7, -4),   geometry::TPoint2DI(-7, 1),
    geometry::TPoint2DI(-8, 11),   geometry::TPoint2DI(-7, -8),
    geometry::TPoint2DI(-13, 6),   geometry::TPoint2DI(-12, -8),
    geometry::TPoint2DI(2, 4),     geometry::TPoint2DI(3, 9),
    geometry::TPoint2DI(10, -5),   geometry::TPoint2DI(12, 3),
    geometry::TPoint2DI(-6, -5),   geometry::TPoint2DI(-6, 7),
    geometry::TPoint2DI(8, -3),    geometry::TPoint2DI(9, -8),
    geometry::TPoint2DI(2, -12),   geometry::TPoint2DI(2, 8),
    geometry::TPoint2DI(-11, -2),  geometry::TPoint2DI(-10, 3),
    geometry::TPoint2DI(-12, -13), geometry::TPoint2DI(-7, -9),
    geometry::TPoint2DI(-11, 0),   geometry::TPoint2DI(-10, -5),
    geometry::TPoint2DI(5, -3),    geometry::TPoint2DI(11, 8),
    geometry::TPoint2DI(-2, -13),  geometry::TPoint2DI(-1, 12),
    geometry::TPoint2DI(-1, -8),   geometry::TPoint2DI(0, 9),
    geometry::TPoint2DI(-13, -11), geometry::TPoint2DI(-12, -5),
    geometry::TPoint2DI(-10, -2),  geometry::TPoint2DI(-10, 11),
    geometry::TPoint2DI(-3, 9),    geometry::TPoint2DI(-2, -13),
    geometry::TPoint2DI(2, -3),    geometry::TPoint2DI(3, 2),
    geometry::TPoint2DI(-9, -13),  geometry::TPoint2DI(-4, 0),
    geometry::TPoint2DI(-4, 6),    geometry::TPoint2DI(-3, -10),
    geometry::TPoint2DI(-4, 12),   geometry::TPoint2DI(-2, -7),
    geometry::TPoint2DI(-6, -11),  geometry::TPoint2DI(-4, 9),
    geometry::TPoint2DI(6, -3),    geometry::TPoint2DI(6, 11),
    geometry::TPoint2DI(-13, 11),  geometry::TPoint2DI(-5, 5),
    geometry::TPoint2DI(11, 11),   geometry::TPoint2DI(12, 6),
    geometry::TPoint2DI(7, -5),    geometry::TPoint2DI(12, -2),
    geometry::TPoint2DI(-1, 12),   geometry::TPoint2DI(0, 7),
    geometry::TPoint2DI(-4, -8),   geometry::TPoint2DI(-3, -2),
    geometry::TPoint2DI(-7, 1),    geometry::TPoint2DI(-6, 7),
    geometry::TPoint2DI(-13, -12), geometry::TPoint2DI(-8, -13),
    geometry::TPoint2DI(-7, -2),   geometry::TPoint2DI(-6, -8),
    geometry::TPoint2DI(-8, 5),    geometry::TPoint2DI(-6, -9),
    geometry::TPoint2DI(-5, -1),   geometry::TPoint2DI(-4, 5),
    geometry::TPoint2DI(-13, 7),   geometry::TPoint2DI(-8, 10),
    geometry::TPoint2DI(1, 5),     geometry::TPoint2DI(5, -13),
    geometry::TPoint2DI(1, 0),     geometry::TPoint2DI(10, -13),
    geometry::TPoint2DI(9, 12),    geometry::TPoint2DI(10, -1),
    geometry::TPoint2DI(5, -8),    geometry::TPoint2DI(10, -9),
    geometry::TPoint2DI(-1, 11),   geometry::TPoint2DI(1, -13),
    geometry::TPoint2DI(-9, -3),   geometry::TPoint2DI(-6, 2),
    geometry::TPoint2DI(-1, -10),  geometry::TPoint2DI(1, 12),
    geometry::TPoint2DI(-13, 1),   geometry::TPoint2DI(-8, -10),
    geometry::TPoint2DI(8, -11),   geometry::TPoint2DI(10, -6),
    geometry::TPoint2DI(2, -13),   geometry::TPoint2DI(3, -6),
    geometry::TPoint2DI(7, -13),   geometry::TPoint2DI(12, -9),
    geometry::TPoint2DI(-10, -10), geometry::TPoint2DI(-5, -7),
    geometry::TPoint2DI(-10, -8),  geometry::TPoint2DI(-8, -13),
    geometry::TPoint2DI(4, -6),    geometry::TPoint2DI(8, 5),
    geometry::TPoint2DI(3, 12),    geometry::TPoint2DI(8, -13),
    geometry::TPoint2DI(-4, 2),    geometry::TPoint2DI(-3, -3),
    geometry::TPoint2DI(5, -13),   geometry::TPoint2DI(10, -12),
    geometry::TPoint2DI(4, -13),   geometry::TPoint2DI(5, -1),
    geometry::TPoint2DI(-9, 9),    geometry::TPoint2DI(-4, 3),
    geometry::TPoint2DI(0, 3),     geometry::TPoint2DI(3, -9),
    geometry::TPoint2DI(-12, 1),   geometry::TPoint2DI(-6, 1),
    geometry::TPoint2DI(3, 2),     geometry::TPoint2DI(4, -8),
    geometry::TPoint2DI(-10, -10), geometry::TPoint2DI(-10, 9),
    geometry::TPoint2DI(8, -13),   geometry::TPoint2DI(12, 12),
    geometry::TPoint2DI(-8, -12),  geometry::TPoint2DI(-6, -5),
    geometry::TPoint2DI(2, 2),     geometry::TPoint2DI(3, 7),
    geometry::TPoint2DI(10, 6),    geometry::TPoint2DI(11, -8),
    geometry::TPoint2DI(6, 8),     geometry::TPoint2DI(8, -12),
    geometry::TPoint2DI(-7, 10),   geometry::TPoint2DI(-6, 5),
    geometry::TPoint2DI(-3, -9),   geometry::TPoint2DI(-3, 9),
    geometry::TPoint2DI(-1, -13),  geometry::TPoint2DI(-1, 5),
    geometry::TPoint2DI(-3, -7),   geometry::TPoint2DI(-3, 4),
    geometry::TPoint2DI(-8, -2),   geometry::TPoint2DI(-8, 3),
    geometry::TPoint2DI(4, 2),     geometry::TPoint2DI(12, 12),
    geometry::TPoint2DI(2, -5),    geometry::TPoint2DI(3, 11),
    geometry::TPoint2DI(6, -9),    geometry::TPoint2DI(11, -13),
    geometry::TPoint2DI(3, -1),    geometry::TPoint2DI(7, 12),
    geometry::TPoint2DI(11, -1),   geometry::TPoint2DI(12, 4),
    geometry::TPoint2DI(-3, 0),    geometry::TPoint2DI(-3, 6),
    geometry::TPoint2DI(4, -11),   geometry::TPoint2DI(4, 12),
    geometry::TPoint2DI(2, -4),    geometry::TPoint2DI(2, 1),
    geometry::TPoint2DI(-10, -6),  geometry::TPoint2DI(-8, 1),
    geometry::TPoint2DI(-13, 7),   geometry::TPoint2DI(-11, 1),
    geometry::TPoint2DI(-13, 12),  geometry::TPoint2DI(-11, -13),
    geometry::TPoint2DI(6, 0),     geometry::TPoint2DI(11, -13),
    geometry::TPoint2DI(0, -1),    geometry::TPoint2DI(1, 4),
    geometry::TPoint2DI(-13, 3),   geometry::TPoint2DI(-9, -2),
    geometry::TPoint2DI(-9, 8),    geometry::TPoint2DI(-6, -3),
    geometry::TPoint2DI(-13, -6),  geometry::TPoint2DI(-8, -2),
    geometry::TPoint2DI(5, -9),    geometry::TPoint2DI(8, 10),
    geometry::TPoint2DI(2, 7),     geometry::TPoint2DI(3, -9),
    geometry::TPoint2DI(-1, -6),   geometry::TPoint2DI(-1, -1),
    geometry::TPoint2DI(9, 5),     geometry::TPoint2DI(11, -2),
    geometry::TPoint2DI(11, -3),   geometry::TPoint2DI(12, -8),
    geometry::TPoint2DI(3, 0),     geometry::TPoint2DI(3, 5),
    geometry::TPoint2DI(-1, 4),    geometry::TPoint2DI(0, 10),
    geometry::TPoint2DI(3, -6),    geometry::TPoint2DI(4, 5),
    geometry::TPoint2DI(-13, 0),   geometry::TPoint2DI(-10, 5),
    geometry::TPoint2DI(5, 8),     geometry::TPoint2DI(12, 11),
    geometry::TPoint2DI(8, 9),     geometry::TPoint2DI(9, -6),
    geometry::TPoint2DI(7, -4),    geometry::TPoint2DI(8, -12),
    geometry::TPoint2DI(-10, 4),   geometry::TPoint2DI(-10, 9),
    geometry::TPoint2DI(7, 3),     geometry::TPoint2DI(12, 4),
    geometry::TPoint2DI(9, -7),    geometry::TPoint2DI(10, -2),
    geometry::TPoint2DI(7, 0),     geometry::TPoint2DI(12, -2),
    geometry::TPoint2DI(-1, -6),   geometry::TPoint2DI(0, -11)};

OrbFeatureExtractor::OrbFeatureExtractor(precision_t scale_factor,
                                         size_t levels,
                                         unsigned init_threshold_FAST,
                                         unsigned min_threshold_FAST)
    : scale_factor_(scale_factor), log_scale_factor_(std::log(scale_factor_)),
      init_threshold_FAST_(init_threshold_FAST),
      min_threshold_FAST_(min_threshold_FAST),
      scale_factors_(ComputeScaleFactors(scale_factor, levels)) {
  assert(scale_factor != 0);
  Initialize();
}

OrbFeatureExtractor::TFeatures
OrbFeatureExtractor::DetectAndExtract(const images::TWorkingImage &image,
                                      size_t number_of_features) const {
  if (!pyramid_.Initialized())
    pyramid_.Initialize(image.Width(), image.Height(), scale_factors_);

  pyramid_.Build(image);
  return ComputeKeyPointsOctTree(number_of_features);
}

void OrbFeatureExtractor::Initialize() {
  ComputeLevels();
  gaussian_filter_x_ = images::CreateGaussianKernelSeparableXFromSize(7, 2.);
  gaussian_filter_y_ = images::CreateGaussianKernelSeparableYFromSize(7, 2.);
}

OrbFeatureExtractor::TDescriptorScalarType
OrbFeatureExtractor::Distance(const TDescriptor &desc1,
                              const TDescriptor &desc2) {

  const uint8_t *a = reinterpret_cast<const uint8_t *>(desc1.data());
  const uint8_t *b = reinterpret_cast<const uint8_t *>(desc2.data());

#if defined(AVX2) || defined(__AVX2__)
  const __m256i va = _mm256_loadu_si256(reinterpret_cast<const __m256i *>(a));
  const __m256i vb = _mm256_loadu_si256(reinterpret_cast<const __m256i *>(b));
  const __m256i vx = _mm256_xor_si256(va, vb);

  const __m256i low_mask = _mm256_set1_epi8(0x0F);
  const __m256i lut =
      _mm256_setr_epi8(0, 1, 1, 2, 1, 2, 2, 3, 1, 2, 2, 3, 2, 3, 3, 4, 0, 1, 1,
                       2, 1, 2, 2, 3, 1, 2, 2, 3, 2, 3, 3, 4);

  const __m256i lo = _mm256_and_si256(vx, low_mask);
  const __m256i hi = _mm256_and_si256(_mm256_srli_epi16(vx, 4), low_mask);

  const __m256i pc_lo = _mm256_shuffle_epi8(lut, lo);
  const __m256i pc_hi = _mm256_shuffle_epi8(lut, hi);
  const __m256i pc = _mm256_add_epi8(pc_lo, pc_hi);

  const __m256i zero = _mm256_setzero_si256();
  const __m256i sums = _mm256_sad_epu8(pc, zero);

  alignas(32) uint64_t tmp[4];
  _mm256_store_si256(reinterpret_cast<__m256i *>(tmp), sums);
  uint32_t dist = static_cast<uint32_t>(tmp[0] + tmp[1] + tmp[2] + tmp[3]);

  return static_cast<OrbFeatureExtractor::TDescriptorScalarType>(dist);

#elif defined(SYMD) || defined(SIMD) || defined(__SSSE3__)
  const __m128i v0a = _mm_loadu_si128(reinterpret_cast<const __m128i *>(a));
  const __m128i v0b = _mm_loadu_si128(reinterpret_cast<const __m128i *>(b));
  const __m128i v1a =
      _mm_loadu_si128(reinterpret_cast<const __m128i *>(a + 16));
  const __m128i v1b =
      _mm_loadu_si128(reinterpret_cast<const __m128i *>(b + 16));

  const __m128i x0 = _mm_xor_si128(v0a, v0b);
  const __m128i x1 = _mm_xor_si128(v1a, v1b);

  const __m128i low_mask = _mm_set1_epi8(0x0F);
  const __m128i lut =
      _mm_setr_epi8(0, 1, 1, 2, 1, 2, 2, 3, 1, 2, 2, 3, 2, 3, 3, 4);

  auto popcnt128_sum = [&](const __m128i x) -> uint64_t {
    const __m128i lo = _mm_and_si128(x, low_mask);
    const __m128i hi = _mm_and_si128(_mm_srli_epi16(x, 4), low_mask);
    const __m128i pc =
        _mm_add_epi8(_mm_shuffle_epi8(lut, lo), _mm_shuffle_epi8(lut, hi));
    const __m128i zero = _mm_setzero_si128();
    const __m128i sad = _mm_sad_epu8(pc, zero);
    alignas(16) uint64_t t[2];
    _mm_store_si128(reinterpret_cast<__m128i *>(t), sad);
    return t[0] + t[1];
  };

  uint32_t dist = static_cast<uint32_t>(popcnt128_sum(x0) + popcnt128_sum(x1));
  return static_cast<TDescriptorScalarType>(dist);

#else
  uint32_t dist = 0;
  for (int i = 0; i < DescriptorSize; ++i) {
#if defined(__GNUC__) || defined(__clang__)
    dist += static_cast<uint32_t>(
        __builtin_popcount(static_cast<unsigned>(a[i] ^ b[i])));
#else
    uint8_t v = static_cast<uint8_t>(a[i] ^ b[i]);
    v = static_cast<uint8_t>(v - ((v >> 1) & 0x55));
    v = static_cast<uint8_t>((v & 0x33) + ((v >> 2) & 0x33));
    dist += static_cast<uint32_t>(((v + (v >> 4)) & 0x0F));
#endif
  }
  return static_cast<TDescriptorScalarType>(dist);
#endif
}

decltype(OrbFeatureExtractor::scale_factors_)
OrbFeatureExtractor::ComputeScaleFactors(precision_t scale, size_t levels) {
  std::vector<precision_t> scale_factors(levels);
  scale_factors[0] = 1.;
  for (size_t i = 1; i < levels; i++) {
    scale_factors[i] = scale_factors[i - 1] * scale;
  }
  return scale_factors;
}

void OrbFeatureExtractor::ComputeLevels() {
  level_sigma2_.resize(scale_factors_.size());
  inv_level_sigma2_.resize(scale_factors_.size());
  for (size_t i = 0; i < scale_factors_.size(); i++) {
    level_sigma2_[i] = scale_factors_[i] * scale_factors_[i];
    inv_level_sigma2_[i] = 1. / level_sigma2_[i];
  }
}

std::vector<size_t>
OrbFeatureExtractor::ComputeFeaturesPerLevel(size_t number_of_features) const {
  precision_t factor = precision_t(1) / scale_factor_;

  precision_t f = pow(factor, scale_factors_.size());
  precision_t number_of_desired_features_per_level =
      number_of_features * (1. - factor) /
      (1. - factor / scale_factors_.back());

  size_t total_features = 0;
  std::vector<size_t> features_per_level(scale_factors_.size());

  for (size_t level = 0; level < scale_factors_.size() - 1; level++) {
    features_per_level[level] =
        std::round(number_of_desired_features_per_level);
    total_features += features_per_level[level];
    number_of_desired_features_per_level *= factor;
  }
  if (number_of_features > total_features)
    features_per_level.back() += std::max(static_cast<int>(number_of_features) -
                                              static_cast<int>(total_features),
                                          int(0));
  return features_per_level;
}

OrbFeatureExtractor::TFeatures
OrbFeatureExtractor::ComputeKeyPointsOctTree(size_t features) const {

  std::vector<size_t> features_per_level = ComputeFeaturesPerLevel(features);

  std::vector<geometry::TPoints2DF> pts;
  std::vector<geometry::TRowVectorF> strngths;
  std::vector<geometry::TRowVectorF> ngls;
  std::vector<TDescriptors> dscrptrs;

  for (size_t level = 0; level < scale_factors_.size(); level++) {
    auto [r_points, r_strengths] =
        ComputePointsPerLevel(level, features_per_level[level]);
    auto r_angles = ComputeOrientation(pyramid_[level], r_points);
    auto r_descriptors =
        ComputeDescriptors(pyramid_[level], r_points, r_angles);
    pts.emplace_back(std::move(r_points));
    strngths.emplace_back(std::move(r_strengths));
    ngls.emplace_back(std::move(r_angles));
    dscrptrs.emplace_back(std::move(r_descriptors));
  }

  size_t total_features =
      std::accumulate(strngths.begin(), strngths.end(), 0ul,
                      [](size_t count, const geometry::TRowVectorF &a) {
                        return count + a.cols();
                      });

  geometry::TPoints2DF points;
  geometry::TRowVectorF strengths;
  geometry::TRowVectorF angles;
  TDescriptors descriptors{};

  points.resize(2, total_features);
  strengths.resize(total_features);
  angles.resize(total_features);
  descriptors.resize(DescriptorSize, total_features);

  size_t c = 0;
  for (size_t level = 0; level < pts.size(); level++) {
    const auto &pt_stack = pts[level];
    const auto &st_stack = strngths[level];
    const auto &ng_stack = ngls[level];

    for (Eigen::Index j = 0; j < st_stack.size(); j++) {
      points(0, c) = pt_stack(0, j) * scale_factors_[level];
      points(1, c) = pt_stack(1, j) * scale_factors_[level];
      angles(c) = ng_stack(j);
      strengths(c) = st_stack(j);
      descriptors.col(c) = dscrptrs[level].col(j);
      ++c;
    }
  }

  return TFeatures(std::move(points), std::move(strengths), std::move(angles),
                   std::move(descriptors));
}

float OrbFeatureExtractor::ComputeOrientationSinglePoint(
    const images::TWorkingImage &image, const geometry::TPoint2DF &point) {

  images::TWorkingImage::TScalar m_01 = 0, m_10 = 0;

  int x = point.x();
  int y = point.y();

  const auto *center = &image.At(y, x, 0);
  for (int u = -HALF_PATCH_SIZE; u <= HALF_PATCH_SIZE; ++u)
    m_10 += u * center[u];

  int step = (int)image.Stride();
  for (int v = 1; v <= HALF_PATCH_SIZE; ++v) {
    images::TWorkingImage::TScalar v_sum = 0;
    int d = UMax[v];
    int step_v = v * step;
    for (int u = -d; u <= d; ++u) {
      images::TWorkingImage::TScalar val_plus = center[u + step_v];
      images::TWorkingImage::TScalar val_minus = center[u - step_v];
      v_sum += (val_plus - val_minus);
      m_10 += u * (val_plus + val_minus);
    }
    m_01 += v * v_sum;
  }

  return atan2(m_01, m_10);
}

geometry::TRowVectorF OrbFeatureExtractor::ComputeOrientation(
    const images::TWorkingImage &image,
    const geometry::TPoints2DF &points) const {
  geometry::TRowVectorF result;
  result.resize(points.cols());
  for (geometry::TPoint2DF::Index i = 0; i < points.cols(); i++) {
    result(i) = ComputeOrientationSinglePoint(image, points.col(i));
  }
  return result;
}

std::tuple<geometry::TPoints2DF, geometry::TRowVectorF>
OrbFeatureExtractor::ComputePointsPerLevel(size_t level, int n_features) const {

  constexpr int W = 35;

  cv::Mat image =
      ImageTypeConverter<images::TWorkingImage>::ConvertToCvMatUint8Gray(
          pyramid_[level]);

  const int n_cols = (image.cols - PATCH_SIZE) / W;
  assert(n_cols > 0);
  const int n_rows = (image.rows - PATCH_SIZE) / W;
  assert(n_rows > 0);

  const int cell_width = std::ceil((image.cols - PATCH_SIZE) / n_cols);
  const int cell_height = std::ceil((image.rows - PATCH_SIZE) / n_rows);

  std::vector<std::vector<cv::KeyPoint>> all_cells_keypoints;

  for (int i = 0; i < n_rows; ++i) {
    int cell_begin_y = i * cell_height + HALF_PATCH_SIZE;
    int cell_end_y = std::min(cell_begin_y + cell_height, image.rows);

    for (int j = 0; j < n_cols; ++j) {
      int cell_begin_x = j * cell_width + HALF_PATCH_SIZE;
      int cell_end_x = std::min(cell_begin_x + cell_width, image.cols);

      std::vector<cv::KeyPoint> key_points_in_cell;

      auto cell_image = image.rowRange(cell_begin_y, cell_end_y)
                            .colRange(cell_begin_x, cell_end_x);
      double min_val, max_val;
      cv::minMaxLoc(cell_image, &min_val, &max_val);

      cv::FAST(cell_image, key_points_in_cell, init_threshold_FAST_, true);

      if (key_points_in_cell.empty()) {
        cv::FAST(cell_image, key_points_in_cell, min_threshold_FAST_, true);
      }
      for (auto &vit : key_points_in_cell) {
        vit.pt.x += static_cast<decltype(vit.pt.x)>(cell_begin_x);
        vit.pt.y += static_cast<decltype(vit.pt.y)>(cell_begin_y);
        vit.size = scale_factors_[level];
      }
      all_cells_keypoints.emplace_back(std::move(key_points_in_cell));
    }
  }

  size_t total_keypoint_count = std::accumulate(
      all_cells_keypoints.begin(), all_cells_keypoints.end(), size_t(0),
      [](int value, const std::vector<cv::KeyPoint> &key_points) {
        return value + key_points.size();
      });

  geometry::TPoints2DF points;
  geometry::TRowVectorF strengths;
  points.resize(2, total_keypoint_count);
  strengths.resize(total_keypoint_count);
  size_t i = 0;
  for (const auto &cell_keypoints : all_cells_keypoints) {
    for (auto cv_keypoint : cell_keypoints) {
      points(0, i) = cv_keypoint.pt.x;
      points(1, i) = cv_keypoint.pt.y;
      strengths(i) = cv_keypoint.response;
      ++i;
    }
  }

  std::vector<size_t> distributed_indices;
  orb::QuadTreeDistributor::DistributeOctTree(
      points, strengths, 0, image.cols, 0, image.rows, n_features,
      distributed_indices);

  geometry::TPoints2DF r_points;
  geometry::TRowVectorF r_strengths;
  r_points.resize(2, distributed_indices.size());
  r_strengths.resize(distributed_indices.size());
  for (size_t i = 0; i < distributed_indices.size(); i++) {
    int idx = distributed_indices[i];
    r_points(0, i) = points(0, idx);
    r_points(1, i) = points(1, idx);
    r_strengths(i) = strengths(idx);
  }
  return std::make_tuple(r_points, r_strengths);
}

OrbFeatureExtractor::TDescriptors OrbFeatureExtractor::ComputeDescriptors(
    const images::TWorkingImage &image, const geometry::TPoints2DF &points,
    const geometry::TRowVectorF &angles) const {
  auto blurred = images::Filter<images::TWorkingImage::TScalar>::ApplySeparable(
      gaussian_filter_x_, gaussian_filter_y_, image);
  TDescriptors result;
  result.resize(DescriptorSize, points.cols());
  for (Eigen::Index i = 0; i < points.cols(); ++i) {

    Eigen::Matrix<float, 2, 2> rotation;
    float sin = std::sin(angles(i));
    float cos = std::cos(angles(i));
    rotation << cos, -sin, sin, cos;

    auto center = &blurred.At(points(1, i), points(0, i), 0);

    constexpr int pattern_bit_count =
        8 * sizeof(OrbFeatureExtractor::TDescriptorScalarType);
    constexpr int pattern_size =
        sizeof(PATTERN) / sizeof(geometry::TPoint2DI) / 2 / pattern_bit_count;
    assert(pattern_size == DescriptorSize);

    for (int descriptor_int_idx = 0; descriptor_int_idx < pattern_size;
         ++descriptor_int_idx) {
      int pattern_start_idx = 2 * descriptor_int_idx * pattern_bit_count;
      OrbFeatureExtractor::TDescriptorScalarType val = 0;
      for (int j = 0; j < pattern_bit_count; ++j) {
        geometry::TPoint2DI dx =
            (rotation * PATTERN[pattern_start_idx + 2 * j].cast<float>())
                .cast<int>();
        int d = dx.y() * blurred.Stride() + dx.x();
        images::TWorkingImage::TScalar t0 = center[d], t1;
        dx = (rotation * PATTERN[pattern_start_idx + 2 * j + 1].cast<float>())
                 .cast<int>();
        d = dx.y() * blurred.Stride() + dx.x();
        t1 = center[d];
        val |= (t0 < t1) << j;
      }
      result(descriptor_int_idx, i) = val;
    }
  }
  return result;
}

} // namespace voyahslam::features
