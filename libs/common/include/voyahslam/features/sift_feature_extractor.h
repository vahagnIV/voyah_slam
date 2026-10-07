#ifndef VOYAHSLAM_FEATURES_SIFT_FEATURE_EXTRACTOR_H
#define VOYAHSLAM_FEATURES_SIFT_FEATURE_EXTRACTOR_H

#include <voyahslam/features/image_features.h>
#include <voyahslam/images/image.h>

namespace voyahslam::features {

class SIFTFeatureExtractor {
public:
    static constexpr int DescriptorSize = 128;
    typedef float TDescriptorScalarType;
    typedef ImageFeatures<TDescriptorScalarType, DescriptorSize> TFeatures;
    typedef TFeatures::TDescriptor TDescriptor;
    typedef TFeatures::TDescriptors TDescriptors;
    typedef TDescriptorScalarType (*distance_f)(const TDescriptor &, const TDescriptor &);

    SIFTFeatureExtractor() = default;

    [[nodiscard]] TFeatures DetectAndExtract(const images::TWorkingImage& image, size_t number_of_features) const;

    // Level = 3 * (octave + 1) + layer, decoded from cv::KeyPoint::octave
    // (octave in the low byte as int8, layer in the next byte).
    static uint8_t LevelFromOctave(int packed_octave);
    // Keypoint scale relative to the full-resolution image: 2^(level/3) / 2.
    [[nodiscard]] double ScaleFactor(uint8_t level) const;

    static TDescriptorScalarType Distance(const TDescriptor& desc1, const TDescriptor& desc2);
};

} // namespace voyahslam::features

#endif // VOYAHSLAM_FEATURES_SIFT_FEATURE_EXTRACTOR_H
