#ifndef VOYAHSLAM_FEATURES_MEASUREMENT_H
#define VOYAHSLAM_FEATURES_MEASUREMENT_H

#include <variant>
#include <voyahslam/features/binocular_measurement.h>
#include <voyahslam/features/monocular_measurement.h>

namespace voyahslam::features {

template <typename TFeatureExtractor>
using TMonocularMeasurement =
    MonocularMeasurement<typename TFeatureExtractor::TDescriptorScalarType,
                         TFeatureExtractor::DescriptorSize>;

template <typename TFeatureExtractor>
using TBinocularMeasurement =
    BinocularMeasurement<typename TFeatureExtractor::TDescriptorScalarType,
                         TFeatureExtractor::DescriptorSize>;

template <typename TFeatureExtractor>
using TMeasurement =
    std::variant<TMonocularMeasurement<TFeatureExtractor>, TBinocularMeasurement<TFeatureExtractor>>;

} // namespace voyahslam::features

#endif // VOYAHSLAM_FEATURES_MEASUREMENT_H
