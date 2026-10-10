#ifndef VOYAHSLAM_FEATURES_MEASUREMENT_H
#define VOYAHSLAM_FEATURES_MEASUREMENT_H

#include <memory>
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

// A measurement as the map holds it: created once by the Tracker for a
// tracked frame, then immutable and shared read-only by the events and every
// thread's replica (no copies, no locks).
template <typename TFeatureExtractor>
using TSharedMeasurement = std::shared_ptr<const TMeasurement<TFeatureExtractor>>;

// The image matched against the map: the left one of a binocular measurement,
// the measurement itself for a monocular one.
template <typename TFeatureExtractor>
const TMonocularMeasurement<TFeatureExtractor>& LeftOf(const TMeasurement<TFeatureExtractor>& measurement) {
    if (const auto* binocular = std::get_if<TBinocularMeasurement<TFeatureExtractor>>(&measurement)) {
        return binocular->left();
    }
    return std::get<TMonocularMeasurement<TFeatureExtractor>>(measurement);
}

} // namespace voyahslam::features

#endif // VOYAHSLAM_FEATURES_MEASUREMENT_H
