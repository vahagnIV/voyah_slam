#include <utility>
#include <voyahslam/features/instantiation.h>
#include <voyahslam/tracking/tracker.h>

namespace voyahslam::tracking {

template <typename TFeatureExtractor>
void Tracker<TFeatureExtractor>::Start() {}

template <typename TFeatureExtractor>
void Tracker<TFeatureExtractor>::Stop() {}

template <typename TFeatureExtractor>
void Tracker<TFeatureExtractor>::Track(TMonocularMeasurement measurement) {
    measurements_.enqueue(std::move(measurement));
}

template <typename TFeatureExtractor>
void Tracker<TFeatureExtractor>::Track(TBinocularMeasurement measurement) {
    measurements_.enqueue(std::move(measurement));
}

INSTANTIATE_CLASS(Tracker)

} // namespace voyahslam::tracking
