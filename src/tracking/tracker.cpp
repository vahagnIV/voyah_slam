#include <voyahslam/features/instantiation.h>
#include <voyahslam/tracking/tracker.h>

namespace voyahslam::tracking {

template <typename TFeatureExtractor>
void Tracker<TFeatureExtractor>::Start() {}

template <typename TFeatureExtractor>
void Tracker<TFeatureExtractor>::Stop() {}

template <typename TFeatureExtractor>
void Tracker<TFeatureExtractor>::Track(const TCameraFrame& /*frame*/) {}

template <typename TFeatureExtractor>
void Tracker<TFeatureExtractor>::Track(const TBinocularCameraFrame& /*frame*/) {}

INSTANTIATE_CLASS(Tracker)

} // namespace voyahslam::tracking
