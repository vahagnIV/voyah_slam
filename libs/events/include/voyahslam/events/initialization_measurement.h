#ifndef VOYAHSLAM_EVENTS_INITIALIZATION_MEASUREMENT_H
#define VOYAHSLAM_EVENTS_INITIALIZATION_MEASUREMENT_H

#include <voyahslam/events/event_types.h>
#include <voyahslam/features/measurement.h>

namespace voyahslam::events {

template <typename TFeatureExtractor>
struct InitializationMeasurement : Event {
    features::TMeasurement<TFeatureExtractor> measurement;

    InitializationMeasurement() : Event(EventType::InitializationMeasurement) {}
};

} // namespace voyahslam::events

#endif // VOYAHSLAM_EVENTS_INITIALIZATION_MEASUREMENT_H
