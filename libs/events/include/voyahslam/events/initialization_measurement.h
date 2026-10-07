#ifndef VOYAHSLAM_EVENTS_INITIALIZATION_MEASUREMENT_H
#define VOYAHSLAM_EVENTS_INITIALIZATION_MEASUREMENT_H

#include <voyahslam/events/event_types.h>
#include <voyahslam/features/measurement.h>
#include <voyahslam/memory/recyclable.h>

namespace voyahslam::events {

template <typename TFeatureExtractor>
struct InitializationMeasurement
    : Event
    , memory::Recyclable<InitializationMeasurement<TFeatureExtractor>, memory::ThreadSafePolicy> {
    features::TMeasurement<TFeatureExtractor> measurement;

    InitializationMeasurement()
        : Event(EventType::InitializationMeasurement,
                [](Event* e) {
                    static_cast<InitializationMeasurement*>(e)->
                        memory::Recyclable<InitializationMeasurement<TFeatureExtractor>, memory::ThreadSafePolicy>::Recycle();
                },
                [](Event* e, int n) {
                    static_cast<InitializationMeasurement*>(e)->SetRefCount(n);
                }) {}
};

} // namespace voyahslam::events

#endif // VOYAHSLAM_EVENTS_INITIALIZATION_MEASUREMENT_H
