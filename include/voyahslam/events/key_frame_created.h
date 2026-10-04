#ifndef VOYAHSLAM_EVENTS_KEY_FRAME_CREATED_H
#define VOYAHSLAM_EVENTS_KEY_FRAME_CREATED_H

#include <cstdint>
#include <vector>
#include <voyahslam/defs.h>
#include <voyahslam/events/event_types.h>
#include <voyahslam/features/measurement.h>
#include <voyahslam/geometry/rigid_transformation.h>
#include <voyahslam/map/frame_id.h>
#include <voyahslam/map/map_point_id.h>
#include <voyahslam/memory/recyclable.h>

namespace voyahslam::events {

template <typename TFeatureExtractor>
struct KeyFrameCreated
    : Event
    , memory::Recyclable<KeyFrameCreated<TFeatureExtractor>, memory::ThreadSafePolicy> {
    map::FrameId frame_id;
    geometry::RigidTransformation pose;  // T_cw (world -> camera)
    features::TMeasurement<TFeatureExtractor> measurement;  // carries the timestamp
    // Observations as pairs: feature observed_features[i] of `measurement`
    // observes map point observed_points[i] of map `map_id`.
    std::vector<uint32_t> observed_features;
    std::vector<map::MapPointId> observed_points;

    // Called by the pool on reuse; keeps the vectors' capacity.
    void Reset() {
        observed_features.clear();
        observed_points.clear();
    }

    KeyFrameCreated()
        : Event(EventType::KeyFrameCreated,
                [](Event* e) {
                    static_cast<KeyFrameCreated*>(e)->
                        memory::Recyclable<KeyFrameCreated, memory::ThreadSafePolicy>::Recycle();
                },
                [](Event* e, int n) {
                    static_cast<KeyFrameCreated*>(e)->SetRefCount(n);
                }) {}
};

} // namespace voyahslam::events

#endif // VOYAHSLAM_EVENTS_KEY_FRAME_CREATED_H
