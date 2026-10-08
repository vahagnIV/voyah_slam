#ifndef VOYAHSLAM_EVENTS_TRACKING_LOST_H
#define VOYAHSLAM_EVENTS_TRACKING_LOST_H

#include <voyahslam/events/event_types.h>
#include <voyahslam/memory/recyclable.h>

namespace voyahslam::events {

// Published by the Tracker when tracking against map `map_id` fails.
struct TrackingLost
    : Event
    , memory::Recyclable<TrackingLost, memory::ThreadSafePolicy> {
    double timestamp = 0.0;  // of the measurement that failed (or timed out)
    // false: recently lost; the Tracker tries to relocalize in the map, which
    // must stay. true: lost for good; the Mapper discards the map.
    bool final = true;

    // Called by the pool on reuse.
    void Reset() {
        timestamp = 0.0;
        final = true;
        priority = false;
    }

    TrackingLost()
        : Event(EventType::TrackingLost,
                [](Event* e) {
                    static_cast<TrackingLost*>(e)->
                        memory::Recyclable<TrackingLost, memory::ThreadSafePolicy>::Recycle();
                },
                [](Event* e, int n) {
                    static_cast<TrackingLost*>(e)->SetRefCount(n);
                }) {}
};

} // namespace voyahslam::events

#endif // VOYAHSLAM_EVENTS_TRACKING_LOST_H
