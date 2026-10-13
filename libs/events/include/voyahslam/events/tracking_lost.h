#ifndef VOYAHSLAM_EVENTS_TRACKING_LOST_H
#define VOYAHSLAM_EVENTS_TRACKING_LOST_H

#include <voyahslam/events/event_types.h>

namespace voyahslam::events {

// Published by the Tracker when tracking against map `map_id` fails.
struct TrackingLost : Event {
    double timestamp = 0.0;  // of the measurement that failed (or timed out)
    // false: recently lost; the Tracker tries to relocalize in the map, which
    // must stay. true: lost for good; the Mapper discards the map.
    bool final = true;

    TrackingLost() : Event(EventType::TrackingLost) {}
};

} // namespace voyahslam::events

#endif // VOYAHSLAM_EVENTS_TRACKING_LOST_H
