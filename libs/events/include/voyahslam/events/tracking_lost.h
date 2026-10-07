#ifndef VOYAHSLAM_EVENTS_TRACKING_LOST_H
#define VOYAHSLAM_EVENTS_TRACKING_LOST_H

#include <voyahslam/events/event_types.h>
#include <voyahslam/memory/recyclable.h>

namespace voyahslam::events {

struct TrackingLost
    : Event
    , memory::Recyclable<TrackingLost, memory::ThreadSafePolicy> {
    double timestamp = 0.0;

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
