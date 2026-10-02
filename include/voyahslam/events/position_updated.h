#ifndef VOYAHSLAM_EVENTS_POSITION_UPDATED_H
#define VOYAHSLAM_EVENTS_POSITION_UPDATED_H

#include <voyahslam/defs.h>
#include <voyahslam/events/event_types.h>
#include <voyahslam/map/frame_id.h>
#include <voyahslam/memory/recyclable.h>

namespace voyahslam::events {

struct PositionUpdated
    : Event
    , memory::Recyclable<PositionUpdated, memory::ThreadSafePolicy> {
    map::FrameId frame_id;
    geometry::TPose pose;
    double timestamp;

    PositionUpdated()
        : Event(EventType::PositionUpdated,
                [](Event* e) {
                    static_cast<PositionUpdated*>(e)->
                        memory::Recyclable<PositionUpdated, memory::ThreadSafePolicy>::Recycle();
                },
                [](Event* e, int n) {
                    static_cast<PositionUpdated*>(e)->SetRefCount(n);
                }) {}
};

} // namespace voyahslam::events

#endif // VOYAHSLAM_EVENTS_POSITION_UPDATED_H
