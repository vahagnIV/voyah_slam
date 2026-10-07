#ifndef VOYAHSLAM_EVENTS_LOOP_DETECTED_H
#define VOYAHSLAM_EVENTS_LOOP_DETECTED_H

#include <voyahslam/events/event_types.h>
#include <voyahslam/map/frame_id.h>
#include <voyahslam/memory/recyclable.h>

namespace voyahslam::events {

struct LoopDetected
    : Event
    , memory::Recyclable<LoopDetected, memory::ThreadSafePolicy> {
    map::FrameId query_frame_id;
    map::FrameId match_frame_id;

    LoopDetected()
        : Event(EventType::LoopDetected,
                [](Event* e) {
                    static_cast<LoopDetected*>(e)->
                        memory::Recyclable<LoopDetected, memory::ThreadSafePolicy>::Recycle();
                },
                [](Event* e, int n) {
                    static_cast<LoopDetected*>(e)->SetRefCount(n);
                }) {}
};

} // namespace voyahslam::events

#endif // VOYAHSLAM_EVENTS_LOOP_DETECTED_H
