#ifndef VOYAHSLAM_EVENTS_KEY_FRAME_CREATED_H
#define VOYAHSLAM_EVENTS_KEY_FRAME_CREATED_H

#include <voyahslam/defs.h>
#include <voyahslam/events/event_types.h>
#include <voyahslam/map/frame_id.h>
#include <voyahslam/memory/recyclable.h>

namespace voyahslam::events {

template <typename TFeatureExtractor, typename TMeasurement>
struct KeyFrameCreated
    : Event
    , memory::Recyclable<KeyFrameCreated<TFeatureExtractor, TMeasurement>, memory::ThreadSafePolicy> {
    map::FrameId frame_id;
    geometry::TPose pose;
    TMeasurement measurement;  // carries the timestamp

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
