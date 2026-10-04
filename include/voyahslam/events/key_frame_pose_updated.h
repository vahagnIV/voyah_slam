#ifndef VOYAHSLAM_EVENTS_KEY_FRAME_POSE_UPDATED_H
#define VOYAHSLAM_EVENTS_KEY_FRAME_POSE_UPDATED_H

#include <voyahslam/defs.h>
#include <voyahslam/events/event_types.h>
#include <voyahslam/geometry/rigid_transformation.h>
#include <voyahslam/map/frame_id.h>
#include <voyahslam/memory/recyclable.h>

namespace voyahslam::events {

struct KeyFramePoseUpdated
    : Event
    , memory::Recyclable<KeyFramePoseUpdated, memory::ThreadSafePolicy> {
    map::FrameId frame_id;
    geometry::RigidTransformation pose;  // T_cw (world -> camera)

    KeyFramePoseUpdated()
        : Event(EventType::KeyFramePoseUpdated,
                [](Event* e) {
                    static_cast<KeyFramePoseUpdated*>(e)->
                        memory::Recyclable<KeyFramePoseUpdated, memory::ThreadSafePolicy>::Recycle();
                },
                [](Event* e, int n) {
                    static_cast<KeyFramePoseUpdated*>(e)->SetRefCount(n);
                }) {}
};

} // namespace voyahslam::events

#endif // VOYAHSLAM_EVENTS_KEY_FRAME_POSE_UPDATED_H
