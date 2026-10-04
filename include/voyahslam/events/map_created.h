#ifndef VOYAHSLAM_EVENTS_MAP_CREATED_H
#define VOYAHSLAM_EVENTS_MAP_CREATED_H

#include <voyahslam/defs.h>
#include <voyahslam/events/event_types.h>
#include <voyahslam/geometry/rigid_transformation.h>
#include <voyahslam/map/frame_id.h>
#include <voyahslam/memory/recyclable.h>

namespace voyahslam::events {

struct MapCreated
    : Event
    , memory::Recyclable<MapCreated, memory::ThreadSafePolicy> {
    map::FrameId frame_id;
    geometry::RigidTransformation pose;

    MapCreated()
        : Event(EventType::MapCreated,
                [](Event* e) {
                    static_cast<MapCreated*>(e)->
                        memory::Recyclable<MapCreated, memory::ThreadSafePolicy>::Recycle();
                },
                [](Event* e, int n) {
                    static_cast<MapCreated*>(e)->SetRefCount(n);
                }) {}
};

} // namespace voyahslam::events

#endif // VOYAHSLAM_EVENTS_MAP_CREATED_H
