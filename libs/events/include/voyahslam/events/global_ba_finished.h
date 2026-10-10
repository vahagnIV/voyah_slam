#ifndef VOYAHSLAM_EVENTS_GLOBAL_BA_FINISHED_H
#define VOYAHSLAM_EVENTS_GLOBAL_BA_FINISHED_H

#include <cstdint>
#include <utility>
#include <vector>
#include <voyahslam/defs.h>
#include <voyahslam/events/event_types.h>
#include <voyahslam/geometry/rigid_transformation.h>
#include <voyahslam/map/frame_id.h>
#include <voyahslam/map/map_point_id.h>
#include <voyahslam/memory/recyclable.h>

namespace voyahslam::events {

// Published by the GlobalBA thread: the global BA of map Event::map_id as its
// replica stood at correction epoch `map_epoch`. Plain data only: the
// optimized pose (T_cw) of every Frame of the problem and the optimized world
// position of every point.
struct GlobalBAFinished
    : Event
    , memory::Recyclable<GlobalBAFinished, memory::ThreadSafePolicy> {
    uint32_t map_epoch = 0;
    std::vector<std::pair<map::FrameId, geometry::RigidTransformation>> frame_poses;  // T_cw
    std::vector<std::pair<map::MapPointId, geometry::TPoint3DF>> point_positions;     // world

    // Called by the pool on reuse.
    void Reset() {
        map_epoch = 0;
        frame_poses.clear();
        point_positions.clear();
        priority = false;
    }

    GlobalBAFinished()
        : Event(EventType::GlobalBAFinished,
                [](Event* e) {
                    static_cast<GlobalBAFinished*>(e)->
                        memory::Recyclable<GlobalBAFinished, memory::ThreadSafePolicy>::Recycle();
                },
                [](Event* e, int n) {
                    static_cast<GlobalBAFinished*>(e)->SetRefCount(n);
                }) {}
};

} // namespace voyahslam::events

#endif // VOYAHSLAM_EVENTS_GLOBAL_BA_FINISHED_H
