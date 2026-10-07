#ifndef VOYAHSLAM_EVENTS_MAP_POINTS_REMOVED_H
#define VOYAHSLAM_EVENTS_MAP_POINTS_REMOVED_H

#include <vector>
#include <voyahslam/events/event_types.h>
#include <voyahslam/map/map_point_id.h>
#include <voyahslam/memory/recyclable.h>

namespace voyahslam::events {

struct MapPointsRemoved
    : Event
    , memory::Recyclable<MapPointsRemoved, memory::ThreadSafePolicy> {
    std::vector<map::MapPointId> ids;

    void Reset() { ids.clear(); }

    MapPointsRemoved()
        : Event(EventType::MapPointsRemoved,
                [](Event* e) {
                    static_cast<MapPointsRemoved*>(e)->
                        memory::Recyclable<MapPointsRemoved, memory::ThreadSafePolicy>::Recycle();
                },
                [](Event* e, int n) {
                    static_cast<MapPointsRemoved*>(e)->SetRefCount(n);
                }) {}
};

} // namespace voyahslam::events

#endif // VOYAHSLAM_EVENTS_MAP_POINTS_REMOVED_H
