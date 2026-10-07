#ifndef VOYAHSLAM_EVENTS_EVENT_TYPES_H
#define VOYAHSLAM_EVENTS_EVENT_TYPES_H

#include <cstdint>
#include <voyahslam/map/map_id.h>

namespace voyahslam::events {

enum class EventType : uint8_t {
    MapPointsUpdated,
    MapPointsRemoved,
    KeyFrameCreated,
    KeyFramePoseUpdated,
    LoopDetected,
    FrameTracked,
    InitializationMeasurement,
    MapCreated,
    TrackingLost,  // keep last: consumers size per-type tables by it
};

struct Event {
    EventType type;
    void (*recycle_)(Event*);
    void (*set_ref_count_)(Event*, int);
    map::MapId map_id{};  // producers set on every publish (pooled events are not reset)

protected:
    Event(EventType t, void (*r)(Event*), void (*s)(Event*, int))
        : type(t), recycle_(r), set_ref_count_(s) {}
};

} // namespace voyahslam::events

#endif // VOYAHSLAM_EVENTS_EVENT_TYPES_H
