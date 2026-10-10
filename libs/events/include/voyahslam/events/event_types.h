#ifndef VOYAHSLAM_EVENTS_EVENT_TYPES_H
#define VOYAHSLAM_EVENTS_EVENT_TYPES_H

#include <cstdint>
#include <voyahslam/map/map_id.h>

namespace voyahslam::events {

enum class EventType : uint8_t {
    MapTransaction,  // every Mapper change (map_transaction.h)
    LoopDetected,
    FrameTracked,
    InitializationMeasurement,
    GlobalBAFinished,
    TrackingLost,  // keep last: consumers size per-type tables by it
};

struct Event {
    EventType type;
    void (*recycle_)(Event*);
    void (*set_ref_count_)(Event*, int);
    map::MapId map_id{};  // producers set on every publish (pooled events are not reset)
    // Urgent: counted by the receiving inbox (AsyncObserver::pending_priority).
    // Producers set it on every publish (not reset on reuse) and must not change
    // it while the event is published.
    bool priority = false;

protected:
    Event(EventType t, void (*r)(Event*), void (*s)(Event*, int))
        : type(t), recycle_(r), set_ref_count_(s) {}
};

} // namespace voyahslam::events

#endif // VOYAHSLAM_EVENTS_EVENT_TYPES_H
