#ifndef VOYAHSLAM_EVENTS_EVENT_TYPES_H
#define VOYAHSLAM_EVENTS_EVENT_TYPES_H

#include <cstdint>

namespace voyahslam::events {

enum class EventType : uint8_t {
    NewKeyFrameCandidate,
    MapPointsAdded,
    MapPointsUpdated,
    MapPointsRemoved,
    KeyFrameCreated,
    KeyFramePoseUpdated,
    LoopDetected,
    PositionUpdated,
};

struct Event {
    EventType type;
    void (*recycle_)(Event*);
    void (*set_ref_count_)(Event*, int);

protected:
    Event(EventType t, void (*r)(Event*), void (*s)(Event*, int))
        : type(t), recycle_(r), set_ref_count_(s) {}
};

} // namespace voyahslam::events

#endif // VOYAHSLAM_EVENTS_EVENT_TYPES_H
