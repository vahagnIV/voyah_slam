#ifndef VOYAHSLAM_EVENTS_EVENT_TYPES_H
#define VOYAHSLAM_EVENTS_EVENT_TYPES_H

#include <cstdint>
#include <memory>
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

// An event owns its memory. A producer creates it with std::make_shared<T>(),
// fills it while it holds the mutable std::shared_ptr<T>, and hands it over with
// Observable::Notify(std::move(event)); from then on it is published as an
// EventPtr (shared, const). There is no reuse and no pool: an event stays valid
// for as long as any holder keeps its EventPtr, whichever object (producer,
// subscription, observer, listener) dies first, and it is destroyed on the
// thread of the last drop. So an event's destructor must not depend on the
// destroying thread: keep members to standard containers, Eigen, shared_ptr and
// ColumnBuffer.
struct Event {
    EventType type;
    map::MapId map_id{};  // set by the producer before Notify; immutable afterwards
    // Urgent: counted by the receiving inbox (AsyncObserver::pending_priority).
    // Set by the producer before Notify; immutable afterwards.
    bool priority = false;

protected:
    explicit Event(EventType t) : type(t) {}
    // Non-virtual and protected: events die through their shared_ptr control
    // block, which remembers the concrete type.
    ~Event() = default;
};

// What the delivery layer carries: shared and immutable.
using EventPtr = std::shared_ptr<const Event>;

} // namespace voyahslam::events

#endif // VOYAHSLAM_EVENTS_EVENT_TYPES_H
