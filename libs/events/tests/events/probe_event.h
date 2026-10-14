#ifndef VOYAHSLAM_EVENTS_TESTS_PROBE_EVENT_H
#define VOYAHSLAM_EVENTS_TESTS_PROBE_EVENT_H

// Test-only helpers: an event that counts live instances, and an EventNode whose
// run state and publishing a test can drive.

#include <voyahslam/events/event_node.h>
#include <voyahslam/events/event_types.h>

#include <atomic>
#include <memory>
#include <string>
#include <thread>
#include <utility>

namespace voyahslam::events {

// Counts live instances in `alive` (every test expects it back at 0 once the
// events it made are dropped) and can record the thread that destroyed it.
struct ProbeEvent : Event {
    static inline std::atomic<int> alive{0};

    int value = 0;
    std::thread::id* destroyed_on = nullptr;  // written by the destructor if set

    ProbeEvent() : Event(EventType::TrackingLost) { alive.fetch_add(1); }
    ~ProbeEvent() {
        if (destroyed_on) {
            *destroyed_on = std::this_thread::get_id();
        }
        alive.fetch_sub(1);
    }
};

inline std::shared_ptr<ProbeEvent> MakeProbe(int value = 0, bool priority = false) {
    auto event = std::make_shared<ProbeEvent>();
    event->value = value;
    event->priority = priority;
    return event;
}

// A producer: Start() / Stop() only drive the run state (StartWith / StopWith with
// no worker), so D-05 and D-07 are testable.
class TestNode : public EventNode {
public:
    explicit TestNode(std::string inbox_name = "TestNode") : EventNode(std::move(inbox_name)) {}

    void Publish(std::shared_ptr<ProbeEvent> event) { observable_.Notify(std::move(event)); }
    void Start() {
        StartWith([] {});
    }
    void Stop() {
        StopWith([] {});
    }
};

}  // namespace voyahslam::events

#endif  // VOYAHSLAM_EVENTS_TESTS_PROBE_EVENT_H
