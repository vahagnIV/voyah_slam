#ifndef VOYAHSLAM_EVENTS_EVENT_NODE_H
#define VOYAHSLAM_EVENTS_EVENT_NODE_H

#include <cstddef>
#include <voyahslam/events/async_observer.h>
#include <voyahslam/events/event_types.h>
#include <voyahslam/events/observable.h>
#include <voyahslam/events/wake_signal.h>

namespace voyahslam::events {

// Connection points of a component that runs its own loop (Tracker, Mapper,
// LoopCloser): it publishes through an Observable and receives through a single
// AsyncObserver, its inbox. Wiring is not thread-safe — connect everything
// before Start(). The inbox notifies the node's WakeSignal on every push, so the
// component's worker thread sleeps on it while there is nothing to do.
class EventNode {
public:
    EventNode() { inbox_.SetWakeSignal(&wake_signal_); }
    EventNode(const EventNode&) = delete;
    EventNode& operator=(const EventNode&) = delete;

    // Delivers every event this component publishes to `observer`'s queue.
    void AddObserver(AsyncObserver& observer) { observable_.ConnectAsync(observer); }
    // Calls `handler(ctx, event)` inline, on this component's thread, for every
    // event it publishes. The handler must not retain the event.
    void AddSyncObserver(void (*handler)(void* ctx, Event*), void* ctx) {
        observable_.ConnectSync(handler, ctx);
    }

    // The queue other components publish into.
    [[nodiscard]] AsyncObserver& inbox() { return inbox_; }
    [[nodiscard]] size_t async_observer_count() const { return observable_.async_count(); }

protected:
    // Notified by the inbox and by any other input the component has.
    [[nodiscard]] WakeSignal& wake_signal() { return wake_signal_; }

    Observable observable_;
    AsyncObserver inbox_;

private:
    WakeSignal wake_signal_;
};

} // namespace voyahslam::events

#endif // VOYAHSLAM_EVENTS_EVENT_NODE_H
