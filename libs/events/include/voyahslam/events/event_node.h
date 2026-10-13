#ifndef VOYAHSLAM_EVENTS_EVENT_NODE_H
#define VOYAHSLAM_EVENTS_EVENT_NODE_H

#include <cstddef>
#include <string>
#include <utility>
#include <voyahslam/events/async_observer.h>
#include <voyahslam/events/event_types.h>
#include <voyahslam/events/observable.h>
#include <voyahslam/events/subscription.h>
#include <voyahslam/events/wake_signal.h>

namespace voyahslam::events {

// Connection points of a component that runs its own loop (Tracker, Mapper,
// LoopCloser): it publishes through an Observable and receives through a single
// AsyncObserver, its inbox. The inbox owns the WakeSignal that every push
// notifies, so the component's worker thread sleeps on wake_signal() while there
// is nothing to do.
//
// Wiring: AddObserver / AddSyncObserver return a Subscription; keep it as long as
// the connection should live. Connect and disconnect only while the component is
// not running (D-05): connecting then throws voyahslam::UsageError, so does
// Subscription::reset(), and destroying a Subscription while the component runs
// terminates. A Subscription may outlive the component (then it does nothing).
// Whether a stopped component may start again is decided by the component
// (the terminal Stopped rule arrives with the lifecycle work, plan 01-12), not
// here: BeginRun() and EndRun() only publish the run state.
class EventNode {
public:
    explicit EventNode(std::string inbox_name = "EventNode") : inbox_(std::move(inbox_name)) {}
    EventNode(const EventNode&) = delete;
    EventNode& operator=(const EventNode&) = delete;

    // Delivers every event this component publishes to `observer`'s queue.
    // Throws UsageError while the component runs.
    [[nodiscard]] Subscription AddObserver(AsyncObserver& observer) {
        return observable_.ConnectAsync(observer);
    }
    // Calls `handler(ctx, event)` inline, on this component's thread, for every
    // event it publishes. The handler must not retain the event. Throws
    // UsageError while the component runs.
    //
    // A sync handler's `ctx` must outlive its connection: keep it valid until
    // the returned Subscription is reset or destroyed (while the component is
    // stopped), or until the component is destroyed; the async side has no such
    // rule (D-22).
    [[nodiscard]] Subscription AddSyncObserver(void (*handler)(void* ctx, const Event& event),
                                               void* ctx) {
        return observable_.ConnectSync(handler, ctx);
    }

    // The queue other components publish into.
    [[nodiscard]] AsyncObserver& inbox() { return inbox_; }
    [[nodiscard]] size_t async_observer_count() const { return observable_.async_count(); }
    [[nodiscard]] RunState run_state() const { return observable_.state(); }

protected:
    // Notified by the inbox and by any other input the component has. Owned by
    // the inbox's core, so it lives as long as the inbox does.
    [[nodiscard]] WakeSignal& wake_signal() { return inbox_.wake_signal(); }

    // Mark the component running (before its worker thread is created, so the
    // thread start is the happens-before edge for the wiring) and stopped (after
    // the worker thread is joined).
    void BeginRun() { observable_.SetState(RunState::Running); }
    void EndRun() { observable_.SetState(RunState::Stopped); }

    Observable observable_;
    AsyncObserver inbox_;
};

} // namespace voyahslam::events

#endif // VOYAHSLAM_EVENTS_EVENT_NODE_H
