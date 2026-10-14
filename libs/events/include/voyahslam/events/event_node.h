#ifndef VOYAHSLAM_EVENTS_EVENT_NODE_H
#define VOYAHSLAM_EVENTS_EVENT_NODE_H

#include <cstddef>
#include <string>
#include <utility>
#include <voyahslam/errors.h>
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
//
// Lifecycle (D-09..D-12): Created -> Starting -> Running -> Stopping -> Stopped,
// driven by StartWith() / StopWith() with an atomic compare-and-swap, so Start()
// and Stop() are safe from any thread. Stopped is terminal: a second Start() and
// a Start() after Stop() throw voyahslam::UsageError, Stop() before Start() moves
// Created to Stopped, and Stop() is idempotent (destructors call it).
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

    // Created -> Starting -> start_worker() -> Running. Any other state throws
    // UsageError ("Start() called twice" / "Start() after Stop(): create a new
    // component"). The Starting store happens before the worker thread exists, so
    // the thread start is the happens-before edge for the wiring. If start_worker
    // throws, the state becomes Stopped and the exception propagates.
    // Removed with the last caller in plan 01-12 task 2.
    void BeginRun() { observable_.SetState(RunState::Running); }
    void EndRun() { observable_.SetState(RunState::Stopped); }

    template <typename TStartWorker>
    void StartWith(TStartWorker&& start_worker) {
        RunState expected = RunState::Created;
        if (!observable_.CompareExchangeState(expected, RunState::Starting)) {
            throw UsageError(expected == RunState::Stopped
                                 ? "Start() after Stop(): create a new component"
                                 : "Start() called twice");
        }
        try {
            start_worker();
        } catch (...) {
            observable_.SetState(RunState::Stopped);
            throw;
        }
        observable_.SetState(RunState::Running);
    }

    // Created -> Stopped; Running -> Stopping -> stop_worker() -> Stopped (the
    // worker is joined by then); Starting or Stopping: waits until the other
    // thread settled the state and retries, so every concurrent Stop() returns
    // only after the worker is joined; Stopped: returns. If stop_worker throws
    // (a Stop() on the component's own worker thread), the state goes back to
    // Running and the exception propagates.
    template <typename TStopWorker>
    void StopWith(TStopWorker&& stop_worker) {
        while (true) {
            RunState state = observable_.state();
            switch (state) {
                case RunState::Stopped:
                    return;
                case RunState::Starting:
                case RunState::Stopping:
                    observable_.WaitWhileState(state);
                    break;
                case RunState::Created:
                    if (observable_.CompareExchangeState(state, RunState::Stopped)) return;
                    break;
                case RunState::Running:
                    if (!observable_.CompareExchangeState(state, RunState::Stopping)) break;
                    try {
                        stop_worker();
                    } catch (...) {
                        observable_.SetState(RunState::Running);
                        throw;
                    }
                    observable_.SetState(RunState::Stopped);
                    return;
            }
        }
    }

    Observable observable_;
    AsyncObserver inbox_;
};

} // namespace voyahslam::events

#endif // VOYAHSLAM_EVENTS_EVENT_NODE_H
