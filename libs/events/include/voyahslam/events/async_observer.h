#ifndef VOYAHSLAM_EVENTS_ASYNC_OBSERVER_H
#define VOYAHSLAM_EVENTS_ASYNC_OBSERVER_H

#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <utility>
#include <voyahslam/events/event_types.h>
#include <voyahslam/events/inbox_core.h>
#include <voyahslam/events/listener.h>
#include <voyahslam/events/wake_signal.h>

namespace voyahslam::events {

// The receiving end of event delivery: an inbox that producers push into and one
// consumer thread drains, through GetListener() or TryPop(). Its queue, its
// WakeSignal and its priority count live in a shared InboxCore (D-22), so
// destroying the observer is safe in any order relative to its producers and
// Listeners: queued events are released here (D-08), and a producer that still
// delivers into the closed core just drops the event.
//
// Not copyable or movable: producers' entries and Listeners already share its
// core, and a consumer waits on wake_signal().
class AsyncObserver {
public:
    // `name` identifies the inbox in the "consumer is behind" log warning.
    explicit AsyncObserver(std::string name = "AsyncObserver")
        : core_(std::make_shared<InboxCore>(std::move(name))) {}
    ~AsyncObserver() { core_->Close(); }
    AsyncObserver(const AsyncObserver&) = delete;
    AsyncObserver& operator=(const AsyncObserver&) = delete;

    [[nodiscard]] Listener GetListener() const { return Listener{core_}; }

    // Dequeues into `out_event`; false if empty. Consumers pop through this (or a
    // Listener) so the priority count stays right.
    bool TryPop(EventPtr& out_event) { return core_->TryPop(out_event); }

    // Number of priority events pushed and not yet popped. A cheap lock-free
    // read for the consumer thread (e.g. to cut long work short).
    [[nodiscard]] uint32_t pending_priority() const { return core_->pending_priority(); }
    [[nodiscard]] size_t size_approx() const { return core_->size_approx(); }

    // Notified by every push. A consumer takes a Snapshot(), checks for work and
    // Wait()s; the signal belongs to the inbox, not to the consumer.
    [[nodiscard]] WakeSignal& wake_signal() { return core_->wake_signal(); }
    [[nodiscard]] const std::string& name() const { return core_->name(); }

private:
    friend class Observable;
    std::shared_ptr<InboxCore> core_;
};

} // namespace voyahslam::events

#endif // VOYAHSLAM_EVENTS_ASYNC_OBSERVER_H
