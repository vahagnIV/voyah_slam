#ifndef VOYAHSLAM_EVENTS_INBOX_CORE_H
#define VOYAHSLAM_EVENTS_INBOX_CORE_H

#include <concurrentqueue.h>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <string>
#include <utility>
#include <voyahslam/events/event_types.h>
#include <voyahslam/events/wake_signal.h>
#include <voyahslam/logging/logger.h>

namespace voyahslam::events {

// An inbox that reaches this many queued events logs a "consumer is behind"
// warning, and again each time its size doubles (D-24).
inline constexpr size_t kBacklogWarningFloor = 1024;

// The shared state of one AsyncObserver: the queue of events, the count of
// priority events in it, the observer's WakeSignal and a closed flag. It is held
// by the AsyncObserver, by every Listener made from it and by every producer
// entry that delivers into it, so any of them may die first: the core lives
// until the last holder is gone (DESIGN §6, D-22). It takes no lock: the queue
// is lock-free and everything else is an atomic.
//
// An observer destroyed before its producer closes the core (Close()): queued
// events are released and later pushes drop their event at once. Only a closed
// inbox drops; a live, connected inbox never does (lossy delivery is EVT-01).
//
// Backlog visibility (D-24): without event pools a stalled consumer grows memory
// silently, so Push logs one warning naming the observer when the queue first
// reaches kBacklogWarningFloor events and again at each doubling (2048, 4096,
// ...), never once per event. The threshold only grows: an inbox that is drained
// and refilled warns again only past the last threshold.
class InboxCore {
public:
    explicit InboxCore(std::string name) : name_(std::move(name)) {}
    InboxCore(const InboxCore&) = delete;
    InboxCore& operator=(const InboxCore&) = delete;

    // Closed: drops `event` (this copy of the pointer). Otherwise counts it if
    // it is a priority event (before the enqueue, so a pop never sees the count
    // behind the queue), enqueues it and wakes the signal's waiter.
    void Push(EventPtr event) {
        if (closed_.load(std::memory_order_acquire)) {
            return;
        }
        if (event->priority) {
            pending_priority_.fetch_add(1, std::memory_order_acq_rel);
        }
        queue_.enqueue(std::move(event));
        WarnIfBehind();
        wake_signal_.Notify();
    }

    // Dequeues into `out_event`; false if empty.
    bool TryPop(EventPtr& out_event) {
        if (!queue_.try_dequeue(out_event)) {
            return false;
        }
        if (out_event->priority) {
            pending_priority_.fetch_sub(1, std::memory_order_acq_rel);
        }
        return true;
    }

    // Called by ~AsyncObserver only: no later Push enqueues, and every queued
    // event is released on the calling thread (D-08). A Push that passed the
    // closed check just before this runs may leave one event in a closed core;
    // it is valid and freed when the core goes.
    void Close() {
        closed_.store(true, std::memory_order_release);
        EventPtr dropped;
        while (TryPop(dropped)) {
            dropped.reset();
        }
    }

    [[nodiscard]] bool closed() const { return closed_.load(std::memory_order_acquire); }
    // Priority events pushed and not yet popped: a cheap lock-free read.
    [[nodiscard]] uint32_t pending_priority() const {
        return pending_priority_.load(std::memory_order_acquire);
    }
    [[nodiscard]] size_t size_approx() const { return queue_.size_approx(); }
    [[nodiscard]] WakeSignal& wake_signal() { return wake_signal_; }
    [[nodiscard]] const std::string& name() const { return name_; }

private:
    // Relaxed ordering is enough: the threshold only rate-limits a log line.
    void WarnIfBehind() {
        size_t threshold = warn_at_.load(std::memory_order_relaxed);
        const size_t queued = queue_.size_approx();
        if (queued >= threshold &&
            warn_at_.compare_exchange_strong(threshold, threshold * 2, std::memory_order_relaxed)) {
            Logger::CreateLogger("AsyncObserver")
                ->warn("{}: consumer is behind, {} events queued", name_, queued);
        }
    }

    std::string name_;
    moodycamel::ConcurrentQueue<EventPtr> queue_;
    std::atomic<uint32_t> pending_priority_{0};
    std::atomic<bool> closed_{false};
    std::atomic<size_t> warn_at_{kBacklogWarningFloor};
    WakeSignal wake_signal_;
};

} // namespace voyahslam::events

#endif // VOYAHSLAM_EVENTS_INBOX_CORE_H
