#ifndef VOYAHSLAM_EVENTS_ASYNC_OBSERVER_H
#define VOYAHSLAM_EVENTS_ASYNC_OBSERVER_H

#include <concurrentqueue.h>
#include <atomic>
#include <cstdint>
#include <voyahslam/events/event_types.h>
#include <voyahslam/events/listener.h>
#include <voyahslam/events/wake_signal.h>

namespace voyahslam::events {

class AsyncObserver {
public:
    AsyncObserver() = default;
    AsyncObserver(const AsyncObserver&) = delete;
    AsyncObserver& operator=(const AsyncObserver&) = delete;

    [[nodiscard]] Listener GetListener() { return Listener{queue_, &pending_priority_}; }
    [[nodiscard]] moodycamel::ConcurrentQueue<Event*>& queue() { return queue_; }

    // Nullable; must be set before producers start pushing.
    void SetWakeSignal(WakeSignal* signal) { wake_signal_ = signal; }

    // Number of priority events pushed and not yet popped. A cheap lock-free
    // read for the consumer thread (e.g. to cut long work short).
    [[nodiscard]] uint32_t pending_priority() const {
        return pending_priority_.load(std::memory_order_acquire);
    }
    [[nodiscard]] const std::atomic<uint32_t>& pending_priority_counter() const {
        return pending_priority_;
    }

    // Dequeues into `out_event`; false if empty. Consumers must pop through this
    // (or a Listener) so the priority count stays right.
    bool TryPop(Event*& out_event) {
        if (!queue_.try_dequeue(out_event)) {
            return false;
        }
        // The flag is stable from push to pop: it is set before publishing and
        // the event is not recycled (so not reused) until every observer has
        // popped and released it.
        if (out_event->priority) {
            pending_priority_.fetch_sub(1, std::memory_order_acq_rel);
        }
        return true;
    }

    // Enqueues `event`, then wakes the attached signal (if any).
    void Push(Event* event) {
        // Counted before the enqueue, so a pop never sees the count behind it.
        if (event->priority) {
            pending_priority_.fetch_add(1, std::memory_order_acq_rel);
        }
        queue_.enqueue(event);
        if (wake_signal_) {
            wake_signal_->Notify();
        }
    }

private:
    moodycamel::ConcurrentQueue<Event*> queue_;
    std::atomic<uint32_t> pending_priority_{0};
    WakeSignal* wake_signal_ = nullptr;
};

} // namespace voyahslam::events

#endif // VOYAHSLAM_EVENTS_ASYNC_OBSERVER_H
