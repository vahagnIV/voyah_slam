#ifndef VOYAHSLAM_EVENTS_ASYNC_OBSERVER_H
#define VOYAHSLAM_EVENTS_ASYNC_OBSERVER_H

#include <concurrentqueue.h>
#include <voyahslam/events/event_types.h>
#include <voyahslam/events/listener.h>
#include <voyahslam/events/wake_signal.h>

namespace voyahslam::events {

class AsyncObserver {
public:
    AsyncObserver() = default;
    AsyncObserver(const AsyncObserver&) = delete;
    AsyncObserver& operator=(const AsyncObserver&) = delete;

    [[nodiscard]] Listener GetListener() { return Listener{queue_}; }
    [[nodiscard]] moodycamel::ConcurrentQueue<Event*>& queue() { return queue_; }

    // Nullable; must be set before producers start pushing.
    void SetWakeSignal(WakeSignal* signal) { wake_signal_ = signal; }

    // Enqueues `event`, then wakes the attached signal (if any).
    void Push(Event* event) {
        queue_.enqueue(event);
        if (wake_signal_) {
            wake_signal_->Notify();
        }
    }

private:
    moodycamel::ConcurrentQueue<Event*> queue_;
    WakeSignal* wake_signal_ = nullptr;
};

} // namespace voyahslam::events

#endif // VOYAHSLAM_EVENTS_ASYNC_OBSERVER_H
