#ifndef VOYAHSLAM_EVENTS_ASYNC_OBSERVER_H
#define VOYAHSLAM_EVENTS_ASYNC_OBSERVER_H

#include <concurrentqueue.h>
#include <voyahslam/events/event_types.h>
#include <voyahslam/events/listener.h>

namespace voyahslam::events {

class AsyncObserver {
public:
    AsyncObserver() = default;
    AsyncObserver(const AsyncObserver&) = delete;
    AsyncObserver& operator=(const AsyncObserver&) = delete;

    [[nodiscard]] Listener GetListener() { return Listener{queue_}; }
    [[nodiscard]] moodycamel::ConcurrentQueue<Event*>& queue() { return queue_; }

private:
    moodycamel::ConcurrentQueue<Event*> queue_;
};

} // namespace voyahslam::events

#endif // VOYAHSLAM_EVENTS_ASYNC_OBSERVER_H
