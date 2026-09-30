#ifndef VOYAHSLAM_EVENTS_LISTENER_H
#define VOYAHSLAM_EVENTS_LISTENER_H

#include <concurrentqueue.h>
#include <voyahslam/events/event_types.h>

namespace voyahslam::events {

class Listener {
public:
    explicit Listener(moodycamel::ConcurrentQueue<Event*>& queue)
        : queue_(&queue) {}

    ~Listener() {
        if (current_) {
            current_->recycle_(current_);
        }
    }

    Listener(const Listener&) = delete;
    Listener& operator=(const Listener&) = delete;

    Listener(Listener&& other) noexcept
        : queue_(other.queue_), current_(other.current_) {
        other.current_ = nullptr;
    }

    Listener& operator=(Listener&& other) noexcept {
        if (this != &other) {
            if (current_) {
                current_->recycle_(current_);
            }
            queue_ = other.queue_;
            current_ = other.current_;
            other.current_ = nullptr;
        }
        return *this;
    }

    Event* GetNextEvent() {
        if (current_) {
            current_->recycle_(current_);
            current_ = nullptr;
        }
        queue_->try_dequeue(current_);
        return current_;
    }

private:
    moodycamel::ConcurrentQueue<Event*>* queue_ = nullptr;
    Event* current_ = nullptr;
};

} // namespace voyahslam::events

#endif // VOYAHSLAM_EVENTS_LISTENER_H
