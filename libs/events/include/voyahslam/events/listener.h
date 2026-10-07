#ifndef VOYAHSLAM_EVENTS_LISTENER_H
#define VOYAHSLAM_EVENTS_LISTENER_H

#include <atomic>
#include <concurrentqueue.h>
#include <cstdint>
#include <voyahslam/events/event_types.h>

namespace voyahslam::events {

class Listener {
public:
    // `pending_priority` (nullable): the owning inbox's counter, decremented
    // when a priority event is popped.
    explicit Listener(moodycamel::ConcurrentQueue<Event*>& queue,
                      std::atomic<uint32_t>* pending_priority = nullptr)
        : queue_(&queue), pending_priority_(pending_priority) {}

    ~Listener() {
        if (current_) {
            current_->recycle_(current_);
        }
    }

    Listener(const Listener&) = delete;
    Listener& operator=(const Listener&) = delete;

    Listener(Listener&& other) noexcept
        : queue_(other.queue_), pending_priority_(other.pending_priority_), current_(other.current_) {
        other.current_ = nullptr;
    }

    Listener& operator=(Listener&& other) noexcept {
        if (this != &other) {
            if (current_) {
                current_->recycle_(current_);
            }
            queue_ = other.queue_;
            pending_priority_ = other.pending_priority_;
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
        if (queue_->try_dequeue(current_) && current_->priority && pending_priority_) {
            pending_priority_->fetch_sub(1, std::memory_order_acq_rel);
        }
        return current_;
    }

private:
    moodycamel::ConcurrentQueue<Event*>* queue_ = nullptr;
    std::atomic<uint32_t>* pending_priority_ = nullptr;
    Event* current_ = nullptr;
};

} // namespace voyahslam::events

#endif // VOYAHSLAM_EVENTS_LISTENER_H
