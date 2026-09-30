#ifndef VOYAHSLAM_EVENTS_OBSERVABLE_H
#define VOYAHSLAM_EVENTS_OBSERVABLE_H

#include <vector>
#include <voyahslam/events/async_observer.h>
#include <voyahslam/events/event_types.h>

namespace voyahslam::events {

class Observable {
public:
    Observable() = default;

    void ConnectAsync(AsyncObserver& observer) {
        async_queues_.push_back(&observer.queue());
    }

    void ConnectSync(void (*fn)(void*, Event*), void* ctx) {
        sync_handlers_.push_back({fn, ctx});
    }

    void Notify(Event* event) {
        for (auto& handler : sync_handlers_) {
            handler.fn(handler.ctx, event);
        }

        if (!async_queues_.empty()) {
            event->set_ref_count_(event, static_cast<int>(async_queues_.size()));
            for (auto* queue : async_queues_) {
                queue->enqueue(event);
            }
        }
    }

    [[nodiscard]] size_t async_count() const { return async_queues_.size(); }

private:
    struct SyncHandler {
        void (*fn)(void* ctx, Event*);
        void* ctx;
    };

    std::vector<moodycamel::ConcurrentQueue<Event*>*> async_queues_;
    std::vector<SyncHandler> sync_handlers_;
};

} // namespace voyahslam::events

#endif // VOYAHSLAM_EVENTS_OBSERVABLE_H
