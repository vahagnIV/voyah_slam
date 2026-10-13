#ifndef VOYAHSLAM_EVENTS_LISTENER_H
#define VOYAHSLAM_EVENTS_LISTENER_H

#include <memory>
#include <utility>
#include <voyahslam/events/event_types.h>
#include <voyahslam/events/inbox_core.h>

namespace voyahslam::events {

// A handle on an AsyncObserver's inbox: it shares the InboxCore, so it stays
// valid (and returns an empty EventPtr) if the observer is destroyed first. It
// is freely copyable; every copy pops from the same queue.
class Listener {
public:
    explicit Listener(std::shared_ptr<InboxCore> core) : core_(std::move(core)) {}

    // The next queued event, or an empty pointer when there is none. The caller
    // owns the returned pointer: dropping it is the acknowledgement, so
    //   while (EventPtr event = listener.GetNextEvent()) { Apply(*event); }
    // releases each event at the end of its iteration.
    EventPtr GetNextEvent() {
        EventPtr event;
        core_->TryPop(event);
        return event;
    }

private:
    std::shared_ptr<InboxCore> core_;
};

} // namespace voyahslam::events

#endif // VOYAHSLAM_EVENTS_LISTENER_H
