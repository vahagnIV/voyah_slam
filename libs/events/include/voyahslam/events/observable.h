#ifndef VOYAHSLAM_EVENTS_OBSERVABLE_H
#define VOYAHSLAM_EVENTS_OBSERVABLE_H

#include <cstddef>
#include <memory>
#include <string>
#include <type_traits>
#include <utility>
#include <voyahslam/errors.h>
#include <voyahslam/events/async_observer.h>
#include <voyahslam/events/event_types.h>
#include <voyahslam/events/subscription.h>

namespace voyahslam::events {

// The publishing end of event delivery. Connections live in a Wiring block
// shared with every Subscription handed out, so a Subscription outliving this
// Observable is a no-op, and every AsyncObserver is reached through its shared
// InboxCore, so an observer destroyed first is just a closed inbox (D-22).
//
// Connecting is allowed only while the producer is not running (D-05): the
// owner (EventNode) drives the run state with SetState(). A bare Observable,
// as used in tests, stays in Created unless SetState() is called.
class Observable {
public:
    Observable() : wiring_(std::make_shared<Wiring>()) {}
    Observable(const Observable&) = delete;
    Observable& operator=(const Observable&) = delete;

    // Nothing publishes any more: the connection list may be edited by the
    // Subscriptions that outlive this object.
    ~Observable() { wiring_->state.store(RunState::Stopped, std::memory_order_release); }

    // Delivers every event published from now on to `observer`'s inbox, until
    // the returned Subscription is reset or destroyed. Throws UsageError while
    // the producer runs (D-05).
    [[nodiscard]] Subscription ConnectAsync(AsyncObserver& observer) {
        ThrowIfRunning("ConnectAsync");
        const uint64_t id = wiring_->next_id++;
        wiring_->async.push_back({id, observer.core_});
        return Subscription{wiring_, id};
    }

    // Calls `fn(ctx, event)` inline, on the publishing thread, for every event
    // published. The handler must not retain the event reference (take an
    // EventPtr from an async observer for that). Throws UsageError while the
    // producer runs.
    //
    // A sync handler's `ctx` must outlive its connection: keep it valid until
    // the returned Subscription is reset or destroyed (while the producer is
    // stopped), or until the producer is destroyed; the async side has no such
    // rule (D-22).
    [[nodiscard]] Subscription ConnectSync(void (*fn)(void* ctx, const Event& event), void* ctx) {
        ThrowIfRunning("ConnectSync");
        const uint64_t id = wiring_->next_id++;
        wiring_->sync.push_back({id, fn, ctx});
        return Subscription{wiring_, id};
    }

    // Publishes `event`: sync handlers run inline first, then a copy of the
    // pointer is pushed into every async observer's inbox. The event becomes
    // immutable (EventPtr is shared and const) and the caller's pointer is moved
    // from, so the producer cannot keep a mutable alias; an lvalue does not
    // compile. With no observers connected the event is freed before Notify
    // returns. A consumer acknowledges by dropping its copy.
    //
    // The entry vectors are read here without synchronization: they change only
    // while the producer is not running, and Notify runs on the producer's
    // thread while it runs.
    template <typename T>
    void Notify(std::shared_ptr<T>&& event) {
        static_assert(std::is_base_of_v<Event, T>, "Notify publishes events only");
        const EventPtr published = std::move(event);
        for (const auto& handler : wiring_->sync) {
            handler.fn(handler.ctx, *published);
        }
        for (const auto& entry : wiring_->async) {
            entry.core->Push(published);
        }
    }

    [[nodiscard]] size_t async_count() const { return wiring_->async.size(); }

    [[nodiscard]] RunState state() const { return wiring_->state.load(std::memory_order_acquire); }
    // Release store; called by the owning EventNode only.
    void SetState(RunState state) { wiring_->state.store(state, std::memory_order_release); }

private:
    void ThrowIfRunning(const char* what) const {
        if (IsRunning(state())) {
            throw UsageError(std::string(what) + " while the producer runs");
        }
    }

    std::shared_ptr<Wiring> wiring_;
};

} // namespace voyahslam::events

#endif // VOYAHSLAM_EVENTS_OBSERVABLE_H
