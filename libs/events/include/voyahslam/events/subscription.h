#ifndef VOYAHSLAM_EVENTS_SUBSCRIPTION_H
#define VOYAHSLAM_EVENTS_SUBSCRIPTION_H

#include <algorithm>
#include <atomic>
#include <cstdint>
#include <cstdio>
#include <exception>
#include <memory>
#include <utility>
#include <vector>
#include <voyahslam/errors.h>
#include <voyahslam/events/event_types.h>
#include <voyahslam/events/inbox_core.h>
#include <voyahslam/logging/logger.h>

namespace voyahslam::events {

// The lifecycle of an event producer, as far as wiring is concerned. Connections
// change only while the producer is not running (D-05).
enum class RunState : uint8_t { Created, Starting, Running, Stopping, Stopped };

[[nodiscard]] constexpr bool IsRunning(RunState state) {
    return state == RunState::Starting || state == RunState::Running || state == RunState::Stopping;
}

// A producer's connection list and its run state, in a block shared by the
// Observable and every Subscription it handed out (D-22). Because the block is
// shared, a Subscription that outlives its producer only edits this block.
//
// Lock-free by construction: the vectors are written only while the producer is
// not running, and the producer's thread reads them only while it runs. Start
// publishes `state` with a release store before the worker thread exists, and
// Stop's join plus the Stopped store give the reverse edge for later disconnects.
struct Wiring {
    std::atomic<RunState> state{RunState::Created};
    uint64_t next_id = 1;

    struct AsyncEntry {
        uint64_t id;
        std::shared_ptr<InboxCore> core;
    };
    struct SyncEntry {
        uint64_t id;
        void (*fn)(void* ctx, const Event& event);
        void* ctx;
    };
    std::vector<AsyncEntry> async;  // written only while !IsRunning(state)
    std::vector<SyncEntry> sync;

    // Erases the entry `id` from either list; unknown ids are ignored.
    void Remove(uint64_t id) {
        std::erase_if(async, [id](const AsyncEntry& entry) { return entry.id == id; });
        std::erase_if(sync, [id](const SyncEntry& entry) { return entry.id == id; });
    }
};

// One connection from a producer to an observer, as a move-only RAII handle:
// its destructor or reset() disconnects (D-06). Keep it as long as the
// connection should live; discarding the return value of AddObserver
// disconnects at once (hence [[nodiscard]]).
//
// While the producer runs, disconnecting is a usage error (D-05, D-07):
//   - reset() throws UsageError;
//   - the destructor cannot throw, so it writes the reason to stderr and the
//     log and calls std::terminate in every build, a defined, loud failure
//     instead of a later use-after-free.
// A Subscription that outlives its producer, a moved-from one and a second
// reset() are no-ops. Destroy subscriptions after their producer has stopped
// (declare them before the Slam that produces, DESIGN §6).
class [[nodiscard]] Subscription {
public:
    Subscription() = default;
    Subscription(const Subscription&) = delete;
    Subscription& operator=(const Subscription&) = delete;

    Subscription(Subscription&& other) noexcept
        : wiring_(std::move(other.wiring_)), id_(std::exchange(other.id_, 0)) {}

    // Disconnects the current connection first (the rules of reset()).
    Subscription& operator=(Subscription&& other) {
        if (this != &other) {
            reset();
            wiring_ = std::move(other.wiring_);
            id_ = std::exchange(other.id_, 0);
        }
        return *this;
    }

    ~Subscription() {
        if (!wiring_) {
            return;
        }
        if (IsRunning(wiring_->state.load(std::memory_order_acquire))) {
            constexpr const char* kMessage = "Subscription destroyed while its producer runs";
            std::fprintf(stderr, "%s\n", kMessage);
            Logger::CreateLogger("Subscription")->critical(kMessage);
            std::terminate();
        }
        wiring_->Remove(id_);
    }

    // Disconnects now. Throws UsageError while the producer runs; a second call
    // (or a call on a moved-from or default Subscription) does nothing.
    void reset() {
        if (!wiring_) {
            return;
        }
        if (IsRunning(wiring_->state.load(std::memory_order_acquire))) {
            throw UsageError("Subscription::reset() while its producer runs");
        }
        wiring_->Remove(id_);
        wiring_.reset();
        id_ = 0;
    }

    [[nodiscard]] bool connected() const { return wiring_ != nullptr; }

private:
    friend class Observable;
    Subscription(std::shared_ptr<Wiring> wiring, uint64_t id)
        : wiring_(std::move(wiring)), id_(id) {}

    std::shared_ptr<Wiring> wiring_;
    uint64_t id_ = 0;
};

} // namespace voyahslam::events

#endif // VOYAHSLAM_EVENTS_SUBSCRIPTION_H
