#ifndef VOYAHSLAM_EVENTS_WAKE_SIGNAL_H
#define VOYAHSLAM_EVENTS_WAKE_SIGNAL_H

#include <atomic>
#include <cstdint>

namespace voyahslam::events {

// Lock-free wake-up primitive for idle-sleeping worker threads. A waiter takes
// a Snapshot(), checks for work, and calls Wait(snapshot) if there was none; a
// Notify() between the snapshot and the wait changes the counter, so Wait
// returns immediately (no lost wake-up).
class WakeSignal {
public:
    [[nodiscard]] uint32_t Snapshot() const {
        return counter_.load(std::memory_order_acquire);
    }

    void Notify() {
        counter_.fetch_add(1, std::memory_order_release);
        counter_.notify_one();
    }

    // Blocks until the counter differs from `seen`.
    void Wait(uint32_t seen) const {
        counter_.wait(seen, std::memory_order_acquire);
    }

private:
    std::atomic<uint32_t> counter_{0};
};

} // namespace voyahslam::events

#endif // VOYAHSLAM_EVENTS_WAKE_SIGNAL_H
