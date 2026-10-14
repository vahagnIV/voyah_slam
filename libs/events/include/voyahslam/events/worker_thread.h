#ifndef VOYAHSLAM_EVENTS_WORKER_THREAD_H
#define VOYAHSLAM_EVENTS_WORKER_THREAD_H

#include <voyahslam/events/wake_signal.h>

#include <atomic>
#include <functional>
#include <thread>

namespace voyahslam::events {

// Runs `process_pending` on a dedicated thread. The callback returns true if it
// did work (it is called again immediately) and false if there was nothing to
// do (the thread sleeps on the WakeSignal until it is notified). A restartable
// primitive: Start() and Stop() must be called from one controlling thread at a
// time. The component lifecycle (EventNode::StartWith / StopWith) serializes
// them and makes Stopped terminal on top of it (D-26). Stop() on the worker's
// own thread throws UsageError instead of joining itself.
class WorkerThread {
public:
    explicit WorkerThread(WakeSignal& signal);
    ~WorkerThread();

    WorkerThread(const WorkerThread&) = delete;
    WorkerThread& operator=(const WorkerThread&) = delete;

    // No-op if already running.
    void Start(std::function<bool()> process_pending);
    // Idempotent; wakes the thread and joins it. Throws UsageError when called on
    // the worker thread itself; the worker keeps running.
    void Stop();

    [[nodiscard]] bool running() const { return running_.load(std::memory_order_acquire); }

private:
    WakeSignal& signal_;
    std::atomic<bool> running_{false};
    // The worker thread's id while it runs (stored by the thread as its first
    // action, cleared when it exits); a default id otherwise.
    std::atomic<std::thread::id> worker_id_{};
    std::thread thread_;
};

} // namespace voyahslam::events

#endif // VOYAHSLAM_EVENTS_WORKER_THREAD_H
