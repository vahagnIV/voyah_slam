#include <voyahslam/events/worker_thread.h>

#include <cstdint>
#include <utility>
#include <voyahslam/errors.h>

namespace voyahslam::events {

WorkerThread::WorkerThread(WakeSignal& signal) : signal_(signal) {}

WorkerThread::~WorkerThread() { Stop(); }

void WorkerThread::Start(std::function<bool()> process_pending) {
    if (running()) return;
    if (thread_.joinable()) thread_.join();
    running_.store(true, std::memory_order_release);
    try {
        thread_ = std::thread([this, process_pending = std::move(process_pending)] {
            worker_id_.store(std::this_thread::get_id(), std::memory_order_release);
            // Snapshot before checking running_: a Stop() between the check and the
            // snapshot would otherwise be swallowed and Wait() would block forever.
            while (true) {
                const uint32_t seen = signal_.Snapshot();
                if (!running_.load(std::memory_order_acquire)) break;
                if (!process_pending()) signal_.Wait(seen);
            }
            worker_id_.store(std::thread::id{}, std::memory_order_release);
        });
    } catch (...) {
        running_.store(false, std::memory_order_release);  // the thread never existed
        throw;
    }
}

void WorkerThread::Stop() {
    if (OnWorkerThread()) {
        throw UsageError("Stop() from the component's own worker thread");
    }
    running_.store(false, std::memory_order_release);
    signal_.Notify();
    if (thread_.joinable()) thread_.join();
}

} // namespace voyahslam::events
