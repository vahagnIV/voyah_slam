#ifndef VOYAHSLAM_LOOP_CLOSING_LOOP_CLOSER_H
#define VOYAHSLAM_LOOP_CLOSING_LOOP_CLOSER_H

#include <voyahslam/events/event_node.h>
#include <voyahslam/events/listener.h>
#include <voyahslam/events/worker_thread.h>

namespace voyahslam::loop_closing {

// Detects loops on its own replica of the map and publishes LoopDetected.
// Detection only — the Mapper closes the loop. Consumes the Mapper's map diffs.
// Runs on its own thread; for now it only acknowledges what it receives.
template <typename TFeatureExtractor>
class LoopCloser : public events::EventNode {
public:
    LoopCloser() = default;
    // Stops the worker before any member it uses is destroyed.
    ~LoopCloser();

    // Start() and Stop() must be called from one controlling thread.
    void Start();
    void Stop();

    // Drains and acknowledges the inbox. Returns true iff an event was
    // processed. Called by the worker thread; public so tests can drive it
    // synchronously (never concurrently with a running worker).
    [[nodiscard]] bool ProcessPending();

private:
    events::Listener inbox_listener_ = inbox_.GetListener();

    // Last member: constructed after, and stopped before, everything it uses.
    events::WorkerThread worker_{wake_signal()};
};

} // namespace voyahslam::loop_closing

#endif // VOYAHSLAM_LOOP_CLOSING_LOOP_CLOSER_H
