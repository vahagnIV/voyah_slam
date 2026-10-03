#ifndef VOYAHVIZ_MAP_VIEWER_H
#define VOYAHVIZ_MAP_VIEWER_H

#include <atomic>
#include <chrono>
#include <cstddef>
#include <thread>
#include <voyahslam/events/async_observer.h>
#include <voyahviz/scene_state.h>

namespace voyahviz {

// Consumes Tracker and Mapper events on its own thread and (later) renders the
// 3D map from them. Everything it knows comes from events through one inbox,
// so it can later run on another machine behind a serializing observer.
//
// Usage: connect producers with AddObserver(viewer.inbox()) before they start;
// stop the producers before the viewer so nothing arrives after Stop()'s final
// drain.
template <typename TFeatureExtractor>
class MapViewer {
public:
    // Throws std::invalid_argument if fps <= 0.
    explicit MapViewer(double fps = 30.0);
    ~MapViewer();
    MapViewer(const MapViewer&) = delete;
    MapViewer& operator=(const MapViewer&) = delete;

    // Starts the viewer thread; no-op if already running. Restart after Stop()
    // is allowed.
    void Start();
    // Joins the viewer thread, then drains (applies and acknowledges) whatever is
    // still queued. Idempotent; safe without Start().
    void Stop();

    [[nodiscard]] voyahslam::events::AsyncObserver& inbox() { return inbox_; }
    // Safe to read from any thread.
    [[nodiscard]] size_t events_processed() const { return events_processed_.load(); }

private:
    void Run();
    void DrainInbox();
    void Render();

    voyahslam::events::AsyncObserver inbox_;
    SceneState<TFeatureExtractor> scene_;  // viewer thread only (or after join)
    std::chrono::steady_clock::duration period_;
    std::atomic<bool> running_{false};
    std::atomic<size_t> events_processed_{0};
    std::thread thread_;
};

} // namespace voyahviz

#endif // VOYAHVIZ_MAP_VIEWER_H
