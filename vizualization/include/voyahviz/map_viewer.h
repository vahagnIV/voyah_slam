#ifndef VOYAHVIZ_MAP_VIEWER_H
#define VOYAHVIZ_MAP_VIEWER_H

#include <atomic>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <thread>
#include <voyahslam/events/async_observer.h>
#include <voyahslam/events/wake_signal.h>
#include <voyahslam/logging/logger.h>
#include <voyahviz/scene_state.h>

namespace voyahviz {

template <typename TFeatureExtractor>
class PangolinRenderer;  // private (src/pangolin_renderer.h)

struct MapViewerOptions {
    double fps = 30.0;          // must be > 0 (std::invalid_argument otherwise)
    bool show_window = false;   // false = headless: drain + scene model only
    int width = 1280;
    int height = 800;
    double camera_size = 0.08;  // frustum depth in map units (median depth = 1 at init)
};

// Consumes Tracker and Mapper events and renders the 3D map from them.
// Everything it knows comes from events through one inbox, so it can later run
// on another machine behind a serializing observer.
//
// The viewer loop runs either on the viewer's own thread (Start) or on the
// caller's thread (RunOnCurrentThread, for platforms whose GUI must live on the
// main thread). Only the thread running the loop touches the scene model and
// the GL context; the window is created at loop entry and destroyed at loop
// exit (or when the user closes it), on that thread. Closing the window does
// not stop the loop: it keeps draining headless until RequestStop()/Stop(), so a
// closed window does not let the inbox grow.
//
// Threading contract: Start() and Stop() must be called from one controlling
// thread (externally ordered; they are not safe to race with each other). In
// RunOnCurrentThread() mode other threads end the run with RequestStop() only;
// Stop() and the destructor belong on the loop's thread, after
// RunOnCurrentThread() has returned.
//
// Usage: connect producers with AddObserver(viewer.inbox()) before they start and
// keep the returned Subscription (e.g. in a vector declared before the Slam);
// stop the producers before the viewer so nothing arrives after the final drain.
template <typename TFeatureExtractor>
class MapViewer {
public:
    // Throws std::invalid_argument if options.fps <= 0. With show_window it may
    // set PANGOLIN_WINDOW_URI (setenv), so construct it before other threads start.
    explicit MapViewer(MapViewerOptions options = {});
    // Calls Stop(). Must not run while RunOnCurrentThread() is still active.
    ~MapViewer();
    MapViewer(const MapViewer&) = delete;
    MapViewer& operator=(const MapViewer&) = delete;

    // Runs the loop on the viewer's own thread; no-op if that thread is already
    // running. While RunOnCurrentThread() is active it is a no-op that logs a
    // warning. Restart after Stop() is allowed.
    void Start();
    // Requests the stop. In Start() mode it then joins the viewer thread and
    // drains (applies and acknowledges) whatever is still queued. While
    // RunOnCurrentThread() is active it only requests the stop:
    // RunOnCurrentThread() does its own final drain on its thread before
    // returning. Idempotent; safe without Start(). Do not call it from another
    // thread before RunOnCurrentThread() is entered: with no loop running it
    // consumes the request, and the later run would block (use RequestStop()).
    void Stop();
    // Runs the loop on the caller's thread; blocks until RequestStop() or
    // Stop(), then drains what is still queued and returns. Closing the window
    // does not return (see window_closed()). Throws std::logic_error if Start()
    // or another RunOnCurrentThread() is active.
    void RunOnCurrentThread();
    // Ends either loop at its next iteration; does not join. Thread-safe. A
    // request made before the loop is entered is kept: that run drains once and
    // returns (so a producer finishing first can't leave RunOnCurrentThread()
    // blocked). Stop() and the end of a run consume it. This is the only way for
    // another thread to end a RunOnCurrentThread() run.
    void RequestStop();

    // True once the window was closed by the user or could not be created.
    // Thread-safe.
    [[nodiscard]] bool window_closed() const { return window_closed_.load(); }
    // Blocks until the window is closed or the loop exits. Returns immediately
    // when headless, when the window is already closed (or failed to open), or
    // when no loop is running. Supports one waiter at a time.
    void WaitForWindowClose();

    [[nodiscard]] voyahslam::events::AsyncObserver& inbox() { return inbox_; }
    // Safe to read from any thread.
    [[nodiscard]] size_t events_processed() const { return events_processed_.load(); }

private:
    using Logger = voyahslam::Logger;  // for CLASS_LOGGER

    // state_ packs the run mode and the stop request into one atomic, so a
    // stop request can never be lost between a mode change and loop entry.
    static constexpr uint32_t kIdle = 0;
    static constexpr uint32_t kOwnThread = 1;      // Start()
    static constexpr uint32_t kCurrentThread = 2;  // RunOnCurrentThread()
    static constexpr uint32_t kModeMask = 3;
    static constexpr uint32_t kStopRequested = 4;

    // Moves state_ from idle to `mode`, keeping a stop request made while idle
    // (that run then only does its final drain); returns the mode found instead
    // if not idle (kIdle on success). Every exit path resets state_ to exactly
    // kIdle, so a request never reaches a later run.
    uint32_t TryEnterMode(uint32_t mode);
    // The loop shared by both modes: Step() paced to fps until stopped, then a
    // final drain. Runs on the loop thread.
    void Loop();
    void Step();
    void DrainInbox();
    // Destroys the renderer, marks the window closed and wakes the waiter.
    void CloseWindow();

    MapViewerOptions options_;
    voyahslam::events::AsyncObserver inbox_;
    SceneState<TFeatureExtractor> scene_;  // loop thread only (or after join)
    std::unique_ptr<PangolinRenderer<TFeatureExtractor>> renderer_;  // loop thread only
    std::chrono::steady_clock::duration period_;
    std::atomic<uint32_t> state_{kIdle};
    std::atomic<bool> loop_running_{false};
    std::atomic<bool> window_closed_{false};
    std::atomic<size_t> events_processed_{0};
    voyahslam::events::WakeSignal window_signal_;  // window close / loop exit
    std::thread thread_;
    CLASS_LOGGER(MapViewer);
};

} // namespace voyahviz

#endif // VOYAHVIZ_MAP_VIEWER_H
