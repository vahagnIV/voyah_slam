#include <voyahviz/map_viewer.h>

#include <exception>
#include <stdexcept>
#include <voyahslam/events/listener.h>
#include <voyahslam/features/instantiation.h>
#include "pangolin_renderer.h"

namespace voyahviz {

template <typename TFeatureExtractor>
MapViewer<TFeatureExtractor>::MapViewer(MapViewerOptions options) : options_(options) {
    if (!(options_.fps > 0.0)) {
        throw std::invalid_argument("voyahviz::MapViewer: fps must be positive");
    }
    period_ = std::chrono::duration_cast<std::chrono::steady_clock::duration>(
        std::chrono::duration<double>(1.0 / options_.fps));
}

template <typename TFeatureExtractor>
MapViewer<TFeatureExtractor>::~MapViewer() {
    Stop();
}

template <typename TFeatureExtractor>
uint32_t MapViewer<TFeatureExtractor>::TryEnterMode(uint32_t mode) {
    uint32_t state = state_.load();
    do {
        if ((state & kModeMask) != kIdle) return state & kModeMask;
    } while (!state_.compare_exchange_weak(state, mode));
    return kIdle;
}

template <typename TFeatureExtractor>
void MapViewer<TFeatureExtractor>::Start() {
    uint32_t found = TryEnterMode(kOwnThread);
    if (found == kCurrentThread) {
        log_->warn("Start() ignored: RunOnCurrentThread() is active");
        return;
    }
    if (found != kIdle) return;  // already running
    window_closed_.store(false);
    loop_running_.store(true);
    thread_ = std::thread(&MapViewer::Loop, this);
}

template <typename TFeatureExtractor>
void MapViewer<TFeatureExtractor>::Stop() {
    uint32_t state = state_.load();
    while (true) {
        uint32_t mode = state & kModeMask;
        if (mode == kCurrentThread) {
            // Only request: RunOnCurrentThread() drains on its own thread.
            if (state_.compare_exchange_weak(state, state | kStopRequested)) return;
        } else if (mode == kOwnThread) {
            if (state_.compare_exchange_weak(state, state | kStopRequested)) break;
        } else {
            // Idle: claim the viewer for the drain so no loop can start meanwhile.
            if (state_.compare_exchange_weak(state, kOwnThread | kStopRequested)) break;
        }
    }
    if (thread_.joinable()) thread_.join();
    // Events still queued hold a reference; acknowledge them so their pool
    // slots return.
    DrainInbox();
    state_.store(kIdle);
}

template <typename TFeatureExtractor>
void MapViewer<TFeatureExtractor>::RunOnCurrentThread() {
    uint32_t found = TryEnterMode(kCurrentThread);
    if (found == kOwnThread) {
        throw std::logic_error("voyahviz::MapViewer: RunOnCurrentThread() while Start() is active");
    }
    if (found != kIdle) {
        throw std::logic_error("voyahviz::MapViewer: RunOnCurrentThread() is already active");
    }
    window_closed_.store(false);
    loop_running_.store(true);
    try {
        Loop();
    } catch (...) {
        // Leave the viewer restartable and never strand a WaitForWindowClose().
        renderer_.reset();
        loop_running_.store(false);
        window_signal_.Notify();
        state_.store(kIdle);
        throw;
    }
    state_.store(kIdle);
}

template <typename TFeatureExtractor>
void MapViewer<TFeatureExtractor>::RequestStop() {
    state_.fetch_or(kStopRequested);
}

template <typename TFeatureExtractor>
void MapViewer<TFeatureExtractor>::WaitForWindowClose() {
    if (!options_.show_window) return;
    while (true) {
        // Snapshot before checking, so a notify in between is not lost.
        uint32_t seen = window_signal_.Snapshot();
        if (window_closed_.load() || !loop_running_.load()) return;
        window_signal_.Wait(seen);
    }
}

template <typename TFeatureExtractor>
void MapViewer<TFeatureExtractor>::Loop() {
    if (options_.show_window) {
        try {
            renderer_ = std::make_unique<PangolinRenderer<TFeatureExtractor>>(options_);
        } catch (const std::exception& e) {
            log_->error("Cannot open the map window, running headless: {}", e.what());
            window_closed_.store(true);
            window_signal_.Notify();
        }
    }
    using TClock = std::chrono::steady_clock;
    TClock::time_point next_tick = TClock::now();
    while (!(state_.load() & kStopRequested)) {
        Step();
        next_tick += period_;
        TClock::time_point now = TClock::now();
        if (next_tick < now) {
            next_tick = now + period_;  // late: no catch-up burst
        }
        std::this_thread::sleep_until(next_tick);
    }
    renderer_.reset();
    DrainInbox();
    loop_running_.store(false);
    window_signal_.Notify();
}

template <typename TFeatureExtractor>
void MapViewer<TFeatureExtractor>::Step() {
    DrainInbox();
    if (renderer_ && !renderer_->Draw(scene_, events_processed_.load())) {
        CloseWindow();
    }
}

template <typename TFeatureExtractor>
void MapViewer<TFeatureExtractor>::CloseWindow() {
    renderer_.reset();
    window_closed_.store(true);
    window_signal_.Notify();
}

template <typename TFeatureExtractor>
void MapViewer<TFeatureExtractor>::DrainInbox() {
    // The Listener acknowledges each event when the next one is requested, and
    // the last one when it goes out of scope.
    voyahslam::events::Listener listener = inbox_.GetListener();
    while (voyahslam::events::Event* event = listener.GetNextEvent()) {
        scene_.Apply(*event);
        events_processed_.fetch_add(1);
    }
}

INSTANTIATE_CLASS(MapViewer)

} // namespace voyahviz
