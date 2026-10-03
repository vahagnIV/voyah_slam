#include <voyahviz/map_viewer.h>

#include <stdexcept>
#include <voyahslam/events/listener.h>
#include <voyahslam/features/instantiation.h>

namespace voyahviz {

template <typename TFeatureExtractor>
MapViewer<TFeatureExtractor>::MapViewer(double fps) {
    if (!(fps > 0.0)) {
        throw std::invalid_argument("voyahviz::MapViewer: fps must be positive");
    }
    period_ = std::chrono::duration_cast<std::chrono::steady_clock::duration>(
        std::chrono::duration<double>(1.0 / fps));
}

template <typename TFeatureExtractor>
MapViewer<TFeatureExtractor>::~MapViewer() {
    Stop();
}

template <typename TFeatureExtractor>
void MapViewer<TFeatureExtractor>::Start() {
    if (running_.load()) return;
    if (thread_.joinable()) thread_.join();
    running_.store(true);
    thread_ = std::thread(&MapViewer::Run, this);
}

template <typename TFeatureExtractor>
void MapViewer<TFeatureExtractor>::Stop() {
    running_.store(false);
    if (thread_.joinable()) thread_.join();
    // Events still queued hold a reference; acknowledge them so their pool
    // slots return.
    DrainInbox();
}

template <typename TFeatureExtractor>
void MapViewer<TFeatureExtractor>::Run() {
    using TClock = std::chrono::steady_clock;
    TClock::time_point next_tick = TClock::now();
    while (running_.load()) {
        DrainInbox();
        Render();
        next_tick += period_;
        TClock::time_point now = TClock::now();
        if (next_tick < now) {
            next_tick = now + period_;  // late: no catch-up burst
        }
        std::this_thread::sleep_until(next_tick);
    }
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

template <typename TFeatureExtractor>
void MapViewer<TFeatureExtractor>::Render() {
    // Pangolin rendering of scene_ goes here.
}

INSTANTIATE_CLASS(MapViewer)

} // namespace voyahviz
