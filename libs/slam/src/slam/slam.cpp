#include <cstdint>
#include <cstdio>
#include <exception>
#include <voyahslam/errors.h>
#include <voyahslam/features/instantiation.h>
#include <voyahslam/logging/logger.h>
#include <voyahslam/slam/slam.h>

namespace voyahslam::slam {

namespace {

// Lost maps are kept only to be merged, and merging needs the LoopCloser: with
// none (loop closing off, or no vocabulary) they are discarded as before the
// Atlas.
mapping::MapperOptions MapperOptionsFor(mapping::MapperOptions options, bool has_loop_closer) {
    if (!has_loop_closer) options.atlas.min_frames_to_keep = SIZE_MAX;
    return options;
}

}  // namespace

template <typename TFeatureExtractor>
Slam<TFeatureExtractor>::Slam(mapping::MapperOptions mapper_options,
                              tracking::TrackerOptions tracker_options,
                              std::shared_ptr<const typename TMapper::TVocabulary> vocabulary,
                              loop_closing::LoopCloserOptions loop_closer_options,
                              global_ba::GlobalBaOptions global_ba_options)
    : tracker_(tracker_options, vocabulary),
      mapper_(MapperOptionsFor(mapper_options, vocabulary && loop_closer_options.enabled), vocabulary),
      loop_closer_(loop_closer_options, vocabulary) {
    subscriptions_.push_back(tracker_.AddObserver(mapper_.inbox()));
    subscriptions_.push_back(mapper_.AddObserver(tracker_.inbox()));
    subscriptions_.push_back(mapper_.AddObserver(loop_closer_.inbox()));
    subscriptions_.push_back(loop_closer_.AddObserver(mapper_.inbox()));
    // Without a vocabulary the LoopCloser never detects a loop, so a GlobalBA
    // would never run: none is created.
    if (vocabulary && loop_closer_options.enabled && global_ba_options.enabled) {
        global_ba_.emplace(global_ba_options);
        subscriptions_.push_back(mapper_.AddObserver(global_ba_->inbox()));
        subscriptions_.push_back(global_ba_->AddObserver(mapper_.inbox()));
    }
}

template <typename TFeatureExtractor>
bool Slam<TFeatureExtractor>::OnComponentThread() const {
    return tracker_.OnWorkerThread() || mapper_.OnWorkerThread() || loop_closer_.OnWorkerThread() ||
           (global_ba_ && global_ba_->OnWorkerThread());
}

template <typename TFeatureExtractor>
Slam<TFeatureExtractor>::~Slam() {
    // Destroying the Slam inside one of its own threads cannot work (that thread
    // would join itself and keep running inside a destroyed object) and a
    // destructor must not throw: log and terminate (DESIGN section 7).
    if (OnComponentThread()) {
        constexpr const char* kMessage = "Slam destroyed on one of its component threads";
        std::fprintf(stderr, "%s\n", kMessage);
        Logger::CreateLogger("Slam")->critical(kMessage);
        std::terminate();
    }
    Stop();
}

template <typename TFeatureExtractor>
void Slam<TFeatureExtractor>::Start() {
    events::RunState expected = events::RunState::Created;
    if (!state_.compare_exchange_strong(expected, events::RunState::Starting, std::memory_order_acq_rel,
                                        std::memory_order_acquire)) {
        throw UsageError(expected == events::RunState::Stopped
                             ? "Start() after Stop(): create a new component"
                             : "Start() called twice");
    }
    try {
        mapper_.Start();
        if (global_ba_) global_ba_->Start();
        loop_closer_.Start();
        tracker_.Start();
    } catch (...) {
        // Stops what was started and makes the rest terminal (Created -> Stopped).
        tracker_.Stop();
        loop_closer_.Stop();
        if (global_ba_) global_ba_->Stop();
        mapper_.Stop();
        state_.store(events::RunState::Stopped, std::memory_order_release);
        state_.notify_all();
        throw;
    }
    state_.store(events::RunState::Running, std::memory_order_release);
    state_.notify_all();
}

template <typename TFeatureExtractor>
void Slam<TFeatureExtractor>::Stop() {
    // Before any state change or component Stop(): a refused call changes nothing.
    // Lock-free: each worker publishes its thread id in an atomic.
    if (OnComponentThread()) {
        throw UsageError("Slam::Stop() from a component's own thread");
    }
    while (true) {
        events::RunState state = state_.load(std::memory_order_acquire);
        switch (state) {
            case events::RunState::Stopped:
                return;
            case events::RunState::Starting:
            case events::RunState::Stopping:
                state_.wait(state, std::memory_order_acquire);
                break;
            case events::RunState::Created:
            case events::RunState::Running: {
                if (!state_.compare_exchange_strong(state, events::RunState::Stopping,
                                                    std::memory_order_acq_rel,
                                                    std::memory_order_acquire)) {
                    break;
                }
                // From Created every component moves to Stopped without a thread, so
                // none can start later.
                try {
                    tracker_.Stop();
                    loop_closer_.Stop();
                    if (global_ba_) global_ba_->Stop();
                    mapper_.Stop();
                } catch (...) {
                    // Not a UsageError (the caller's thread was checked above); for
                    // example a failed join. Do not leave other callers waiting on
                    // Stopping.
                    state_.store(state, std::memory_order_release);
                    state_.notify_all();
                    throw;
                }
                state_.store(events::RunState::Stopped, std::memory_order_release);
                state_.notify_all();
                return;
            }
        }
    }
}

INSTANTIATE_CLASS(Slam)

} // namespace voyahslam::slam
