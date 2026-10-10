#include <chrono>
#include <functional>
#include <optional>
#include <utility>
#include <voyahslam/events/map_transaction.h>
#include <voyahslam/features/instantiation.h>
#include <voyahslam/global_ba/global_bundle_adjuster.h>

namespace voyahslam::global_ba {

template <typename TFeatureExtractor>
GlobalBundleAdjuster<TFeatureExtractor>::~GlobalBundleAdjuster() {
    worker_.Stop();
    for (events::Event* event : waiting_) {
        event->recycle_(event);
    }
}

template <typename TFeatureExtractor>
void GlobalBundleAdjuster<TFeatureExtractor>::Start() {
    stop_requested_.store(false, std::memory_order_release);
    worker_.Start([this] { return ProcessPending(); });
}

template <typename TFeatureExtractor>
void GlobalBundleAdjuster<TFeatureExtractor>::Stop() {
    // Before joining: a running BA polls it and stops.
    stop_requested_.store(true, std::memory_order_release);
    const bool was_running = worker_.running();
    worker_.Stop();
    if (was_running && num_runs_started_ > 0) {  // once per run
        log_->info("Global BA: {} runs, {} interrupted, {} results dropped", num_runs_started_,
                   num_runs_interrupted_, num_results_dropped_);
    }
}

template <typename TFeatureExtractor>
bool GlobalBundleAdjuster<TFeatureExtractor>::ProcessPending() {
    // The events a run moved aside come first: they were popped before
    // anything still in the inbox. Each is released once applied, so no
    // transaction is held during a run beyond those that arrive meanwhile.
    bool processed = false;
    while (!waiting_.empty()) {
        events::Event* event = waiting_.front();
        waiting_.pop_front();
        processed = true;
        ApplyEvent(*event);
        event->recycle_(event);
    }
    while (const events::Event* event = inbox_listener_.GetNextEvent()) {
        processed = true;  // acknowledged by the next GetNextEvent()
        ApplyEvent(*event);
    }
    if (!run_pending_) {
        return processed;
    }
    // One run per call: the inbox is drained again before the next, and
    // Stop() interrupts this one.
    run_pending_ = false;
    Run();
    return true;
}

template <typename TFeatureExtractor>
void GlobalBundleAdjuster<TFeatureExtractor>::ApplyEvent(const events::Event& event) {
    // Map 0 is exempt; only the Mapper's transactions are applied.
    if (event.map_id.value == 0 || event.type != events::EventType::MapTransaction) {
        return;
    }
    const auto& transaction = static_cast<const events::MapTransaction<TFeatureExtractor>&>(event);
    if (event.map_id < replica_.map_id()) {
        return;  // an older map's
    }
    if (transaction.new_map || event.map_id > replica_.map_id()) {
        run_pending_ = false;  // the replica clears itself too
    }
    replica_.Apply(event);
    if (!transaction.added_loop_edges.empty()) {
        run_pending_ = true;  // a loop correction (ORB-SLAM3 CorrectLoop's end)
    }
}

template <typename TFeatureExtractor>
bool GlobalBundleAdjuster<TFeatureExtractor>::CheckWaiting(map::MapId map_id, uint32_t map_epoch) {
    // The inbox cannot be peeked: waiting events move, unapplied and in
    // order, to waiting_, which the next ProcessPending() applies first.
    bool superseded = false;
    events::Event* event = nullptr;
    while (inbox_.TryPop(event)) {
        waiting_.push_back(event);
        if (event->map_id.value == 0 || event->type != events::EventType::MapTransaction ||
            event->map_id < map_id) {
            continue;
        }
        const auto& transaction = static_cast<const events::MapTransaction<TFeatureExtractor>&>(*event);
        if (transaction.new_map || event->map_id > map_id ||
            (transaction.map_epoch > map_epoch && !transaction.added_loop_edges.empty())) {
            superseded = true;  // a new map, or a newer correction
        }
    }
    return superseded;
}

template <typename TFeatureExtractor>
void GlobalBundleAdjuster<TFeatureExtractor>::Run() {
    if (run_hook_) {
        run_hook_();
    }
    ++num_runs_started_;
    const map::MapId map_id = replica_.map_id();
    const uint32_t map_epoch = replica_.map_epoch();
    bool interrupted = false;
    // Sticky: once true, the BA stops at this poll.
    const std::function<bool()> should_stop = [&] {
        interrupted = interrupted || stop_requested() || CheckWaiting(map_id, map_epoch);
        return interrupted;
    };
    const auto start = std::chrono::steady_clock::now();
    std::optional<GlobalBaSolution> solution = RunGlobalBa(replica_, options_, should_stop);
    const double ms = 1e3 * std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
    if (interrupted) {
        ++num_runs_interrupted_;
        log_->info("Global BA of map {} at epoch {} interrupted after {:.1f} ms", map_id.value, map_epoch, ms);
        return;
    }
    if (!solution) {
        log_->warn("Global BA of map {} at epoch {} failed after {:.1f} ms", map_id.value, map_epoch, ms);
        return;
    }
    log_->info("Global BA of map {} at epoch {}: {} Frames, {} points in {:.1f} ms", map_id.value, map_epoch,
               solution->frame_poses.size(), solution->point_positions.size(), ms);
    Publish(std::move(*solution));
}

template <typename TFeatureExtractor>
void GlobalBundleAdjuster<TFeatureExtractor>::Publish(GlobalBaSolution&& solution) {
    auto* event = result_pool_.TryCreate();
    if (event == nullptr) {
        ++num_results_dropped_;
        log_->warn("Global BA result of map {} at epoch {} dropped: {} GlobalBAFinished events in flight",
                   replica_.map_id().value, replica_.map_epoch(), options_.max_events_in_flight);
        return;
    }
    event->map_id = replica_.map_id();
    event->map_epoch = replica_.map_epoch();
    event->frame_poses = std::move(solution.frame_poses);
    event->point_positions = std::move(solution.point_positions);
    // Urgent: the Mapper's running local BA stops for the merge.
    event->priority = true;
    observable_.Notify(event);
}

INSTANTIATE_CLASS(GlobalBundleAdjuster)

} // namespace voyahslam::global_ba
