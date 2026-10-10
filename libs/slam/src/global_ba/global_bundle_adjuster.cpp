#include <chrono>
#include <functional>
#include <optional>
#include <utility>
#include <voyahslam/events/map_transaction.h>
#include <voyahslam/features/instantiation.h>
#include <voyahslam/global_ba/global_bundle_adjuster.h>

namespace voyahslam::global_ba {
namespace {

// True iff `event` supersedes the run of (map_id, map_epoch): a transaction
// of a non-zero, not older map that starts a new map (new_map or a newer map
// id) or is a newer correction (a larger epoch with a loop edge).
template <typename TFeatureExtractor>
bool Supersedes(const events::Event& event, map::MapId map_id, uint32_t map_epoch) {
    if (event.map_id.value == 0 || event.type != events::EventType::MapTransaction || event.map_id < map_id) {
        return false;
    }
    const auto& transaction = static_cast<const events::MapTransaction<TFeatureExtractor>&>(event);
    return transaction.new_map || event.map_id > map_id ||
           (transaction.map_epoch > map_epoch && !transaction.added_loop_edges.empty());
}

} // namespace

template <typename TFeatureExtractor>
GlobalBundleAdjuster<TFeatureExtractor>::~GlobalBundleAdjuster() {
    worker_.Stop();
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
    // Every event is applied on receipt and so released at once, here and at
    // each poll of a run: GlobalBA never holds the Mapper's pooled
    // transactions.
    bool processed = false;
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
bool GlobalBundleAdjuster<TFeatureExtractor>::ApplyArrived(map::MapId map_id, uint32_t map_epoch) {
    // Each event is judged against the run, then applied and released: the
    // run optimizes its own snapshot (RunGlobalBa's contract), so the
    // replica may move on under it.
    bool superseded = false;
    while (const events::Event* event = inbox_listener_.GetNextEvent()) {
        superseded = Supersedes<TFeatureExtractor>(*event, map_id, map_epoch) || superseded;
        ApplyEvent(*event);
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
        if (interrupted) {
            return true;
        }
        if (poll_hook_) {
            poll_hook_();
        }
        interrupted = stop_requested() || ApplyArrived(map_id, map_epoch);
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
    Publish(map_id, map_epoch, std::move(*solution));
}

template <typename TFeatureExtractor>
void GlobalBundleAdjuster<TFeatureExtractor>::Publish(map::MapId map_id, uint32_t map_epoch,
                                                      GlobalBaSolution&& solution) {
    auto* event = result_pool_.TryCreate();
    if (event == nullptr) {
        ++num_results_dropped_;
        log_->warn("Global BA result of map {} at epoch {} dropped: {} GlobalBAFinished events in flight",
                   map_id.value, map_epoch, options_.max_events_in_flight);
        return;
    }
    // The run's: the replica may have moved on since.
    event->map_id = map_id;
    event->map_epoch = map_epoch;
    event->frame_poses = std::move(solution.frame_poses);
    event->point_positions = std::move(solution.point_positions);
    // Urgent: the Mapper's running local BA stops for the merge.
    event->priority = true;
    observable_.Notify(event);
}

INSTANTIATE_CLASS(GlobalBundleAdjuster)

} // namespace voyahslam::global_ba
