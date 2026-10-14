#include <chrono>
#include <cstddef>
#include <functional>
#include <optional>
#include <voyahslam/tracking/map_replica.h>
#include <utility>
#include <voyahslam/events/map_transaction.h>
#include <voyahslam/features/instantiation.h>
#include <voyahslam/global_ba/global_bundle_adjuster.h>

namespace voyahslam::global_ba {
template <typename TFeatureExtractor>
GlobalBundleAdjuster<TFeatureExtractor>::~GlobalBundleAdjuster() {
    // Interrupts a running BA instead of waiting it out (it publishes nothing).
    stop_requested_.store(true, std::memory_order_release);
    worker_.Stop();
    EndRun();
}

template <typename TFeatureExtractor>
void GlobalBundleAdjuster<TFeatureExtractor>::Start() {
    stop_requested_.store(false, std::memory_order_release);
    BeginRun();
    worker_.Start([this] { return ProcessPending(); });
}

template <typename TFeatureExtractor>
void GlobalBundleAdjuster<TFeatureExtractor>::Stop() {
    // Before joining: a running BA polls it and stops.
    stop_requested_.store(true, std::memory_order_release);
    const bool was_running = worker_.running();
    worker_.Stop();
    EndRun();
    if (was_running && num_runs_started() > 0) {  // once per run
        log_->info("Global BA: {} runs, {} interrupted", num_runs_started(), num_runs_interrupted());
    }
}

template <typename TFeatureExtractor>
bool GlobalBundleAdjuster<TFeatureExtractor>::ProcessPending() {
    // Every event is applied on receipt and so released at once, here and at
    // each poll of a run: GlobalBA never holds the Mapper's
    // transactions.
    bool processed = false;
    while (events::EventPtr event = inbox_listener_.GetNextEvent()) {
        processed = true;  // released at the end of this iteration
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
bool GlobalBundleAdjuster<TFeatureExtractor>::ApplyEvent(const events::Event& event,
                                                         std::optional<map::MapId> map_id,
                                                         uint32_t map_epoch) {
    // Only the Mapper's transactions build the Atlas.
    if (event.type != events::EventType::MapTransaction) {
        return false;
    }
    const auto& transaction = static_cast<const events::MapTransaction<TFeatureExtractor>&>(event);
    const std::optional<map::MapId> previous = atlas_.active_map();
    if (!atlas_.Apply(transaction)) {
        return false;  // an unknown or removed map's
    }
    const std::optional<map::MapId> active = atlas_.active_map();
    if (transaction.new_map || active != previous) {
        run_pending_ = false;  // a new active map, or none
    }
    const bool loop_edge = transaction.map_id == active && !transaction.added_loop_edges.empty();
    if (loop_edge) {
        run_pending_ = true;  // a loop correction (ORB-SLAM3 CorrectLoop's end)
    }
    // The run's map recreated or no longer active, or a newer correction of it.
    return map_id && (active != map_id || (transaction.map_id == *map_id && transaction.new_map) ||
                      (loop_edge && transaction.map_epoch > map_epoch));
}

template <typename TFeatureExtractor>
bool GlobalBundleAdjuster<TFeatureExtractor>::ApplyArrived(map::MapId map_id, uint32_t map_epoch) {
    // Each event is applied, and so released, then judged against the run:
    // the run optimizes its own snapshot (RunGlobalBa's contract), so the
    // replica may move on under it, the run's map even dropped.
    bool superseded = false;
    while (events::EventPtr event = inbox_listener_.GetNextEvent()) {
        superseded = ApplyEvent(*event, map_id, map_epoch) || superseded;
    }
    return superseded;
}

template <typename TFeatureExtractor>
void GlobalBundleAdjuster<TFeatureExtractor>::Run() {
    if (run_hook_) {
        run_hook_();
    }
    num_runs_started_.fetch_add(1, std::memory_order_acq_rel);
    // Pending only while the active map is the corrected one.
    const tracking::MapReplica<TFeatureExtractor>& run_map = map_replica();
    const map::MapId map_id = run_map.map_id();
    const uint32_t map_epoch = run_map.map_epoch();
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
    std::optional<GlobalBaSolution> solution = RunGlobalBa(run_map, options_, should_stop);
    const double ms = 1e3 * std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
    if (interrupted) {
        num_runs_interrupted_.fetch_add(1, std::memory_order_acq_rel);
        log_->info("Global BA of map {} at epoch {} interrupted after {:.1f} ms", map_id.value, map_epoch, ms);
        return;
    }
    if (!solution) {
        log_->warn("Global BA of map {} at epoch {} failed after {:.1f} ms", map_id.value, map_epoch, ms);
        return;
    }
    // The payload of the result event (the vectors' elements).
    const size_t result_bytes =
        solution->frame_poses.size() * sizeof(typename decltype(solution->frame_poses)::value_type) +
        solution->point_positions.size() * sizeof(typename decltype(solution->point_positions)::value_type);
    log_->info("Global BA of map {} at epoch {}: {} Frames, {} points in {:.1f} ms, result {:.2f} MB", map_id.value,
               map_epoch, solution->frame_poses.size(), solution->point_positions.size(), ms,
               static_cast<double>(result_bytes) / (1024.0 * 1024.0));
    if (publish_hook_) {
        publish_hook_();
    }
    if (stop_requested()) {  // a Stop() after the last poll: publish nothing
        num_runs_interrupted_.fetch_add(1, std::memory_order_acq_rel);
        log_->info("Global BA of map {} at epoch {} discarded: stop requested", map_id.value, map_epoch);
        return;
    }
    Publish(map_id, map_epoch, std::move(*solution));
}

template <typename TFeatureExtractor>
void GlobalBundleAdjuster<TFeatureExtractor>::Publish(map::MapId map_id, uint32_t map_epoch,
                                                      GlobalBaSolution&& solution) {
    auto event = std::make_shared<events::GlobalBAFinished>();
    // The run's: the replica may have moved on since.
    event->map_id = map_id;
    event->map_epoch = map_epoch;
    event->frame_poses = std::move(solution.frame_poses);
    event->point_positions = std::move(solution.point_positions);
    // Urgent: the Mapper's running local BA stops for the merge.
    event->priority = true;
    observable_.Notify(std::move(event));
}

INSTANTIATE_CLASS(GlobalBundleAdjuster)

} // namespace voyahslam::global_ba
