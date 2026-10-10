#ifndef VOYAHSLAM_GLOBAL_BA_GLOBAL_BUNDLE_ADJUSTER_H
#define VOYAHSLAM_GLOBAL_BA_GLOBAL_BUNDLE_ADJUSTER_H

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <functional>
#include <utility>
#include <voyahslam/events/event_node.h>
#include <voyahslam/events/global_ba_finished.h>
#include <voyahslam/events/listener.h>
#include <voyahslam/events/worker_thread.h>
#include <voyahslam/global_ba/global_ba.h>
#include <voyahslam/logging/logger.h>
#include <voyahslam/map/map_id.h>
#include <voyahslam/memory/object_pool.h>
#include <voyahslam/tracking/map_replica.h>

namespace voyahslam::global_ba {

// Runs ORB-SLAM3's global BA after a loop correction, on its own replica of
// the map and its own thread, and publishes GlobalBAFinished for the Mapper to
// merge. Consumes the Mapper's MapTransactions; never modifies map objects.
//
// Trigger: an applied transaction of the current map that adds a loop edge
// (the correction at epoch N). After the inbox is drained, one run optimizes
// the replica as it stands; transactions arriving meanwhile stay queued and
// are applied after it.
// Interruption: Stop(), or a waiting transaction that is a newer correction
// (a larger map epoch with a loop edge) or a new map (new_map or a newer map
// id). An interrupted run publishes nothing; a newer correction starts a new
// run once applied. The inbox cannot be peeked, so the stop check moves the
// waiting events, unapplied and in order, to a backlog that the next drain
// applies first (they were held in the inbox anyway).
// Map ids: events of map 0 and of older maps are ignored; a new map clears the
// replica (it clears itself) and drops a pending run.
template <typename TFeatureExtractor>
class GlobalBundleAdjuster : public events::EventNode {
public:
    explicit GlobalBundleAdjuster(GlobalBaOptions options = {}) : options_(options) {}
    // Stops the worker before any member it uses is destroyed.
    ~GlobalBundleAdjuster();

    // Start() and Stop() must be called from one controlling thread.
    void Start();
    // Interrupts a running BA (it publishes nothing) and joins the worker.
    void Stop();

    // Drains the inbox, applying every transaction, then runs a pending
    // global BA. Returns true iff an event or a run was processed. Called by
    // the worker thread; public so tests can drive it synchronously (never
    // concurrently with a running worker).
    [[nodiscard]] bool ProcessPending();

    // GlobalBA-thread state, for tests that drive ProcessPending()
    // synchronously: never read while the worker runs.
    [[nodiscard]] const tracking::MapReplica<TFeatureExtractor>& replica() const { return replica_; }
    [[nodiscard]] size_t num_runs_started() const { return num_runs_started_; }
    // Runs cut short by Stop() or a waiting newer correction or new map.
    [[nodiscard]] size_t num_runs_interrupted() const { return num_runs_interrupted_; }
    // Results dropped because max_events_in_flight GlobalBAFinished were in use.
    [[nodiscard]] size_t num_results_dropped() const { return num_results_dropped_; }
    // True from Stop() until the next Start(); any thread.
    [[nodiscard]] bool stop_requested() const { return stop_requested_.load(std::memory_order_acquire); }

    // Tests: `hook` is called on the GlobalBA thread before each run. Set it
    // while the worker is stopped.
    void set_run_hook(std::function<void()> hook) { run_hook_ = std::move(hook); }

private:
    void ApplyEvent(const events::Event& event);
    // Runs the global BA of the replica and publishes its result unless
    // interrupted.
    void Run();
    // Moves every waiting event to waiting_; true iff one supersedes the run
    // of (map_id, map_epoch).
    bool CheckWaiting(map::MapId map_id, uint32_t map_epoch);
    void Publish(GlobalBaSolution&& solution);

    const GlobalBaOptions options_;
    events::Listener inbox_listener_ = inbox_.GetListener();
    std::atomic<bool> stop_requested_{false};

    // GlobalBA-thread state.
    tracking::MapReplica<TFeatureExtractor> replica_;
    // Popped from the inbox during a run, not yet applied (oldest first);
    // each is released once applied.
    std::deque<events::Event*> waiting_;
    bool run_pending_ = false;
    size_t num_runs_started_ = 0;
    size_t num_runs_interrupted_ = 0;
    size_t num_results_dropped_ = 0;
    std::function<void()> run_hook_;

    // Published with TryCreate: a result finished while every event is in
    // flight (a Mapper backlog) is dropped and counted; the map keeps its
    // essential-graph correction until the next one.
    memory::ObjectPool<events::GlobalBAFinished, memory::ThreadSafePolicy> result_pool_{
        options_.max_events_in_flight};

    CLASS_LOGGER(GlobalBundleAdjuster);

    // Last member: constructed after, and stopped before, everything it uses.
    events::WorkerThread worker_{wake_signal()};
};

} // namespace voyahslam::global_ba

#endif // VOYAHSLAM_GLOBAL_BA_GLOBAL_BUNDLE_ADJUSTER_H
