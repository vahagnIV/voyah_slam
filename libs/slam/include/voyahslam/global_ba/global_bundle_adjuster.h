#ifndef VOYAHSLAM_GLOBAL_BA_GLOBAL_BUNDLE_ADJUSTER_H
#define VOYAHSLAM_GLOBAL_BA_GLOBAL_BUNDLE_ADJUSTER_H

#include <atomic>
#include <cstddef>
#include <cstdint>
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
// a snapshot of the replica at epoch N (RunGlobalBa's contract).
// Transactions arriving meanwhile are applied, and so released, at each poll
// of should_stop: the Mapper's transaction pool aborts when exhausted and a
// run on a large map lasts seconds, so none may be held for a run.
// Interruption: Stop(), or an arrived transaction that is a newer correction
// (a larger map epoch with a loop edge) or a new map (new_map or a newer map
// id). An interrupted run publishes nothing; a newer correction applied
// during the run starts a new run next, a new map drops it. A run interrupted
// by Stop() is not resumed: after a restart there is no global BA until the
// next loop correction.
// The result carries the run's map id and epoch N.
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
    // Interrupts a running BA (it publishes nothing, even if it had passed its
    // last poll, and is not resumed by a later Start()) and joins the worker.
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
    // Runs cut short by Stop() or an arrived newer correction or new map.
    [[nodiscard]] size_t num_runs_interrupted() const { return num_runs_interrupted_; }
    // Results dropped because max_events_in_flight GlobalBAFinished were in use.
    [[nodiscard]] size_t num_results_dropped() const { return num_results_dropped_; }
    // True from Stop() until the next Start(); any thread.
    [[nodiscard]] bool stop_requested() const { return stop_requested_.load(std::memory_order_acquire); }

    // Tests: `hook` is called on the GlobalBA thread before each run. Set it
    // while the worker is stopped.
    void set_run_hook(std::function<void()> hook) { run_hook_ = std::move(hook); }
    // Tests: `hook` is called on the GlobalBA thread at each poll of a run's
    // should_stop (before the first LM iteration and after each), before the
    // inbox is checked. Set it while the worker is stopped.
    void set_poll_hook(std::function<void()> hook) { poll_hook_ = std::move(hook); }
    // Tests: `hook` is called on the GlobalBA thread after a run's last poll,
    // just before its result is published. Set it while the worker is stopped.
    void set_publish_hook(std::function<void()> hook) { publish_hook_ = std::move(hook); }

private:
    void ApplyEvent(const events::Event& event);
    // Runs the global BA of the replica and publishes its result unless
    // interrupted.
    void Run();
    // Applies (and releases) every event in the inbox; true iff one
    // supersedes the run of (map_id, map_epoch).
    bool ApplyArrived(map::MapId map_id, uint32_t map_epoch);
    // Publishes the result of the run of (map_id, map_epoch).
    void Publish(map::MapId map_id, uint32_t map_epoch, GlobalBaSolution&& solution);

    const GlobalBaOptions options_;
    events::Listener inbox_listener_ = inbox_.GetListener();
    std::atomic<bool> stop_requested_{false};

    // GlobalBA-thread state.
    tracking::MapReplica<TFeatureExtractor> replica_;
    bool run_pending_ = false;
    size_t num_runs_started_ = 0;
    size_t num_runs_interrupted_ = 0;
    size_t num_results_dropped_ = 0;
    std::function<void()> run_hook_;
    std::function<void()> poll_hook_;
    std::function<void()> publish_hook_;

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
