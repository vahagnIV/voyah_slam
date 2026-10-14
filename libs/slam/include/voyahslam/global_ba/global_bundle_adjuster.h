#ifndef VOYAHSLAM_GLOBAL_BA_GLOBAL_BUNDLE_ADJUSTER_H
#define VOYAHSLAM_GLOBAL_BA_GLOBAL_BUNDLE_ADJUSTER_H

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <optional>
#include <utility>
#include <voyahslam/events/event_node.h>
#include <voyahslam/events/global_ba_finished.h>
#include <voyahslam/events/listener.h>
#include <voyahslam/events/worker_thread.h>
#include <voyahslam/global_ba/global_ba.h>
#include <voyahslam/logging/logger.h>
#include <voyahslam/map/map_id.h>
#include <voyahslam/tracking/atlas_replica.h>
#include <voyahslam/tracking/map_replica.h>

namespace voyahslam::global_ba {

// Test access to the GlobalBundleAdjuster's thread-affine state (defined by the
// tests).
struct GlobalBundleAdjusterTestAccess;

// Runs ORB-SLAM3's global BA after a loop correction, on its own replica of
// the map and its own thread, and publishes GlobalBAFinished for the Mapper to
// merge. Consumes the Mapper's MapTransactions; never modifies map objects.
//
// Trigger: an applied transaction of the active map that adds a loop edge
// (the correction at epoch N). After the inbox is drained, one run optimizes
// a snapshot of the replica at epoch N (RunGlobalBa's contract).
// Transactions arriving meanwhile are applied, and so released, at each poll
// of should_stop: a run on a large map lasts seconds, and holding the
// transactions that arrive meanwhile would hold their memory for as long.
// Interruption: Stop(), or an arrived transaction that is a newer correction
// of the run's map (a larger map epoch with a loop edge), recreates it
// (new_map), or leaves it not active (a new active map, or none after the
// Mapper drops a finally lost map). An interrupted run publishes nothing; a
// newer correction applied during the run starts a new run next, a change of
// the active map drops it. A run interrupted by Stop() is not resumed: after
// a restart there is no global BA until the next loop correction.
// The result carries the run's map id and epoch N.
// Maps: the replica is an AtlasReplica (the replica rule: atlas_replica.h),
// and the run is on the active map. A change of the active map, or a new map,
// drops a pending run.
//
// Thread safety. Every public accessor is either atomic (any thread:
// num_runs_started(), num_runs_interrupted(), stop_requested()) or a
// lifecycle/driver call (Start, Stop, ProcessPending, the test hook setters).
// The replica is private (tests read it through
// GlobalBundleAdjusterTestAccess): apps read the map through
// mapping::MapFollower.
template <typename TFeatureExtractor>
class GlobalBundleAdjuster : public events::EventNode {
public:
    explicit GlobalBundleAdjuster(GlobalBaOptions options = {}) : EventNode("GlobalBA"), options_(options) {}
    // Stops the worker before any member it uses is destroyed.
    ~GlobalBundleAdjuster();

    // Lifecycle (D-10..D-12): Created -> Running -> Stopped, Stopped is terminal.
    // Start() twice or after Stop() throws UsageError; Stop() before Start() moves
    // to Stopped; Stop() is idempotent and every concurrent call returns only after
    // the worker is joined. Both are safe from any thread, except that Stop() on
    // the component's own worker thread throws UsageError (it would join itself).
    void Start();
    // Interrupts a running BA (it publishes nothing, even if it had passed its
    // last poll, and is not resumed by a later Start()) and joins the worker.
    void Stop();

    // Drains the inbox, applying every transaction, then runs a pending
    // global BA. Returns true iff an event or a run was processed. Called by
    // the worker thread; public so tests can drive it synchronously (never
    // concurrently with a running worker).
    [[nodiscard]] bool ProcessPending();

    // Runs started; any thread.
    [[nodiscard]] size_t num_runs_started() const { return num_runs_started_.load(std::memory_order_acquire); }
    // Runs cut short by Stop(), an arrived newer correction or a change of
    // the active map; any thread.
    [[nodiscard]] size_t num_runs_interrupted() const {
        return num_runs_interrupted_.load(std::memory_order_acquire);
    }
    // True once Stop() was called (Stopped is terminal); any thread.
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
    friend struct GlobalBundleAdjusterTestAccess;

    // GlobalBA-thread state; tests that drive ProcessPending() synchronously
    // read it through GlobalBundleAdjusterTestAccess. Apps read the map
    // through mapping::MapFollower.
    [[nodiscard]] const tracking::AtlasReplica<TFeatureExtractor>& atlas() const { return atlas_; }
    // The active map; empty if there is none.
    [[nodiscard]] const tracking::MapReplica<TFeatureExtractor>& map_replica() const {
        return atlas_.active_or_empty();
    }

    // Applies a transaction to the Atlas; true iff it supersedes the run of
    // (map_id, map_epoch) (pass none for no run).
    bool ApplyEvent(const events::Event& event, std::optional<map::MapId> map_id = std::nullopt,
                    uint32_t map_epoch = 0);
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
    tracking::AtlasReplica<TFeatureExtractor> atlas_;
    bool run_pending_ = false;
    std::atomic<size_t> num_runs_started_{0};
    std::atomic<size_t> num_runs_interrupted_{0};
    std::function<void()> run_hook_;
    std::function<void()> poll_hook_;
    std::function<void()> publish_hook_;

    CLASS_LOGGER(GlobalBundleAdjuster);

    // Last member: constructed after, and stopped before, everything it uses.
    events::WorkerThread worker_{wake_signal()};
};

} // namespace voyahslam::global_ba

#endif // VOYAHSLAM_GLOBAL_BA_GLOBAL_BUNDLE_ADJUSTER_H
