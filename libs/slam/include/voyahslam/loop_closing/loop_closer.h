#ifndef VOYAHSLAM_LOOP_CLOSING_LOOP_CLOSER_H
#define VOYAHSLAM_LOOP_CLOSING_LOOP_CLOSER_H

#include <cstddef>
#include <cstdint>
#include <deque>
#include <functional>
#include <memory>
#include <optional>
#include <utility>
#include <vector>
#include <voyahslam/bow/vocabulary.h>
#include <voyahslam/events/event_node.h>
#include <voyahslam/events/listener.h>
#include <voyahslam/events/loop_detected.h>
#include <voyahslam/events/map_transaction.h>
#include <voyahslam/events/tracking_lost.h>
#include <voyahslam/events/worker_thread.h>
#include <voyahslam/geometry/sim3.h>
#include <voyahslam/logging/logger.h>
#include <voyahslam/loop_closing/common_region.h>
#include <voyahslam/map/frame_id.h>
#include <voyahslam/map/map_id.h>
#include <voyahslam/map/map_point_id.h>
#include <voyahslam/memory/object_pool.h>
#include <voyahslam/tracking/atlas_replica.h>
#include <voyahslam/tracking/map_replica.h>

namespace voyahslam::loop_closing {

// The LoopCloser's queries so far: detected, dropped from the backlog, and
// the detection time per detected query.
struct QueryStats {
    size_t detected = 0;
    size_t dropped = 0;
    double total_seconds = 0.0;
    double max_seconds = 0.0;
};

// Detects loops and merges on its own replica of the Atlas and publishes
// LoopDetected (ORB-SLAM3 NewDetectCommonRegions); the Mapper closes them.
// Consumes the Mapper's MapTransactions; runs on its own thread; never
// modifies map objects. Opt-in: unless LoopCloserOptions::enabled is set (it
// is off by default), or without a vocabulary, it only acknowledges what it
// receives.
//
// Each transaction is applied to the replica on receipt, which releases it
// (a slow detector never holds the Mapper's pooled events); every Frame it
// adds is queued as a query, the oldest dropped beyond max_query_backlog.
// Queries are detected one at a time, once the map has min_map_frames
// Frames; one culled while queued is skipped.
// There are two pending regions, each with its own confirmations and
// failures: a loop region (in the active map) and a merge region (in an
// inactive map). Each pending region (one confirmed at least once) is first
// checked against the query with RefineCommonRegion; a confirmation counts, a
// failure counts towards max_unconfirmed_queries, after which the region is
// dropped. Unless one of them is then confirmed required_coincidences times,
// DetectNBestCandidates splits the database hits into loop candidates (the
// active map) and merge candidates (the inactive maps), and DetectCommonRegion
// verifies each list whose region was not confirmed by this query; its
// region, with its covisible confirmations, becomes the pending one. At
// required_coincidences confirmations LoopDetected is published for the
// latest confirming query and the region is dropped: a merge (match_map_id
// is the inactive map's) before a loop; a loop confirmed at the same query as
// a merge is dropped unpublished (ORB-SLAM3 handles mbMergeDetected first).
// LoopDetected has priority unless one was already published at the
// replica's map epoch (a repeat found before its correction arrived).
// A higher epoch of the active map (a loop correction or a global-BA merge,
// whose coordinates differ from the pending loop region's) drops the pending
// loop region; a higher epoch or the removal of the merge region's map drops
// the pending merge region.
// The replica is an AtlasReplica (the replica rule: atlas_replica.h); queries
// are Frames of the active map only. A change of the active map (a new map,
// or none after the Mapper drops or deactivates a finally lost map) drops
// both pending regions and the queries. A TrackingLost changes nothing: the
// Mapper's transaction does.
template <typename TFeatureExtractor>
class LoopCloser : public events::EventNode {
public:
    using TVocabulary = bow::Vocabulary<TFeatureExtractor>;

    // `vocabulary` (may be null) is immutable and shared read-only with the
    // Mapper and the Tracker.
    explicit LoopCloser(LoopCloserOptions options = {}, std::shared_ptr<const TVocabulary> vocabulary = nullptr)
        : options_(options), vocabulary_(std::move(vocabulary)) {}
    // Stops the worker before any member it uses is destroyed.
    ~LoopCloser();

    // Start() and Stop() must be called from one controlling thread.
    void Start();
    void Stop();

    // Drains the inbox, applying each transaction and queueing its new Frames
    // as queries (dropping the oldest beyond max_query_backlog), then detects
    // the oldest query. Returns true iff an event or a query was processed.
    // Called by the worker thread; public so tests can drive it
    // synchronously (never concurrently with a running worker).
    [[nodiscard]] bool ProcessPending();

    // LoopCloser-thread state, for tests that drive ProcessPending()
    // synchronously: never read while the worker runs.
    [[nodiscard]] const tracking::AtlasReplica<TFeatureExtractor>& atlas() const { return atlas_; }
    // The active map; empty if there is none.
    [[nodiscard]] const tracking::MapReplica<TFeatureExtractor>& map_replica() const {
        return atlas_.active_or_empty();
    }
    // Confirmations of the pending region; 0 if none.
    [[nodiscard]] size_t num_coincidences() const { return pending_ ? pending_->num_coincidences : 0; }
    // The last DetectCommonRegion's gates.
    [[nodiscard]] const CommonRegionStats& last_stats() const { return last_stats_; }
    // Confirmations of the pending merge region; 0 if none.
    [[nodiscard]] size_t num_merge_coincidences() const {
        return pending_merge_ ? pending_merge_->num_coincidences : 0;
    }
    // The last merge detection's gates (DetectCommonRegion on the merge
    // candidates).
    [[nodiscard]] const CommonRegionStats& last_merge_stats() const { return last_merge_stats_; }
    // Loops dropped because max_events_in_flight LoopDetected were in use.
    [[nodiscard]] size_t num_dropped_loops() const { return num_dropped_loops_; }
    [[nodiscard]] const QueryStats& query_stats() const { return query_stats_; }

    // Tests: `hook` is called on the LoopCloser's thread before each query is
    // detected (a slow detector, or one the Mapper outpaces). Set it while
    // the worker is stopped.
    void set_query_hook(std::function<void(map::FrameId)> hook) { query_hook_ = std::move(hook); }

private:
    // A region being confirmed (ORB-SLAM3 mpLoopMatchedKF,
    // mpLoopLastCurrentKF, mg2oLoopSlw, mnLoopNumCoincidences,
    // mnLoopNumNotFound, and their Merge counterparts). S_last_world takes
    // the match map's world to last_query's camera.
    struct PendingRegion {
        map::FrameId match_frame;
        map::MapId match_map{0};
        uint32_t match_epoch = 0;  // the match map's epoch at detection
        map::FrameId last_query;
        geometry::Sim3 S_last_world;
        std::vector<std::optional<map::MapPointId>> matches;  // per last_query feature
        size_t num_coincidences = 0;
        size_t num_not_found = 0;
    };

    void ApplyEvent(const events::Event& event);
    // Detects `query` unless it was culled while queued; times it.
    void DetectQuery(map::FrameId query);
    void DetectRegions(map::FrameId query);
    // Checks `out_pending` (if confirmed at least once) against `query`
    // (DetectAndReffineSim3FromLastKF); true iff it confirmed. Drops it after
    // max_unconfirmed_queries failures in a row, or at once if its last query
    // or match map is gone.
    bool ConfirmPending(std::optional<PendingRegion>& out_pending, map::FrameId query);
    // Verifies `candidates` (DetectCommonRegionsFromBoW) into `out_pending`
    // unless none matches; `kind` names it in the log.
    void DetectPending(map::FrameId query, const std::vector<RegionCandidate>& candidates, const char* kind,
                       std::optional<PendingRegion>& out_pending, CommonRegionStats& out_stats);
    [[nodiscard]] bool Confirmed(const std::optional<PendingRegion>& pending) const {
        return pending && pending->num_coincidences >= options_.required_coincidences;
    }
    void PublishLoop(map::FrameId query, const PendingRegion& region);
    void LogNormalRejections(const CommonRegionStats& stats);

    const LoopCloserOptions options_;
    const std::shared_ptr<const TVocabulary> vocabulary_;
    events::Listener inbox_listener_ = inbox_.GetListener();

    // LoopCloser-thread state.
    tracking::AtlasReplica<TFeatureExtractor> atlas_;
    std::optional<PendingRegion> pending_;
    std::optional<PendingRegion> pending_merge_;
    // Frames applied, awaiting detection (at most max_query_backlog after
    // each drain of the inbox).
    std::deque<map::FrameId> queries_;
    bool dropping_queries_ = false;  // since the backlog was last empty
    // (map, epoch) of the last loop published.
    std::optional<std::pair<map::MapId, uint32_t>> last_loop_epoch_;
    CommonRegionStats last_stats_;
    CommonRegionStats last_merge_stats_;
    size_t num_dropped_loops_ = 0;
    QueryStats query_stats_;
    std::function<void(map::FrameId)> query_hook_;

    // Published with TryCreate: a loop found while every event is in flight
    // (a Mapper backlog) is dropped; the next query finds it again.
    memory::ObjectPool<events::LoopDetected, memory::ThreadSafePolicy> loop_detected_pool_{
        options_.max_events_in_flight};

    CLASS_LOGGER(LoopCloser);

    // Last member: constructed after, and stopped before, everything it uses.
    events::WorkerThread worker_{wake_signal()};
};

} // namespace voyahslam::loop_closing

#endif // VOYAHSLAM_LOOP_CLOSING_LOOP_CLOSER_H
