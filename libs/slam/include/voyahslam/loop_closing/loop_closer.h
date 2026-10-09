#ifndef VOYAHSLAM_LOOP_CLOSING_LOOP_CLOSER_H
#define VOYAHSLAM_LOOP_CLOSING_LOOP_CLOSER_H

#include <cstddef>
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
#include <voyahslam/tracking/map_replica.h>

namespace voyahslam::loop_closing {

// Detects loops on its own replica of the map and publishes LoopDetected
// (ORB-SLAM3 NewDetectCommonRegions); the Mapper closes them. Consumes the
// Mapper's MapTransactions; runs on its own thread; never modifies map
// objects. Without a vocabulary, or disabled (LoopCloserOptions::enabled),
// it only acknowledges what it receives.
//
// Every Frame a transaction adds is a query, once the map has min_map_frames
// Frames. A pending region (one confirmed at least once) is first checked
// against the query with RefineCommonRegion; a confirmation counts, a failure
// counts towards max_unconfirmed_queries, after which the region is dropped.
// Otherwise DetectCommonRegion runs and its region, with its covisible
// confirmations, becomes the pending one. At required_coincidences
// confirmations LoopDetected is published for the latest confirming query and
// the region is dropped. A new map (new_map, a newer map id, or a final
// TrackingLost of the current map) clears the replica and the pending region.
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

    // Drains the inbox: applies each transaction and runs detection on its
    // new Frames. Returns true iff an event was processed. Called by the
    // worker thread; public so tests can drive it synchronously (never
    // concurrently with a running worker).
    [[nodiscard]] bool ProcessPending();

    // LoopCloser-thread state, for tests that drive ProcessPending()
    // synchronously: never read while the worker runs.
    [[nodiscard]] const tracking::MapReplica<TFeatureExtractor>& replica() const { return replica_; }
    // Confirmations of the pending region; 0 if none.
    [[nodiscard]] size_t num_coincidences() const { return pending_ ? pending_->num_coincidences : 0; }
    // The last DetectCommonRegion's gates.
    [[nodiscard]] const CommonRegionStats& last_stats() const { return last_stats_; }
    // Loops dropped because max_events_in_flight LoopDetected were in use.
    [[nodiscard]] size_t num_dropped_loops() const { return num_dropped_loops_; }

private:
    // The region being confirmed (ORB-SLAM3 mpLoopMatchedKF,
    // mpLoopLastCurrentKF, mg2oLoopSlw, mnLoopNumCoincidences,
    // mnLoopNumNotFound).
    struct PendingRegion {
        map::FrameId match_frame;
        map::FrameId last_query;
        geometry::Sim3 S_last_world;
        std::vector<std::optional<map::MapPointId>> matches;  // per last_query feature
        size_t num_coincidences = 0;
        size_t num_not_found = 0;
    };

    void ApplyEvent(const events::Event& event);
    // Clears the replica and the pending region; events of maps up to
    // `lost_map` are ignored from now on.
    void ClearMap(map::MapId lost_map);
    void DetectLoop(map::FrameId query);
    void PublishLoop(map::FrameId query, const PendingRegion& region);
    void LogNormalRejections(const CommonRegionStats& stats);

    const LoopCloserOptions options_;
    const std::shared_ptr<const TVocabulary> vocabulary_;
    events::Listener inbox_listener_ = inbox_.GetListener();

    // LoopCloser-thread state.
    tracking::MapReplica<TFeatureExtractor> replica_;
    map::MapId lost_map_{0};
    std::optional<PendingRegion> pending_;
    CommonRegionStats last_stats_;
    size_t num_dropped_loops_ = 0;

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
