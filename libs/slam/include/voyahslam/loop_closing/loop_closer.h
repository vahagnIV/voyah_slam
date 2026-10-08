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
#include <voyahslam/features/projection_matcher.h>
#include <voyahslam/geometry/sim3.h>
#include <voyahslam/logging/logger.h>
#include <voyahslam/map/frame_id.h>
#include <voyahslam/map/map_id.h>
#include <voyahslam/map/map_point_id.h>
#include <voyahslam/memory/object_pool.h>
#include <voyahslam/tracking/map_replica.h>

namespace voyahslam::loop_closing {

// Loop detection (spec sections 2 and 4; ORB-SLAM3 NewDetectCommonRegions).
// The first group are the spec's values; the rest are ORB-SLAM3's, which the
// spec leaves implicit.
struct LoopCloserOptions {
    // A map with fewer Frames is not searched.
    size_t min_map_frames = 12;
    // DetectNBestCandidates: the best groups' Frames tried per query.
    size_t num_candidates = 3;
    // A candidate's window: the candidate plus its top `window` covisible
    // Frames; the match side's points: the match Frame plus its top `window`.
    size_t window = 5;
    // Per candidate: SearchByBoW matches, RANSAC inliers, OptimizeSim3
    // inliers, then matches by projection with the optimized Sim3.
    size_t min_bow_matches = 20;
    size_t min_sim3_inliers = 20;
    size_t min_optimized = 15;
    size_t min_projection = 100;
    // A covisible Frame of the query, or a later query, confirms a region with
    // at least this many matches by projection.
    size_t confirmation_projection = 50;
    // Confirmations before a region is a loop; consecutive later queries that
    // fail to confirm before the pending region is dropped.
    size_t required_coincidences = 3;
    size_t max_unconfirmed_queries = 3;
    // SolveSim3Ransac.
    int ransac_iterations = 300;
    double ransac_confidence = 0.99;
    double ransac_chi2 = 9.21;

    // ORB-SLAM3 values. Database hits share at least this fraction of the
    // best non-covisible hit's common words.
    double min_common_words_ratio = 0.8;
    // nNumCovisibles: the covisible Frames a candidate group scores over, and
    // the query's covisible Frames tried as confirmations.
    size_t num_covisibles = 10;
    // SearchByBoW against the window (ORBmatcher(0.9, true)).
    double bow_nn_ratio = 0.9;
    // SearchBySim3Projection after RANSAC (ORB-SLAM2 SearchBySim3: radius 7.5
    // px at level 0, TH_HIGH).
    double sim3_search_radius = 7.5;
    int sim3_search_max_distance = 100;
    // Projection with the optimized Sim3 (th 5, TH_LOW x 1.0) and the
    // confirmations' projection (FindMatchesByProjection: th 3, TH_LOW x 1.5);
    // radius th * level_scale^level px.
    double projection_radius = 5.0;
    double projection_max_distance = 50.0;
    double confirmation_radius = 3.0;
    double confirmation_max_distance = 75.0;
    // Residuals in raw pixels, sigma = sigma_px * level_scale^level.
    double sigma_px = 1.0;
    double level_scale = 1.2;
    // Level scale, number of levels and ray-angle limit of the projections.
    features::ProjectionMatcherOptions matcher;
};

// How far a detection got: per gate, the most any candidate reached.
struct CommonRegionStats {
    size_t num_candidates = 0;      // after DetectNBestCandidates
    size_t bow_matches = 0;
    size_t ransac_inliers = 0;
    size_t optimized_inliers = 0;
    size_t projection_matches = 0;  // with the optimized Sim3
};

// A place the query Frame shares with earlier Frames that are not covisible
// with it.
struct CommonRegion {
    // The window Frame with the most SearchByBoW matches (ORB-SLAM3
    // pMostBoWMatchesKF), the frame of S_query_match.
    map::FrameId match_frame;
    // Camera similarity, x_query = S_query_match * x_match (ORB-SLAM3 gScm):
    // the query's corrected pose is S_query_match composed with the match
    // Frame's T_cw, S_query_world.
    geometry::Sim3 S_query_match;
    geometry::Sim3 S_query_world;
    // Per query (left) feature, the match side's point it matched by
    // projection with S_query_world, or none.
    std::vector<std::optional<map::MapPointId>> matches;
    size_t num_matches = 0;
    // Covisible Frames of the query that confirmed the region (at most
    // required_coincidences).
    size_t num_coincidences = 0;
};

struct CommonRegionResult {
    std::optional<CommonRegion> region;
    CommonRegionStats stats;
};

// ORB-SLAM3 DetectNBestCandidates + DetectCommonRegionsFromBoW, visual only,
// for Frame `query_frame` of `replica` (spec section 2):
//   1. database hits that are neither the query nor covisible with it,
//      sharing >= min_common_words_ratio of the best one's words, grouped by
//      their top num_covisibles covisible Frames; the best num_candidates
//      groups give a candidate each;
//   2. per candidate, its window (skipped if any window Frame is covisible
//      with the query): SearchByBoW from the query's point-observing features
//      to each window Frame's points, each query feature and point once; at
//      least min_bow_matches;
//   3. SolveSim3Ransac between the query's points (camera coordinates) and
//      the matched points in the match Frame's camera; pixels are each point's
//      projection in its own camera (ORB-SLAM3 mvP1im1 / mvP2im2); scale fixed
//      for a binocular query; at least min_sim3_inliers;
//   4. SearchBySim3Projection against each window Frame adds matches to the
//      RANSAC inliers, then OptimizeSim3; at least min_optimized inliers;
//   5. the match Frame's points (it and its top `window` covisible Frames)
//      projected into the query with the optimized Sim3; at least
//      min_projection matches;
//   6. the query's top num_covisibles covisible Frames, until
//      required_coincidences of them confirm, each by projection with its own
//      pose relative to the query (confirmation_projection).
// A Sim3 that is not finite or has a scale <= 0 drops the candidate. The
// candidate with the most matches in step 5 wins. Deterministic. Reusable on
// another map's replica (map merging).
template <typename TFeatureExtractor>
[[nodiscard]] CommonRegionResult DetectCommonRegion(const tracking::MapReplica<TFeatureExtractor>& replica,
                                                    map::FrameId query_frame, const LoopCloserOptions& options);

// ORB-SLAM3 DetectAndReffineSim3FromLastKF: confirms a region of `match_frame`
// in a later query from the guess S_query_world (the last confirmed query's
// Sim3 moved by the replica's relative pose): projection of the match side's
// points (confirmation_projection), OptimizeSim3 over the matched query
// points (min_optimized), projection with the refined Sim3
// (confirmation_projection). None if a gate fails or a Frame is unknown.
template <typename TFeatureExtractor>
[[nodiscard]] std::optional<CommonRegion> RefineCommonRegion(const tracking::MapReplica<TFeatureExtractor>& replica,
                                                             map::FrameId query_frame, map::FrameId match_frame,
                                                             const geometry::Sim3& S_query_world,
                                                             const LoopCloserOptions& options);

// Detects loops on its own replica of the map and publishes LoopDetected
// (ORB-SLAM3 NewDetectCommonRegions); the Mapper closes them. Consumes the
// Mapper's MapTransactions; runs on its own thread; never modifies map
// objects. Without a vocabulary it only acknowledges what it receives.
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

    const LoopCloserOptions options_;
    const std::shared_ptr<const TVocabulary> vocabulary_;
    events::Listener inbox_listener_ = inbox_.GetListener();

    // LoopCloser-thread state.
    tracking::MapReplica<TFeatureExtractor> replica_;
    map::MapId lost_map_{0};
    std::optional<PendingRegion> pending_;
    CommonRegionStats last_stats_;

    memory::ObjectPool<events::LoopDetected, memory::ThreadSafePolicy> loop_detected_pool_{16};

    CLASS_LOGGER(LoopCloser);

    // Last member: constructed after, and stopped before, everything it uses.
    events::WorkerThread worker_{wake_signal()};
};

} // namespace voyahslam::loop_closing

#endif // VOYAHSLAM_LOOP_CLOSING_LOOP_CLOSER_H
