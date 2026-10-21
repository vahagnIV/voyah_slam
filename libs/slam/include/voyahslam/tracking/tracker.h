#ifndef VOYAHSLAM_TRACKING_TRACKER_H
#define VOYAHSLAM_TRACKING_TRACKER_H

#include <atomic>
#include <chrono>
#include <concurrentqueue.h>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <utility>
#include <vector>
#include <voyahslam/bow/vocabulary.h>
#include <voyahslam/defs.h>
#include <voyahslam/events/event_node.h>
#include <voyahslam/events/frame_tracked.h>
#include <voyahslam/events/initialization_measurement.h>
#include <voyahslam/events/listener.h>
#include <voyahslam/events/map_transaction.h>
#include <voyahslam/events/tracking_lost.h>
#include <voyahslam/events/worker_thread.h>
#include <voyahslam/features/measurement.h>
#include <voyahslam/features/projection_matcher.h>
#include <voyahslam/features/stereo_matcher.h>
#include <voyahslam/geometry/rigid_transformation.h>
#include <voyahslam/logging/logger.h>
#include <voyahslam/map/frame_id.h>
#include <voyahslam/map/map_id.h>
#include <voyahslam/map/map_point_id.h>
#include <voyahslam/optimization/pose_optimization.h>
#include <voyahslam/tracking/local_map.h>
#include <voyahslam/tracking/atlas_replica.h>
#include <voyahslam/tracking/map_replica.h>
#include <voyahslam/tracking/relocalizer.h>

namespace voyahslam::tracking {

enum class TrackerState : uint8_t {
    WaitingForMap,  // no map yet: measurements go to the Mapper for initialization
    Tracking,       // tracking against the current map
    // Tracking failed; relocalization in the current map is tried on every
    // measurement, nothing is published, and the Mapper keeps the map.
    RecentlyLost,
    Lost,           // tracking failed for good: behaves like WaitingForMap
};

// Frame-to-map tracking thresholds (ORB-SLAM3 TrackWithMotionModel /
// TrackLocalMap).
struct TrackingOptions {
    double window_mono_px = 15.0;       // last-frame search window, monocular
    double window_stereo_px = 7.0;      // last-frame search window, binocular
    size_t min_motion_matches = 20;     // below: retry with the window x2; still below: fail
    size_t min_motion_inliers = 10;     // after pose optimization (motion model and zero motion)
    // Motion-model step only: a pose with fewer final inliers counts as failed
    // and the zero-motion fallback runs (fast rotations otherwise yield
    // 10-29 inlier poses that drift).
    size_t min_motion_model_inliers = 30;
    double fallback_window_scale = 4.0; // zero-motion fallback window factor
    // Reference-Frame tracking (ORB-SLAM3 TrackReferenceKeyFrame), with a
    // vocabulary only: SearchByBoW ratio, minimum matches, and minimum inliers
    // after pose optimization.
    double reference_frame_nn_ratio = 0.7;
    size_t min_reference_frame_matches = 15;
    size_t min_reference_frame_inliers = 10;
    double local_map_th = 1.0;          // local-map search window factor
    size_t min_local_map_inliers = 30;
    // A frame with fewer than urgent_inlier_factor * min_local_map_inliers
    // inliers is published urgent (default: below 45).
    double urgent_inlier_factor = 1.5;
    size_t local_neighbours = 10;       // K2: top covisible Frames per K1 Frame
    size_t max_local_frames = 80;       // local-map Frames, K1 first

    [[nodiscard]] bool IsUrgent(size_t num_inliers) const {
        return static_cast<double>(num_inliers) <
               urgent_inlier_factor * static_cast<double>(min_local_map_inliers);
    }
};

struct TrackerOptions {
    features::StereoMatcherOptions stereo_matcher;
    TrackingOptions tracking;
    RelocalizationOptions relocalization;
    features::ProjectionMatcherOptions matcher;
    optimization::PoseOptimizationOptions pose_optimization;
    double sigma_px = 1.0;     // keypoint noise std-dev at level 0, px
    // sigma_level = sigma_px * level_scale^level. Separate from
    // StereoMatcherOptions::level_scale on purpose; both default to 1.2.
    // level_scale here, ProjectionMatcherOptions::level_scale and max_level
    // must match the extractor's scale factor and number of levels.
    // TODO: derive them from the extractor.
    double level_scale = 1.2;
};

// Test access to the Tracker's thread-affine state (defined by the tests).
struct TrackerTestAccess;

// Estimates the camera pose of every incoming measurement against its own
// replica of the map. Runs on its own thread with two inputs:
//   - inbox():        map diffs from the Mapper,
//   - measurements_:  monocular / binocular measurements from Track().
// Each ProcessPending() pass first applies all pending map diffs, then tracks
// one measurement, so the replica is never modified while it is being tracked
// against. Binocular measurements are stereo-matched on dequeue. Publishes
// InitializationMeasurement, FrameTracked and TrackingLost; never modifies
// map objects.
//
// Tracking (ORB-SLAM3 Track): an initial pose from the motion model
// (projection search from the last frame at velocity * T_last, pose
// optimization), else a wide-window search at zero motion, else, with a
// vocabulary, the reference Frame (bag-of-words matches against the last
// FrameTracked's reference Frame, pose optimization from the last pose; the
// FeatureVectors at the Mapper's levels_up); then
// the local map and a final pose optimization. The local map (local_map.h) is
// built from the covisibility of the Frames observing the initial step's
// matches; FrameTracked carries its Frames, the local points passing the
// projection gate, the reference Frame (the local Frame observing the most
// final inliers) and, as its stage, the initial pose's source.
//
// Loss (spec section 2). With a vocabulary, RelocalizationOptions::enabled and
// a map of more than min_map_frames_to_relocalize Frames, a failure enters RecentlyLost
// (TrackingLost, final = false): each measurement tries relocalization
// (relocalizer.h); a success resumes Tracking in the same map with the
// velocity reset, and for post_relocalization_seconds from the relocalized
// measurement on the local-map step needs post_relocalization_min_inliers;
// after recently_lost_seconds of measurement timestamps
// the Tracker is Lost (TrackingLost, final = true). Otherwise a failure is
// Lost at once (final = true). Lost, like WaitingForMap, forwards
// measurements to the Mapper for a new map.
//
// MapTransactions. They build the Tracker's AtlasReplica (the replica rule:
// atlas_replica.h); the Tracker tracks and relocalizes only in the active map
// (Tracker-thread state: apps read the map through mapping::MapFollower,
// tests through TrackerTestAccess). A change of the active map to another map resets the last
// frame, the velocity and the reference Frame; current_map_id() is the last
// active map (kept while none is active, after a final loss). Only a
// transaction of the active map acts on the Tracker's state below. A new map
// starts tracking from its start Frame. An added
// Frame whose measurement is the last tracked one (same timestamp) merges its
// observations into the last frame's matches, so the next motion-model search
// uses the Mapper's new points at once. Then the last frame's matches follow
// the transaction's replaced points (loser -> winner) and drop its removed
// points. A loop correction or a global-BA merge (a transaction raising the map epoch) moves the
// world under the last frame: while Tracking, the last frame keeps its pose
// relative to its reference Frame, T_last <- T_last * T_ref_old^-1 *
// T_ref_new (loop-closing spec section 3), or, if the reference Frame is gone,
// relative to the Frame observing most of its matches; the velocity is
// relative and unchanged. While RecentlyLost or Lost there is no last frame:
// relocalization and a new map use the corrected replica. FrameTracked
// carries the replica's epoch.
//
// Map merging (map-merging spec section 4). A transaction merging the active
// map M into map N (absorbed_map M, active_map N) switches the Tracker to N
// without a reset: its state, velocity and reference Frame are kept (Frames
// keep their ids in N). While Tracking, the last frame follows its reference
// Frame (or another Frame of its matches) as through a loop correction; its
// matches take their ids in N by the merge's id rule and fusions
// (AtlasReplica::TranslatePoint), and one that has none becomes unmatched.
// While RecentlyLost or Lost there is no last frame: relocalization searches
// N. FrameTracked then carries N and N's epoch. A merge landing while Lost
// overtook the Tracker's final TrackingLost(M) in the Mapper, which ignores
// that loss as stale: the Tracker publishes a final TrackingLost(N) (with the
// first one's timestamp), and the Mapper handles it as any final loss.
template <typename TFeatureExtractor>
class Tracker : public events::EventNode {
public:
    using TMonocularMeasurement = features::TMonocularMeasurement<TFeatureExtractor>;
    using TBinocularMeasurement = features::TBinocularMeasurement<TFeatureExtractor>;
    using TMeasurement = features::TMeasurement<TFeatureExtractor>;
    using TSharedMeasurement = features::TSharedMeasurement<TFeatureExtractor>;

    // `vocabulary` (may be null) is immutable and shared read-only with the
    // Mapper.
    explicit Tracker(TrackerOptions options = {},
                     std::shared_ptr<const bow::Vocabulary<TFeatureExtractor>> vocabulary = nullptr)
        : EventNode("Tracker"), options_(options), vocabulary_(std::move(vocabulary)) {}
    // Stops the worker before any member it uses is destroyed.
    ~Tracker();

    // Lifecycle (D-10..D-12): Created -> Running -> Stopped, Stopped is terminal.
    // Start() twice or after Stop() throws UsageError; Stop() before Start() moves
    // to Stopped; Stop() is idempotent and every concurrent call returns only after
    // the worker is joined. Both are safe from any thread, except that Stop() on
    // the Tracker's own worker thread throws UsageError (it would join itself), in
    // every state, also while another thread is stopping or starting it.
    void Start();
    void Stop();
    // True on this component's own worker thread (a sync handler or other callback
    // it runs), where Stop() is refused with UsageError. Any thread.
    [[nodiscard]] bool OnWorkerThread() const { return worker_.OnWorkerThread(); }

    // Enqueue the measurement for the Tracker thread and return immediately.
    // Taken by value: pass an rvalue to move the feature matrices instead of
    // copying them. Before Start() it keeps queuing; during or after Stop() it
    // throws UsageError (D-12).
    void Track(TMonocularMeasurement measurement);
    void Track(TBinocularMeasurement measurement);

    // Safe to call from any thread.
    [[nodiscard]] TrackerState state() const { return state_.load(std::memory_order_acquire); }
    [[nodiscard]] map::MapId current_map_id() const {
        return current_map_id_.load(std::memory_order_acquire);
    }

    // Approximate number of measurements waiting to be tracked.
    [[nodiscard]] size_t pending_measurements() const { return measurements_.size_approx(); }
    // Measurements processed (tracked, lost or forwarded to the Mapper), in
    // total; each counted after whatever it published.
    [[nodiscard]] size_t num_processed() const { return num_processed_.load(std::memory_order_acquire); }

private:
    friend struct TrackerTestAccess;

    // One pass of the Tracker loop: drains the inbox, then tracks at most one
    // measurement. Returns true iff an event or a measurement was processed.
    // Called by the worker thread; tests drive it synchronously through
    // TrackerTestAccess::ProcessPending (never concurrently with a running worker).
    [[nodiscard]] bool ProcessPending();

    // Tracker-thread state; tests that drive ProcessPending() synchronously
    // read it through TrackerTestAccess. Apps read the map through
    // mapping::MapFollower.
    [[nodiscard]] const AtlasReplica<TFeatureExtractor>& atlas() const { return atlas_; }
    // The active map, which the Tracker tracks and relocalizes in; empty if
    // there is none.
    [[nodiscard]] const MapReplica<TFeatureExtractor>& map_replica() const { return atlas_.active_or_empty(); }
    // The last tracked frame's inlier matches: per (left) feature, the
    // MapPointId value or -1.
    [[nodiscard]] const std::vector<int64_t>& last_matches() const { return last_frame_.matches; }

    struct LastFrame {
        // Null when there is no last frame. A tracked frame's measurement is
        // created here (AcceptPose), once, and shared by its FrameTracked.
        TSharedMeasurement measurement;
        geometry::RigidTransformation T_cw;
        std::vector<int64_t> matches;  // per (left) feature: MapPointId value or -1
    };

    // A pose with its matches, after outliers are dropped.
    struct PoseResult {
        geometry::RigidTransformation T_cw;
        std::vector<int64_t> matches;
        size_t num_matches = 0;   // before the optimization
        size_t num_inliers = 0;
        events::TrackingStage stage = events::TrackingStage::MotionModel;
        // Local map step.
        std::vector<map::FrameId> local_frames;
        std::vector<map::MapPointId> visible_points;
        std::optional<map::FrameId> reference_frame_id;
    };

    // Applies a transaction to the Atlas (the replica rule). A merge of the
    // active map into the map it makes active is followed (FollowMerge); any
    // other change of the active map adopts the new one (AdoptMap), and a
    // merge of the active map that leaves none active resets as AdoptMap of
    // the target. A merge that does not absorb the active map leaves the
    // Tracker alone; one into the active map is a correction of it (its
    // epoch rises; the active map's Frames and points keep their ids).
    void ApplyEvent(const events::Event& event);
    // The active map became `map_id`: nothing tracked against the previous
    // one carries over.
    void AdoptMap(map::MapId map_id);
    // The active map `absorbed` was merged into the new active map (already
    // applied): the Tracker switches to it, keeping its state, and
    // last_frame_'s points take their ids in it
    // (AtlasReplica::TranslatePoint); a point without one becomes unmatched.
    // While Lost, the final loss is published again for the new map.
    void FollowMerge(map::MapId absorbed);
    void OnMapTransaction(const events::MapTransaction<TFeatureExtractor>& transaction);
    // A Frame's pose before a loop correction is applied.
    struct FramePose {
        map::FrameId frame;
        geometry::RigidTransformation T_cw;
    };
    // The Frames last_frame_ may follow through a loop correction or a merge,
    // with their current poses: the reference Frame first, then the Frames
    // observing its matches, most matches first. Empty unless Tracking.
    [[nodiscard]] std::vector<FramePose> AnchorsBeforeCorrection() const;
    // After the correction or merge: last_frame_ keeps its pose relative to
    // the first anchor still in the (new) active map; unchanged if none is.
    void FollowCorrection(const std::vector<FramePose>& anchors);
    // Throws UsageError when the Tracker is Stopping or Stopped (Track()).
    void ThrowIfStopped() const;
    // Makes the replica's Frame `start_frame` the last frame and starts tracking.
    void StartTracking(map::FrameId start_frame);
    // Merges `frame`'s observations into last_frame_.matches if it is made of
    // last_frame_'s measurement.
    void MergeOwnFrame(const events::NewFrame<TFeatureExtractor>& frame);
    void ProcessMeasurement(TMeasurement& measurement);
    // Tracks `measurement` while Tracking: publishes FrameTracked, or enters
    // RecentlyLost or Lost.
    void TrackMeasurement(TMeasurement& measurement);
    // While RecentlyLost: relocalizes `measurement`, or times out to Lost.
    void ProcessRecentlyLost(TMeasurement& measurement);
    // Makes `measurement`, tracked as `pose`, the last frame and publishes it.
    void AcceptPose(TMeasurement& measurement, PoseResult pose);
    // Steps 1-3 of the tracking flow; on success T_cw and the inlier matches.
    std::optional<PoseResult> TrackFrame(const TMeasurement& measurement);
    // Projection search from the last frame at `predicted` (retried with the
    // window x2 below min_motion_matches), then pose optimization.
    std::optional<PoseResult> TrackFromLastFrame(const TMeasurement& measurement,
                                                 const geometry::RigidTransformation& predicted,
                                                 double window_px, events::TrackingStage stage);
    // Bag-of-words matches against the reference Frame, then pose optimization
    // from the last frame's pose (spec section 3, steps 1-3).
    std::optional<PoseResult> TrackReferenceFrame(const TMeasurement& measurement);
    // Relocalization in the current map (spec section 4, Relocalize), then the
    // local-map step; the reference is the matched Frame, the stage
    // Relocalization.
    std::optional<PoseResult> TryRelocalize(const TMeasurement& measurement);
    std::optional<PoseResult> TrackLocalMap(const TMeasurement& measurement, PoseResult pose);
    // Optimizes `initial_T_cw` against the matches and drops the outliers from
    // them; on an optimization failure every match is dropped.
    PoseResult OptimizeAndDropOutliers(const TMeasurement& measurement,
                                       const geometry::RigidTransformation& initial_T_cw,
                                       std::vector<int64_t> matches) const;
    [[nodiscard]] PoseRefinement Refinement() const {
        return {options_.pose_optimization, options_.sigma_px, options_.level_scale};
    }
    void SetState(TrackerState state);

    void PublishInitializationMeasurement(TMeasurement& measurement);
    // Publishes last_frame_, just tracked as `pose`.
    void PublishFrameTracked(const PoseResult& pose);
    void PublishTrackingLost(double timestamp, bool final);

    const TrackerOptions options_;
    const std::shared_ptr<const bow::Vocabulary<TFeatureExtractor>> vocabulary_;
    moodycamel::ConcurrentQueue<TMeasurement> measurements_;
    events::Listener inbox_listener_ = inbox_.GetListener();

    std::atomic<TrackerState> state_{TrackerState::WaitingForMap};
    std::atomic<map::MapId> current_map_id_{map::MapId{0}};
    std::atomic<size_t> num_processed_{0};

    // Tracker-thread state.
    AtlasReplica<TFeatureExtractor> atlas_;
    LastFrame last_frame_;
    std::optional<geometry::RigidTransformation> velocity_;  // T_cur_last
    // The last FrameTracked's reference Frame (the start Frame of a new map).
    std::optional<map::FrameId> reference_frame_id_;
    double recently_lost_since_ = 0.0;  // timestamp of the measurement that failed
    double final_lost_at_ = 0.0;  // timestamp of the last final TrackingLost
    // Timestamp of the last relocalized measurement in the current map.
    std::optional<double> relocalized_at_;
    size_t tracked_frames_ = 0;  // in the current map
    std::chrono::steady_clock::time_point last_info_log_{};

    CLASS_LOGGER(Tracker);

    // Last member: constructed after, and stopped before, everything it uses.
    events::WorkerThread worker_{wake_signal()};
};

} // namespace voyahslam::tracking

#endif // VOYAHSLAM_TRACKING_TRACKER_H
