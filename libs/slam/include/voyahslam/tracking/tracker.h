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
#include <voyahslam/memory/object_pool.h>
#include <voyahslam/optimization/pose_optimization.h>
#include <voyahslam/tracking/local_map.h>
#include <voyahslam/tracking/map_replica.h>

namespace voyahslam::tracking {

enum class TrackerState : uint8_t {
    WaitingForMap,  // no map yet: measurements go to the Mapper for initialization
    Tracking,       // tracking against the current map
    Lost,           // tracking failed: behaves like WaitingForMap (relocalization later)
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
// Tracking (ORB-SLAM3 Track without BoW): the motion model (projection search
// from the last frame at velocity * T_last, pose optimization), else a
// wide-window search at zero motion, then the local map and a final pose
// optimization. The local map (local_map.h) is built from the covisibility of
// the Frames observing that step's matches; FrameTracked carries its Frames,
// the local points passing the projection gate, and the reference Frame (the
// local Frame observing the most final inliers).
//
// MapTransactions. A new map starts tracking from its start Frame. An added
// Frame whose measurement is the last tracked one (same timestamp) merges its
// observations into the last frame's matches, so the next motion-model search
// uses the Mapper's new points at once. Then the last frame's matches follow
// the transaction's replaced points (loser -> winner) and drop its removed
// points.
template <typename TFeatureExtractor>
class Tracker : public events::EventNode {
public:
    using TMonocularMeasurement = features::TMonocularMeasurement<TFeatureExtractor>;
    using TBinocularMeasurement = features::TBinocularMeasurement<TFeatureExtractor>;
    using TMeasurement = features::TMeasurement<TFeatureExtractor>;

    // `vocabulary` (may be null) is immutable and shared read-only with the
    // Mapper.
    explicit Tracker(TrackerOptions options = {},
                     std::shared_ptr<const bow::Vocabulary<TFeatureExtractor>> vocabulary = nullptr)
        : options_(options), vocabulary_(std::move(vocabulary)) {}
    // Stops the worker before any member it uses is destroyed.
    ~Tracker();

    // Start() and Stop() must be called from one controlling thread.
    void Start();
    void Stop();

    // Enqueue the measurement for the Tracker thread and return immediately.
    // Taken by value: pass an rvalue to move the feature matrices instead of
    // copying them.
    void Track(TMonocularMeasurement measurement);
    void Track(TBinocularMeasurement measurement);

    // One pass of the Tracker loop: drains the inbox, then tracks at most one
    // measurement. Returns true iff an event or a measurement was processed.
    // Called by the worker thread; public so tests can drive it synchronously
    // (never concurrently with a running worker).
    [[nodiscard]] bool ProcessPending();

    // Safe to call from any thread.
    [[nodiscard]] TrackerState state() const { return state_.load(std::memory_order_acquire); }
    [[nodiscard]] map::MapId current_map_id() const {
        return current_map_id_.load(std::memory_order_acquire);
    }

    // Approximate number of measurements waiting to be tracked.
    [[nodiscard]] size_t pending_measurements() const { return measurements_.size_approx(); }

    // Tracker-thread state, for tests that drive ProcessPending()
    // synchronously: never read while the worker runs.
    [[nodiscard]] const MapReplica<TFeatureExtractor>& replica() const { return replica_; }
    // The last tracked frame's inlier matches: per (left) feature, the
    // MapPointId value or -1.
    [[nodiscard]] const std::vector<int64_t>& last_matches() const { return last_frame_.matches; }

private:
    struct LastFrame {
        TMeasurement measurement;
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

    // Applies the consumer rule; returns false if the event is stale.
    bool AdoptMapOf(const events::Event& event);
    void ApplyEvent(const events::Event& event);
    void OnMapTransaction(const events::MapTransaction<TFeatureExtractor>& transaction);
    // Makes the replica's Frame `start_frame` the last frame and starts tracking.
    void StartTracking(map::FrameId start_frame);
    // Merges `frame`'s observations into last_frame_.matches if it is made of
    // last_frame_'s measurement.
    void MergeOwnFrame(const events::NewFrame<TFeatureExtractor>& frame);
    void ProcessMeasurement(TMeasurement& measurement);
    // Steps 1-3 of the tracking flow; on success T_cw and the inlier matches.
    std::optional<PoseResult> TrackFrame(const TMeasurement& measurement);
    // Projection search from the last frame at `predicted` (retried with the
    // window x2 below min_motion_matches), then pose optimization.
    std::optional<PoseResult> TrackFromLastFrame(const TMeasurement& measurement,
                                                 const geometry::RigidTransformation& predicted,
                                                 double window_px, events::TrackingStage stage);
    std::optional<PoseResult> TrackLocalMap(const TMeasurement& measurement, PoseResult pose);
    // Optimizes `initial_T_cw` against the matches and drops the outliers from
    // them; on an optimization failure every match is dropped.
    PoseResult OptimizeAndDropOutliers(const TMeasurement& measurement,
                                       const geometry::RigidTransformation& initial_T_cw,
                                       std::vector<int64_t> matches) const;
    void SetState(TrackerState state);

    void PublishInitializationMeasurement(TMeasurement& measurement);
    // Publishes last_frame_, just tracked as `pose`.
    void PublishFrameTracked(const PoseResult& pose);
    void PublishTrackingLost(double timestamp);

    const TrackerOptions options_;
    const std::shared_ptr<const bow::Vocabulary<TFeatureExtractor>> vocabulary_;
    moodycamel::ConcurrentQueue<TMeasurement> measurements_;
    events::Listener inbox_listener_ = inbox_.GetListener();

    std::atomic<TrackerState> state_{TrackerState::WaitingForMap};
    std::atomic<map::MapId> current_map_id_{map::MapId{0}};

    // Tracker-thread state.
    MapReplica<TFeatureExtractor> replica_;
    LastFrame last_frame_;
    std::optional<geometry::RigidTransformation> velocity_;  // T_cur_last
    size_t tracked_frames_ = 0;  // in the current map
    std::chrono::steady_clock::time_point last_info_log_{};

    memory::ObjectPool<events::InitializationMeasurement<TFeatureExtractor>,
                       memory::ThreadSafePolicy>
        initialization_measurement_pool_{256};
    memory::ObjectPool<events::FrameTracked<TFeatureExtractor>, memory::ThreadSafePolicy>
        frame_tracked_pool_{256};
    memory::ObjectPool<events::TrackingLost, memory::ThreadSafePolicy> tracking_lost_pool_{256};

    CLASS_LOGGER(Tracker);

    // Last member: constructed after, and stopped before, everything it uses.
    events::WorkerThread worker_{wake_signal()};
};

} // namespace voyahslam::tracking

#endif // VOYAHSLAM_TRACKING_TRACKER_H
