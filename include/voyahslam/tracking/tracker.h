#ifndef VOYAHSLAM_TRACKING_TRACKER_H
#define VOYAHSLAM_TRACKING_TRACKER_H

#include <atomic>
#include <concurrentqueue.h>
#include <cstddef>
#include <cstdint>
#include <voyahslam/defs.h>
#include <voyahslam/events/event_node.h>
#include <voyahslam/events/initialization_measurement.h>
#include <voyahslam/events/listener.h>
#include <voyahslam/events/position_updated.h>
#include <voyahslam/events/tracking_lost.h>
#include <voyahslam/events/worker_thread.h>
#include <voyahslam/features/measurement.h>
#include <voyahslam/features/stereo_matcher.h>
#include <voyahslam/geometry/rigid_transformation.h>
#include <voyahslam/logging/logger.h>
#include <voyahslam/map/frame_id.h>
#include <voyahslam/map/map_id.h>
#include <voyahslam/memory/object_pool.h>
#include <voyahslam/tracking/pose_estimator.h>

namespace voyahslam::tracking {

enum class TrackerState : uint8_t {
    WaitingForMap,  // no map yet: measurements go to the Mapper for initialization
    Tracking,       // tracking against the current map
    Lost,           // tracking failed: behaves like WaitingForMap (relocalization later)
};

struct TrackerOptions {
    features::StereoMatcherOptions stereo_matcher;
};

// Estimates the camera pose of every incoming measurement against its own
// replica of the map. Runs on its own thread with two inputs:
//   - inbox():        map diffs from the Mapper,
//   - measurements_:  monocular / binocular measurements from Track().
// Each ProcessPending() pass first applies all pending map diffs, then tracks
// one measurement, so the replica is never modified while it is being tracked
// against. Binocular measurements are stereo-matched on dequeue. Publishes InitializationMeasurement, PositionUpdated and
// TrackingLost; never modifies map objects.
template <typename TFeatureExtractor>
class Tracker : public events::EventNode {
public:
    using TMonocularMeasurement = features::TMonocularMeasurement<TFeatureExtractor>;
    using TBinocularMeasurement = features::TBinocularMeasurement<TFeatureExtractor>;
    using TMeasurement = features::TMeasurement<TFeatureExtractor>;

    explicit Tracker(TrackerOptions options = {}) : options_(options) {}
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

private:
    // Applies the consumer rule; returns false if the event is stale.
    bool AdoptMapOf(const events::Event& event);
    void ApplyEvent(const events::Event& event);
    void ProcessMeasurement(TMeasurement& measurement);
    void SetState(TrackerState state);

    void PublishInitializationMeasurement(TMeasurement& measurement);
    void PublishPositionUpdated(double timestamp);
    void PublishTrackingLost(double timestamp);

    const TrackerOptions options_;
    moodycamel::ConcurrentQueue<TMeasurement> measurements_;
    events::Listener inbox_listener_ = inbox_.GetListener();

    std::atomic<TrackerState> state_{TrackerState::WaitingForMap};
    std::atomic<map::MapId> current_map_id_{map::MapId{0}};
    map::FrameId map_frame_id_{0};  // keyframe of the current map's MapCreated
    geometry::RigidTransformation last_pose_ = geometry::RigidTransformation::Identity();
    PoseEstimator<TFeatureExtractor> pose_estimator_;

    memory::ObjectPool<events::InitializationMeasurement<TFeatureExtractor>,
                       memory::ThreadSafePolicy>
        initialization_measurement_pool_{256};
    memory::ObjectPool<events::PositionUpdated, memory::ThreadSafePolicy> position_updated_pool_{256};
    memory::ObjectPool<events::TrackingLost, memory::ThreadSafePolicy> tracking_lost_pool_{256};

    CLASS_LOGGER(Tracker);

    // Last member: constructed after, and stopped before, everything it uses.
    events::WorkerThread worker_{wake_signal()};
};

} // namespace voyahslam::tracking

#endif // VOYAHSLAM_TRACKING_TRACKER_H
