#ifndef VOYAHSLAM_MAPPING_MAPPER_H
#define VOYAHSLAM_MAPPING_MAPPER_H

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <vector>
#include <voyahslam/events/event_node.h>
#include <voyahslam/events/initialization_measurement.h>
#include <voyahslam/events/key_frame_created.h>
#include <voyahslam/events/listener.h>
#include <voyahslam/events/map_created.h>
#include <voyahslam/events/map_points_added.h>
#include <voyahslam/events/tracking_lost.h>
#include <voyahslam/events/worker_thread.h>
#include <voyahslam/features/measurement.h>
#include <voyahslam/logging/logger.h>
#include <voyahslam/map/frame.h>
#include <voyahslam/map/frame_id.h>
#include <voyahslam/map/map_id.h>
#include <voyahslam/map/map_point_storage.h>
#include <voyahslam/map/observation_storage.h>
#include <voyahslam/mapping/map_initializer.h>
#include <voyahslam/memory/object_pool.h>

namespace voyahslam::mapping {

struct MapperOptions {
    MapInitializerOptions initializer;
};

enum class MapperState : uint8_t {
    Initializing,  // no map: builds one from the Tracker's InitializationMeasurements
    Mapping,       // extends the current map
};

// Owns the authoritative map and is its only writer: creates keyframes and map
// points, runs local BA and culling, and performs loop closure. Publishes map
// diffs; consumes InitializationMeasurement, TrackingLost and
// NewKeyFrameCandidate (Tracker) and LoopDetected (LoopCloser). Runs on its own
// thread; each ProcessPending() pass drains the inbox.
template <typename TFeatureExtractor>
class Mapper : public events::EventNode {
public:
    using TMeasurement = features::TMeasurement<TFeatureExtractor>;
    using TKeyFrame = map::Frame<TFeatureExtractor, TMeasurement>;

    explicit Mapper(MapperOptions options = {});
    // Stops the worker before any member it uses is destroyed.
    ~Mapper();

    // Start() and Stop() must be called from one controlling thread.
    void Start();
    void Stop();

    // Drains the inbox. Returns true iff an event was processed. Called by the
    // worker thread; public so tests can drive it synchronously (never
    // concurrently with a running worker).
    [[nodiscard]] bool ProcessPending();

    // Safe to call from any thread.
    [[nodiscard]] MapperState state() const { return state_.load(std::memory_order_acquire); }
    [[nodiscard]] map::MapId current_map_id() const {
        return current_map_id_.load(std::memory_order_acquire);
    }
    [[nodiscard]] size_t num_keyframes() const {
        return num_keyframes_.load(std::memory_order_acquire);
    }
    [[nodiscard]] size_t num_map_points() const {
        return num_map_points_.load(std::memory_order_acquire);
    }

private:
    void ApplyEvent(const events::Event& event);
    void OnInitializationMeasurement(const events::InitializationMeasurement<TFeatureExtractor>& event);
    void OnTrackingLost(const events::TrackingLost& event);
    void SetState(MapperState state);

    // Stores the initial map points and their observations in both keyframes.
    void AddInitialMapPoints(const std::vector<InitialMapPoint>& map_points);

    void PublishMapPointsAdded();
    void PublishKeyFrameCreated(const TKeyFrame& keyframe);
    void PublishMapCreated(const TKeyFrame& keyframe);

    events::Listener inbox_listener_ = inbox_.GetListener();

    std::atomic<MapperState> state_{MapperState::Initializing};
    std::atomic<map::MapId> current_map_id_{map::MapId{0}};
    map::MapId next_map_id_{1};
    map::FrameId next_frame_id_{0};  // never reset: frame ids are unique across maps
    std::vector<TKeyFrame> keyframes_;
    std::atomic<size_t> num_keyframes_{0};  // keyframes_.size(), readable from any thread
    // The current map's points (ids restart at 0 per map) and observations.
    map::MapPointStorage<TFeatureExtractor> map_points_;
    map::ObservationStorage<map::ObservationType::Monocular> observations_;
    std::atomic<size_t> num_map_points_{0};  // map_points_.size(), readable from any thread
    MapInitializer<TFeatureExtractor> initializer_;

    memory::ObjectPool<events::KeyFrameCreated<TFeatureExtractor>, memory::ThreadSafePolicy>
        key_frame_created_pool_{256};
    memory::ObjectPool<events::MapCreated, memory::ThreadSafePolicy> map_created_pool_{256};
    memory::ObjectPool<events::MapPointsAdded<TFeatureExtractor>, memory::ThreadSafePolicy>
        map_points_added_pool_{256};

    CLASS_LOGGER(Mapper);

    // Last member: constructed after, and stopped before, everything it uses.
    events::WorkerThread worker_{wake_signal()};
};

} // namespace voyahslam::mapping

#endif // VOYAHSLAM_MAPPING_MAPPER_H
