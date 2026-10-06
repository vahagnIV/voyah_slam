#ifndef VOYAHSLAM_MAPPING_MAPPER_H
#define VOYAHSLAM_MAPPING_MAPPER_H

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <vector>
#include <voyahslam/events/event_node.h>
#include <voyahslam/events/event_types.h>
#include <voyahslam/events/frame_tracked.h>
#include <voyahslam/events/initialization_measurement.h>
#include <voyahslam/events/key_frame_created.h>
#include <voyahslam/events/map_created.h>
#include <voyahslam/events/tracking_lost.h>
#include <voyahslam/events/worker_thread.h>
#include <voyahslam/features/measurement.h>
#include <voyahslam/logging/logger.h>
#include <voyahslam/map/covisibility_index.h>
#include <voyahslam/map/frame.h>
#include <voyahslam/map/frame_id.h>
#include <voyahslam/map/map_id.h>
#include <voyahslam/map/map_point_storage.h>
#include <voyahslam/map/observation_storage.h>
#include <voyahslam/mapping/keyframe_decision.h>
#include <voyahslam/mapping/map_initializer.h>
#include <voyahslam/mapping/triangulation.h>
#include <voyahslam/memory/object_pool.h>

namespace voyahslam::mapping {

struct MapperOptions {
    MapInitializerOptions initializer;
    KeyFrameOptions keyframe;            // when a tracked measurement becomes a Frame
    TriangulationOptions triangulation;  // new points against neighbour Frames
    size_t neighbours_mono = 30;         // Frames triangulated against, by covisibility
    size_t neighbours_stereo = 10;
    size_t min_close_points = 100;       // stereo close points created at least per Frame
    double min_baseline_depth_ratio = 0.01;  // mono: skip a neighbour below baseline / depth
};

enum class MapperState : uint8_t {
    Initializing,  // no map: builds one from the Tracker's InitializationMeasurements
    Mapping,       // extends the current map
};

// Owns the authoritative map and is its only writer. Builds the map from the
// Tracker's InitializationMeasurements; then, per FrameTracked, decides
// (ShouldCreateFrame) whether the tracked measurement becomes a Frame and
// creates it with its tracked observations, stereo close points and points
// triangulated against covisible Frames. Each Frame insertion is published
// as one KeyFrameCreated (a new map is then announced by MapCreated). A
// TrackingLost of the current map discards it.
// LoopDetected is not consumed yet. Runs on its own thread; each
// ProcessPending() pass drains the inbox.
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
    // FrameTracked events evaluated by the Frame creation rule (not skipped).
    [[nodiscard]] size_t num_frame_evaluations() const {
        return num_frame_evaluations_.load(std::memory_order_acquire);
    }
    // FrameTracked events of the current map skipped, in total, because a
    // newer FrameTracked (or a TrackingLost) of the same map was in the same
    // drained batch.
    [[nodiscard]] size_t backlog_skipped() const {
        return backlog_skipped_.load(std::memory_order_acquire);
    }

    // The current map's observations. Mapper thread (or a test driving
    // ProcessPending synchronously) only.
    [[nodiscard]] const map::CovisibilityIndex& covisibility() const { return covisibility_; }

private:
    // Marks in superseded_ each FrameTracked of batch_ that is followed in the
    // batch by a FrameTracked or a TrackingLost of the same map.
    void MarkSupersededFrameTracked();
    void ApplyEvent(const events::Event& event);
    // A superseded FrameTracked: counted in the backlog if it is of the map
    // being extended.
    void SkipFrameTracked(const events::Event& event);
    void OnInitializationMeasurement(const events::InitializationMeasurement<TFeatureExtractor>& event);
    void OnFrameTracked(const events::FrameTracked<TFeatureExtractor>& event);
    void OnTrackingLost(const events::TrackingLost& event);
    void SetState(MapperState state);
    // Empties the current map (Frames, points, observations).
    void ClearMap();

    // The current map's Frame `id`; nullptr if it is not in the map.
    [[nodiscard]] const TKeyFrame* FindKeyFrame(map::FrameId id) const;
    // `point`'s observations as ORB-SLAM3 counts them for ref_points: 2 for a
    // stereo-matched one (right index >= 0), 1 otherwise. Unlike
    // CovisibilityIndex::NumObservations, which counts Frames.
    [[nodiscard]] size_t WeightedObservations(map::MapPointId point) const;
    // The inputs to ShouldCreateFrame for `event` (spec section 3.2).
    [[nodiscard]] KeyFrameSignals MeasureSignals(
        const events::FrameTracked<TFeatureExtractor>& event) const;
    // Inserts `event`'s measurement as a new Frame with its tracked
    // observations, then new points (stereo close points, then points
    // triangulated against neighbour Frames), and publishes the insertion.
    void CreateFrame(const events::FrameTracked<TFeatureExtractor>& event);
    // Stores `frame`'s observation of `point` through its (left) feature
    // `feature`, a stereo-matched feature of a binocular Frame with its depth
    // and right index, and indexes it in covisibility_. Returns it as an
    // AddedObservation.
    events::AddedObservation AddObservation(const TKeyFrame& frame, uint32_t feature,
                                            map::MapPointId point);
    // Adds a point at `position` observed by `keyframe`'s `feature`: that
    // feature's descriptor, the mean viewing direction from `centres` as its
    // normal; the observation goes into `out_event`'s Frame part.
    map::MapPointId AddNewPoint(const geometry::TPoint3DF& position, const TKeyFrame& keyframe,
                                uint32_t feature, const std::vector<geometry::TPoint3DF>& centres,
                                events::KeyFrameCreated<TFeatureExtractor>& out_event);
    // Binocular `keyframe`: a point per untracked stereo-matched feature
    // closer than the close depth, and at least for the min_close_points
    // closest, at its stereo back-projection. Marks their features in
    // `out_has_point`.
    void AddClosePoints(const TKeyFrame& keyframe, std::vector<bool>& out_has_point,
                        events::KeyFrameCreated<TFeatureExtractor>& out_event);
    // Triangulates `keyframe`'s features without a point (`has_point` false)
    // against its covisible neighbour Frames, skipping a neighbour with too
    // small a baseline. The neighbour's observation of each new point goes
    // into `out_event`'s added_observations.
    void TriangulateWithNeighbours(const TKeyFrame& keyframe, std::vector<bool> has_point,
                                   events::KeyFrameCreated<TFeatureExtractor>& out_event);
    // `frame`'s side of a triangulation, with `has_point` per feature.
    [[nodiscard]] TriangulationView<TFeatureExtractor> ViewOf(const TKeyFrame& frame,
                                                              std::vector<bool> has_point) const;
    // Per (left) feature of `frame`: whether it observes a map point.
    [[nodiscard]] std::vector<bool> FeaturesWithPoints(const TKeyFrame& frame) const;
    // The median camera-frame depth of `frame`'s alive points; nullopt if it
    // observes none.
    [[nodiscard]] std::optional<double> SceneMedianDepth(const TKeyFrame& frame) const;
    // Recomputes the normal and representative descriptor of `points` from all
    // their observations, stores them and writes them, with the unchanged
    // positions, into `out_event`'s updated part.
    void UpdatePointStatistics(const std::vector<map::MapPointId>& points,
                               events::KeyFrameCreated<TFeatureExtractor>& out_event);

    // Stores the initial map points and their observations: in both keyframes
    // (monocular) or in the one stereo keyframe, with depth and right index.
    void AddInitialMapPoints(const std::vector<InitialMapPoint>& map_points);

    // Publishes the insertion of `keyframe`; with `with_new_points`, every
    // point of the (fresh) map is new in it.
    void PublishKeyFrameCreated(const TKeyFrame& keyframe, bool with_new_points);
    void PublishMapCreated(const TKeyFrame& keyframe);

    MapperOptions options_;
    // One ProcessPending() batch: the events drained from the inbox and, per
    // event, whether it is a superseded FrameTracked.
    std::vector<events::Event*> batch_;
    std::vector<bool> superseded_;
    size_t backlog_ = 0;  // FrameTracked skipped since the last evaluated one
    std::atomic<size_t> num_frame_evaluations_{0};
    std::atomic<size_t> backlog_skipped_{0};

    std::atomic<MapperState> state_{MapperState::Initializing};
    std::atomic<map::MapId> current_map_id_{map::MapId{0}};
    map::MapId next_map_id_{1};
    map::FrameId next_frame_id_{0};  // never reset: frame ids are unique across maps
    std::vector<TKeyFrame> keyframes_;
    std::atomic<size_t> num_keyframes_{0};  // keyframes_.size(), readable from any thread
    // The current map's points (ids restart at 0 per map) and observations:
    // monocular keyframes' in observations_, binocular keyframes' (left
    // feature, depth, right feature) in stereo_observations_.
    map::MapPointStorage<TFeatureExtractor> map_points_;
    map::ObservationStorage<map::ObservationType::Monocular> observations_;
    map::ObservationStorage<map::ObservationType::Stereo> stereo_observations_;
    std::atomic<size_t> num_map_points_{0};  // map_points_.size(), readable from any thread
    // Both kinds of observations, indexed per point and per Frame.
    map::CovisibilityIndex covisibility_;
    MapInitializer<TFeatureExtractor> initializer_;

    memory::ObjectPool<events::KeyFrameCreated<TFeatureExtractor>, memory::ThreadSafePolicy>
        key_frame_created_pool_{256};
    memory::ObjectPool<events::MapCreated, memory::ThreadSafePolicy> map_created_pool_{256};

    CLASS_LOGGER(Mapper);

    // Last member: constructed after, and stopped before, everything it uses.
    events::WorkerThread worker_{wake_signal()};
};

} // namespace voyahslam::mapping

#endif // VOYAHSLAM_MAPPING_MAPPER_H
