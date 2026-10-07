#ifndef VOYAHSLAM_MAPPING_MAPPER_H
#define VOYAHSLAM_MAPPING_MAPPER_H

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <unordered_map>
#include <unordered_set>
#include <vector>
#include <voyahslam/events/event_node.h>
#include <voyahslam/events/event_types.h>
#include <voyahslam/events/frame_tracked.h>
#include <voyahslam/events/initialization_measurement.h>
#include <voyahslam/events/map_transaction.h>
#include <voyahslam/events/tracking_lost.h>
#include <voyahslam/events/worker_thread.h>
#include <voyahslam/features/fuse_matcher.h>
#include <voyahslam/features/measurement.h>
#include <voyahslam/logging/logger.h>
#include <voyahslam/map/covisibility_index.h>
#include <voyahslam/map/frame.h>
#include <voyahslam/map/frame_id.h>
#include <voyahslam/map/map_id.h>
#include <voyahslam/map/map_point_storage.h>
#include <voyahslam/map/point_reference.h>
#include <voyahslam/mapping/keyframe_decision.h>
#include <voyahslam/mapping/map_initializer.h>
#include <voyahslam/mapping/map_quality.h>
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
    MapQualityOptions map_quality;       // fusion targets, recent-point culling, ...
    features::FuseOptions fuse;          // the fusion match gate
};

enum class MapperState : uint8_t {
    Initializing,  // no map: builds one from the Tracker's InitializationMeasurements
    Mapping,       // extends the current map
};

// Owns the authoritative map and is its only writer. Builds the map from the
// Tracker's InitializationMeasurements; then, per FrameTracked, decides
// (ShouldCreateFrame) whether the tracked measurement becomes a Frame and
// creates it with its tracked observations, stereo close points and points
// triangulated against covisible Frames. Every FrameTracked of the current map,
// evaluated or skipped, counts its visible and found points. Around a Frame's
// creation the Mapper first culls poor recent points, then fuses the new
// Frame with its neighbours (spec sections 3, 4.1). Every change is published
// as one MapTransaction: a new map with all its Frames and points, or
// transaction 1 of a created Frame (culling, creation, fusion). A
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
    // The current map's point `point`: its reference observation, nullopt if
    // the point is not alive; how many FrameTracked found it in their view
    // (visible) and matched it as an inlier (found), counting 1 each from its
    // creation. Same thread rule as covisibility().
    [[nodiscard]] std::optional<map::PointReference> Reference(map::MapPointId point) const;
    [[nodiscard]] uint32_t NumVisible(map::MapPointId point) const;
    [[nodiscard]] uint32_t NumFound(map::MapPointId point) const;

private:
    // A point created with Frame `created`, culled or graduated by
    // CullRecentPoints (spec section 4.1).
    struct RecentPoint {
        map::MapPointId id;
        map::FrameId created;
        bool stereo;  // created by a binocular Frame
    };

    // Marks in superseded_ each FrameTracked of batch_ that is followed in the
    // batch by a FrameTracked or a TrackingLost of the same map.
    void MarkSupersededFrameTracked();
    void ApplyEvent(const events::Event& event);
    // A superseded FrameTracked: counted in the backlog, and its visible and
    // found points counted, if it is of the map being extended.
    void SkipFrameTracked(const events::Event& event);
    // Increments the visible and found counters of `event`'s points.
    void CountTrackedPoints(const events::FrameTracked<TFeatureExtractor>& event);
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
    // Transaction 1 of a new Frame (spec section 1, steps 1-3): culls poor
    // recent points, inserts `event`'s measurement as a new Frame with its
    // tracked observations and new points (stereo close points, then points
    // triangulated against neighbour Frames), fuses it with its neighbours,
    // and publishes all of it as one MapTransaction.
    void CreateFrame(const events::FrameTracked<TFeatureExtractor>& event);
    // Stores `frame`'s observation of `point` through its (left) feature
    // `feature` (a stereo-matched feature of a binocular Frame with its depth
    // and right index), indexes it in covisibility_ and gives a point without
    // a reference this one. Records it in `out_event`: in the new Frame's part
    // (the last added Frame) if `frame` is that Frame, else in
    // added_observations.
    void AddObservation(const TKeyFrame& frame, uint32_t feature, map::MapPointId point,
                        events::MapTransaction<TFeatureExtractor>& out_event);
    // Adds a point at `position` observed by `keyframe`'s `feature`: that
    // feature's descriptor, the mean viewing direction from `centres` as its
    // normal.
    map::MapPointId AddNewPoint(const geometry::TPoint3DF& position, const TKeyFrame& keyframe,
                                uint32_t feature, const std::vector<geometry::TPoint3DF>& centres,
                                events::MapTransaction<TFeatureExtractor>& out_event);
    // Starts the bookkeeping of a point just added to map_points_: visible =
    // found = 1, no reference yet.
    void OnPointAdded(map::MapPointId point);
    // Makes `frame`'s observation of `point` through `feature` its reference:
    // that feature's level and the point's current distance to the Frame's
    // centre, as the replica computes it.
    void SetReference(map::MapPointId point, const TKeyFrame& frame, uint32_t feature);
    // Removes `point` with all its observations and lists it in `out_event`'s
    // removed points.
    void RemovePoint(map::MapPointId point, events::MapTransaction<TFeatureExtractor>& out_event);
    // Binocular `keyframe`: a point per untracked stereo-matched feature
    // closer than the close depth, and at least for the min_close_points
    // closest, at its stereo back-projection. Marks their features in
    // `out_has_point`.
    void AddClosePoints(const TKeyFrame& keyframe, std::vector<bool>& out_has_point,
                        events::MapTransaction<TFeatureExtractor>& out_event);
    // Triangulates `keyframe`'s features without a point (`has_point` false)
    // against its covisible neighbour Frames, skipping a neighbour with too
    // small a baseline. The neighbour's observation of each new point goes
    // into `out_event`'s added_observations.
    void TriangulateWithNeighbours(const TKeyFrame& keyframe, std::vector<bool> has_point,
                                   events::MapTransaction<TFeatureExtractor>& out_event);

    // Map quality (mapper_quality.cpp).

    // Step 1 (spec section 4.1), before the Frame with id next_frame_id_ is
    // created: judges every recent point, removes the poor ones into
    // `out_event` and drops the graduated ones from the list.
    void CullRecentPoints(events::MapTransaction<TFeatureExtractor>& out_event);
    // Step 3 (spec section 3): the forward pass projects `keyframe`'s points
    // into each fusion target, the backward pass the targets' other points
    // into `keyframe`; matches add observations or merge duplicates.
    void Fuse(const TKeyFrame& keyframe, events::MapTransaction<TFeatureExtractor>& out_event);
    // `keyframe`'s top covisible Frames, then each one's top covisible
    // Frames: unique, without `keyframe`.
    [[nodiscard]] std::vector<map::FrameId> FusionTargets(const TKeyFrame& keyframe) const;
    // Matches `points` (unique, alive) in `frame` with FuseMatches and
    // applies the decisions.
    void FuseInto(const TKeyFrame& frame, const std::vector<map::MapPointId>& points,
                  events::MapTransaction<TFeatureExtractor>& out_event);
    // Merges duplicates `a` and `b`: the one with more weighted observations
    // (the lower id on a tie) wins and takes the loser's observations and
    // counters, except in Frames that already observe it; the loser is
    // removed. Returns the winner.
    map::MapPointId Merge(map::MapPointId a, map::MapPointId b,
                          events::MapTransaction<TFeatureExtractor>& out_event);
    // `point`, or the point it was finally merged into in this transaction.
    [[nodiscard]] map::MapPointId Resolve(map::MapPointId point) const;
    // The (left) feature through which `frame` observes `point`; nullopt if
    // it does not.
    [[nodiscard]] std::optional<uint32_t> FeatureOf(map::FrameId frame, map::MapPointId point) const;
    // Per (left) feature of `frame`: the point it observes.
    [[nodiscard]] std::vector<std::optional<map::MapPointId>> FeaturePoints(
        const TKeyFrame& frame) const;
    // Rewrites the observation records of `out_event` to their final points
    // (merges resolved, stale ones dropped), lists the merges with their final
    // winners, recomputes the touched points' statistics and writes the new
    // and updated points. `first_new` is the first new point id.
    void FinishTransaction(map::MapPointId first_new,
                           events::MapTransaction<TFeatureExtractor>& out_event);

    // `frame`'s side of a triangulation, with `has_point` per feature.
    [[nodiscard]] TriangulationView<TFeatureExtractor> ViewOf(const TKeyFrame& frame,
                                                              std::vector<bool> has_point) const;
    // Per (left) feature of `frame`: whether it observes a map point.
    [[nodiscard]] std::vector<bool> FeaturesWithPoints(const TKeyFrame& frame) const;
    // The median camera-frame depth of `frame`'s alive points; nullopt if it
    // observes none.
    [[nodiscard]] std::optional<double> SceneMedianDepth(const TKeyFrame& frame) const;
    // Recomputes the normal and representative descriptor of `point` from
    // all its observations.
    void UpdatePointStatistics(map::MapPointId point);

    // Stores the initial map points and their observations: in both keyframes
    // (monocular) or in the one stereo keyframe. The first keyframe's
    // observation is each point's reference.
    void AddInitialMapPoints(const std::vector<InitialMapPoint>& map_points);

    // Publishes the fresh map as one transaction: every Frame with its
    // observations, every point; tracking starts from the last Frame.
    void PublishNewMap();
    // Appends `keyframe`, with its pose and measurement, to `out_event`'s
    // added Frames; returns that part.
    events::NewFrame<TFeatureExtractor>& AddFramePart(
        const TKeyFrame& keyframe, events::MapTransaction<TFeatureExtractor>& out_event) const;

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
    // The current map's points; ids restart at 0 per map.
    map::MapPointStorage<TFeatureExtractor> map_points_;
    std::atomic<size_t> num_map_points_{0};  // map_points_.size(), readable from any thread
    // The current map's observations (Frame, left feature), indexed per point
    // and per Frame. A binocular Frame's depth and right index of a feature
    // are in its measurement.
    map::CovisibilityIndex covisibility_;
    // Per point id of the current map: the reference observation (nullopt
    // once removed) and the visible / found counters.
    std::vector<std::optional<map::PointReference>> references_;
    std::vector<uint32_t> num_visible_;
    std::vector<uint32_t> num_found_;
    // Points created by the last recent_window Frames, oldest first.
    std::vector<RecentPoint> recent_points_;
    // Transaction 1 in progress: the (Frame, point) keys of the observations
    // it added; loser -> winner of each merge, and the losers in merge order;
    // the points whose observations changed.
    std::unordered_set<uint64_t> added_keys_;
    std::unordered_map<uint32_t, map::MapPointId> replaced_;
    std::vector<map::MapPointId> losers_;
    std::vector<map::MapPointId> touched_;
    MapInitializer<TFeatureExtractor> initializer_;

    memory::ObjectPool<events::MapTransaction<TFeatureExtractor>, memory::ThreadSafePolicy>
        transaction_pool_{256};

    CLASS_LOGGER(Mapper);

    // Last member: constructed after, and stopped before, everything it uses.
    events::WorkerThread worker_{wake_signal()};
};

} // namespace voyahslam::mapping

#endif // VOYAHSLAM_MAPPING_MAPPER_H
