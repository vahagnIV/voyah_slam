#ifndef VOYAHSLAM_MAPPING_MAPPER_H
#define VOYAHSLAM_MAPPING_MAPPER_H

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <unordered_map>
#include <unordered_set>
#include <vector>
#include <voyahslam/bow/bow_vector.h>
#include <voyahslam/bow/vocabulary.h>
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

// What the local BA of the last created Frame did (spec section 5).
struct LocalBaSummary {
    // False when it was skipped: no free Frame with an observation, no fixed
    // Frame, or input BundleAdjust rejected.
    bool ran = false;
    bool interrupted = false;  // stopped by an urgent event
    int stage_reached = 2;     // BaResult::stage_reached
    int iterations = 0;
    size_t num_free = 0;       // optimized Frames
    size_t num_fixed = 0;
    size_t num_points = 0;
    size_t num_observations = 0;
    size_t num_outliers = 0;   // observations removed as outliers
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
// creation the Mapper culls poor recent points once the Frame's tracked
// observations are in, then fuses the new Frame with its neighbours (spec
// sections 3, 4.1). It then runs a local bundle adjustment around the new
// Frame, which an urgent FrameTracked interrupts, and culls its redundant
// covisible Frames (spec sections 5, 4.2). Every change is published as one
// MapTransaction: a new map with all its Frames and points, or, per created
// Frame, transaction 1 (creation, culling, fusion) and transaction 2 (local
// BA, Frame culling). A final TrackingLost of the current map discards it; a
// non-final one (the Tracker is recently lost) is ignored. In a map with more
// than relocalization_guard_min_frames Frames, no Frame is created from a
// FrameTracked within relocalization_guard_seconds of the latest relocalized
// one (stage Relocalization), that one included.
// LoopDetected is not consumed yet. Runs on its own thread; each
// ProcessPending() pass drains the inbox.
//
// With a vocabulary (immutable, so it may be shared read-only), every
// Frame, the initial ones included, keeps the BowVector and FeatureVector of
// its (left) descriptors, published with the Frame in its NewFrame (the
// Tracker's relocalization database), and triangulation takes
// its candidate pairs from the FeatureVector nodes two Frames share (spec
// section 3). Without one, Frames have no bag of words and triangulation
// stays brute force.
template <typename TFeatureExtractor>
class Mapper : public events::EventNode {
public:
    using TMeasurement = features::TMeasurement<TFeatureExtractor>;
    using TKeyFrame = map::Frame<TFeatureExtractor, TMeasurement>;
    using TVocabulary = bow::Vocabulary<TFeatureExtractor>;

    // FeatureVector nodes are this many levels above the words (ORB-SLAM3).
    static constexpr int kBowLevelsUp = 4;

    // `vocabulary` may be null: no bags of words.
    explicit Mapper(MapperOptions options = {},
                    std::shared_ptr<const TVocabulary> vocabulary = nullptr);
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
    // newer FrameTracked (or a final TrackingLost) of the same map was in the same
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
    // The local BA of the last created Frame. Same thread rule.
    [[nodiscard]] const LocalBaSummary& last_local_ba() const { return last_local_ba_; }
    // The BowVector of the current map's Frame `frame`; nullptr without a
    // vocabulary or if the Frame is not in the map. Same thread rule.
    [[nodiscard]] const bow::BowVector* BowVectorOf(map::FrameId frame) const;

private:
    // A Frame of the current map with its bag of words (empty without a
    // vocabulary).
    struct KeyFrameRecord {
        TKeyFrame frame;
        bow::BowVector bow;
        bow::FeatureVector features;
    };

    // A point created with Frame `created`, culled or graduated by
    // CullRecentPoints (spec section 4.1).
    struct RecentPoint {
        map::MapPointId id;
        map::FrameId created;
        bool stereo;  // created by a binocular Frame
    };

    // Marks in superseded_ each FrameTracked of batch_ that is followed in the
    // batch by a FrameTracked or a final TrackingLost of the same map.
    void MarkSupersededFrameTracked();
    void ApplyEvent(const events::Event& event);
    // A superseded FrameTracked: counted in the backlog, and its visible and
    // found points counted, if it is of the map being extended.
    void SkipFrameTracked(const events::Event& event);
    // Increments the visible and found counters of `event`'s points, and
    // notes a relocalized one for the relocalization guard.
    void CountTrackedPoints(const events::FrameTracked<TFeatureExtractor>& event);
    // True when the map has more than relocalization_guard_min_frames Frames
    // and `event` is within relocalization_guard_seconds of the latest
    // relocalized FrameTracked, that one included: it becomes no Frame
    // (ORB-SLAM3's guard after a relocalization, Ruling J).
    [[nodiscard]] bool InRelocalizationGuard(const events::FrameTracked<TFeatureExtractor>& event) const;
    void OnInitializationMeasurement(const events::InitializationMeasurement<TFeatureExtractor>& event);
    void OnFrameTracked(const events::FrameTracked<TFeatureExtractor>& event);
    void OnTrackingLost(const events::TrackingLost& event);
    void SetState(MapperState state);
    // Empties the current map (Frames, points, observations).
    void ClearMap();

    // Appends Frame `id` to the current map, with its bag of words when there
    // is a vocabulary; returns it.
    const TKeyFrame& AddKeyFrame(map::FrameId id, const geometry::RigidTransformation& pose,
                                 TMeasurement&& measurement);
    // The current map's Frame `id` with its bag of words; nullptr if it is not
    // in the map (or was culled).
    [[nodiscard]] const KeyFrameRecord* FindRecord(map::FrameId id) const;
    // The current map's Frame `id`; nullptr if it is not in the map (or was
    // culled).
    [[nodiscard]] const TKeyFrame* FindKeyFrame(map::FrameId id) const;
    [[nodiscard]] TKeyFrame* FindKeyFrame(map::FrameId id);
    // The FeatureVector of the current map's Frame `id` for SearchByBoW;
    // nullptr without a vocabulary or if the Frame is not in the map.
    [[nodiscard]] const bow::FeatureVector* BowFeaturesOf(map::FrameId id) const;
    // The current map's first Frame, never culled.
    [[nodiscard]] const TKeyFrame& FirstKeyFrame() const { return keyframes_.front()->frame; }
    // The current map's newest Frame, never culled.
    [[nodiscard]] const TKeyFrame& LastKeyFrame() const { return keyframes_.back()->frame; }
    // `point`'s observations as ORB-SLAM3 counts them for ref_points: 2 for a
    // stereo-matched one (right index >= 0), 1 otherwise. Unlike
    // CovisibilityIndex::NumObservations, which counts Frames.
    [[nodiscard]] size_t WeightedObservations(map::MapPointId point) const;
    // The inputs to ShouldCreateFrame for `event` (spec section 3.2).
    [[nodiscard]] KeyFrameSignals MeasureSignals(
        const events::FrameTracked<TFeatureExtractor>& event) const;
    // Transaction 1 of a new Frame (spec section 1, steps 1-3, in ORB-SLAM3's
    // order): inserts `event`'s measurement as a new Frame with its tracked
    // observations, culls poor recent points (the new observations counted),
    // adds new points (stereo close points, then points triangulated against
    // neighbour Frames), fuses the Frame with its neighbours, and publishes
    // all of it as one MapTransaction; then publishes transaction 2
    // (RefineAroundFrame).
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
    // small a baseline; with a vocabulary, the candidate pairs share a
    // FeatureVector node. The neighbour's observation of each new point goes
    // into `out_event`'s added_observations.
    void TriangulateWithNeighbours(const TKeyFrame& keyframe, std::vector<bool> has_point,
                                   events::MapTransaction<TFeatureExtractor>& out_event);

    // Map quality (mapper_quality.cpp).

    // Recent-point culling (spec section 4.1), once the new Frame (the last
    // one) has its tracked observations: judges every recent point, removes
    // the poor ones into `out_event` and drops the graduated ones from the
    // list.
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

    // Transaction 2 of the new Frame `frame` (spec section 1, steps 4-5):
    // local BA, then Frame culling, published as one MapTransaction even if
    // both change nothing. Returns the milliseconds of each step and the
    // number of culled Frames and removed points.
    struct Refinement {
        double ba_ms = 0.0;
        double cull_ms = 0.0;
        size_t num_culled_frames = 0;
        size_t num_removed_points = 0;
    };
    Refinement RefineAroundFrame(map::FrameId frame);
    // Local BA (spec section 5): the new Frame `frame` and its covisible
    // Frames with enough shared points are optimized with all their points,
    // the other observers of those points are fixed, as is the map's first
    // Frame. Skipped without a free Frame that has an observation or without a
    // fixed Frame. Poses and points are applied; outliers are removed only
    // once the robust stage has finished. The optimized points, and those
    // that lost an observation, are appended to `out_changed`.
    void LocalBundleAdjust(map::FrameId frame, events::MapTransaction<TFeatureExtractor>& out_event,
                           std::vector<map::MapPointId>& out_changed);
    // Frame culling (spec section 4.2): removes each covisible Frame of
    // `frame` that IsRedundantFrame judges redundant, never the map's first
    // Frame. The points that lost an observation are appended to
    // `out_changed`.
    void CullFrames(map::FrameId frame, events::MapTransaction<TFeatureExtractor>& out_event,
                    std::vector<map::MapPointId>& out_changed);
    // Drops the Frame and its observations; a point left with fewer than 2
    // observations is removed, the others are appended to `out_changed`.
    void RemoveFrame(map::FrameId frame, events::MapTransaction<TFeatureExtractor>& out_event,
                     std::vector<map::MapPointId>& out_changed);
    // After transaction 2's removals: each changed point still alive whose
    // reference observation is gone takes the remaining observation with the
    // smallest Frame id (current pose and position), as the replica does; its
    // statistics are recomputed and it is written to `out_event`'s updated
    // points.
    void FinishRefinement(std::vector<map::MapPointId> changed,
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
    // added Frames, with its BoW data if there is a vocabulary; returns that part.
    events::NewFrame<TFeatureExtractor>& AddFramePart(
        const TKeyFrame& keyframe, events::MapTransaction<TFeatureExtractor>& out_event) const;

    MapperOptions options_;
    std::shared_ptr<const TVocabulary> vocabulary_;  // may be null
    // One ProcessPending() batch: the events drained from the inbox and, per
    // event, whether it is a superseded FrameTracked.
    std::vector<events::Event*> batch_;
    std::vector<bool> superseded_;
    size_t backlog_ = 0;  // FrameTracked skipped since the last evaluated one
    // Timestamp of the current map's latest FrameTracked with stage
    // Relocalization, evaluated or skipped; none since the map was cleared.
    std::optional<double> relocalized_at_;
    std::atomic<size_t> num_frame_evaluations_{0};
    std::atomic<size_t> backlog_skipped_{0};

    std::atomic<MapperState> state_{MapperState::Initializing};
    std::atomic<map::MapId> current_map_id_{map::MapId{0}};
    map::MapId next_map_id_{1};
    map::FrameId next_frame_id_{0};  // never reset: frame ids are unique across maps
    // The current map's Frames by id from its first one; a culled Frame is
    // empty. The first and the last one are never culled.
    std::vector<std::optional<KeyFrameRecord>> keyframes_;
    // Frames in the current map (culled ones excluded), readable from any thread.
    std::atomic<size_t> num_keyframes_{0};
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
    LocalBaSummary last_local_ba_;
    MapInitializer<TFeatureExtractor> initializer_;

    memory::ObjectPool<events::MapTransaction<TFeatureExtractor>, memory::ThreadSafePolicy>
        transaction_pool_{256};

    CLASS_LOGGER(Mapper);

    // Last member: constructed after, and stopped before, everything it uses.
    events::WorkerThread worker_{wake_signal()};
};

} // namespace voyahslam::mapping

#endif // VOYAHSLAM_MAPPING_MAPPER_H
