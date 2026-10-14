#ifndef VOYAHSLAM_MAPPING_MAPPER_H
#define VOYAHSLAM_MAPPING_MAPPER_H

#include <atomic>
#include <cassert>
#include <cstddef>
#include <cstdint>
#include <map>
#include <memory>
#include <optional>
#include <set>
#include <unordered_map>
#include <unordered_set>
#include <vector>
#include <voyahslam/bow/bow_vector.h>
#include <voyahslam/bow/vocabulary.h>
#include <voyahslam/events/event_node.h>
#include <voyahslam/events/event_types.h>
#include <voyahslam/events/frame_tracked.h>
#include <voyahslam/events/global_ba_finished.h>
#include <voyahslam/events/initialization_measurement.h>
#include <voyahslam/events/loop_detected.h>
#include <voyahslam/events/map_transaction.h>
#include <voyahslam/events/tracking_lost.h>
#include <voyahslam/events/worker_thread.h>
#include <voyahslam/features/fuse_matcher.h>
#include <voyahslam/features/measurement.h>
#include <voyahslam/geometry/sim3.h>
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

namespace voyahslam::mapping {

// Loop correction (loop-closing spec sections 3, 4). Values from ORB-SLAM3's
// LoopClosing::CorrectLoop and Optimizer::OptimizeEssentialGraph.
struct LoopCorrectionOptions {
    // KeyFrame::UpdateConnections (th 15): a Frame is connected to the Frames
    // sharing at least this many points with it, or else to its most
    // covisible one. The window is the query and its connected Frames; a
    // window Frame's connected Frames before the fusion are not loop
    // connections.
    size_t min_connection_weight = 15;
    // nNumCovisibles: the loop points are those of the match Frame and of its
    // top `num_covisibles` covisible Frames (the LoopCloser's window).
    size_t num_covisibles = 10;
    // SearchAndFuse (Fuse th 4): window radius fuse_radius * level_scale^level px.
    double fuse_radius = 4.0;
    // Essential graph (minFeat 100): covisibility edges and loop connections
    // need this many shared points (a loop connection of the query with the
    // match Frame is kept regardless).
    size_t min_essential_weight = 100;
    int essential_iterations = 20;  // Levenberg-Marquardt iterations
};

// The Atlas (map-merging spec sections 1, 5). Values from ORB-SLAM3.
struct AtlasOptions {
    // Tracking: on a final loss, a map with fewer Frames is discarded
    // (ResetActiveMap), a larger one kept inactive (CreateMapInAtlas).
    size_t min_frames_to_keep = 10;
    // LoopClosing::MergeLocal numTemporalKFs: the merge window (map merging).
    size_t merge_window = 25;
};

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
    LoopCorrectionOptions loop;          // LoopDetected
    AtlasOptions atlas;                  // lost maps kept, merged
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

// Test access to the Mapper's private graph edits and Mapper-thread accessors
// (defined by the tests, libs/slam/tests/test_access.h).
struct MapperTestAccess;

enum class MapperState : uint8_t {
    Initializing,  // no map: builds one from the Tracker's InitializationMeasurements
    Mapping,       // extends the current map
};

// Owns the authoritative Atlas and is its only writer (map-merging spec
// section 1): maps by id, at most one active, the current map below. The
// others are inactive and frozen: no event changes them (a FrameTracked,
// LoopDetected or GlobalBAFinished naming one is of "another map"). Builds
// each map from the Tracker's InitializationMeasurements; then, per FrameTracked, decides
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
// BA, Frame culling). Every transaction names the current map as the Atlas's
// active_map. A final TrackingLost of the current map deactivates it
// (ORB-SLAM3 Tracking): with fewer than atlas.min_frames_to_keep Frames it is
// discarded (ResetActiveMap), published as one transaction with removed_maps
// = {the map} and no active map; otherwise it is kept inactive
// (CreateMapInAtlas), published as one transaction with no active map. The
// next new map takes the next id. A non-final TrackingLost (the Tracker is
// recently lost) is ignored. In a map with more than
// relocalization_guard_min_frames Frames, no Frame is created from a
// FrameTracked within relocalization_guard_seconds of the latest relocalized
// one (stage Relocalization), that one included.
// A LoopDetected of the current map is corrected in place of new-Frame work
// (ORB-SLAM3 LoopClosing::CorrectLoop, loop-closing spec section 3): the
// query's window is moved by the loop similarity with its points, the loop
// points are fused into it, the essential graph is optimized, and the query
// and match Frames get a loop edge; all of it is one MapTransaction at the
// next correction epoch. A LoopDetected of another map, naming a culled Frame
// or a loop already closed is ignored. A LoopDetected across two maps is a
// merge (OnMergeDetected, map-merging spec section 3): the active map is
// absorbed by the inactive match map, which becomes active, as one
// MapTransaction. A FrameTracked of an older epoch (the
// Tracker had not seen the correction or the global-BA merge) creates no Frame; its points are
// counted. A GlobalBAFinished of the current map at the current epoch is
// merged (ORB-SLAM3's post-GBA block in LoopClosing::RunGlobalBundleAdjustment,
// global-BA spec section 3): the optimized Frames and points take their
// results, the rest follow through the spanning tree, all of it one
// MapTransaction at the next correction epoch without loop edges; one of
// another map or epoch is ignored. Runs on its own thread; each
// ProcessPending() pass drains the inbox.
//
// The map graph (loop-closing spec section 1), published in the
// transactions: each Frame but the map's first has a parent in a spanning
// tree, the Frame sharing the most points with it once transaction 1's
// fusion is done (ORB-SLAM3 UpdateConnections). A culled Frame's children are
// re-parented as in ORB-SLAM3's KeyFrame::SetBadFlag. Frames with loop edges
// are never culled. Every transaction carries the map's correction epoch,
// 0 for a new map.
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
    using TSharedMeasurement = features::TSharedMeasurement<TFeatureExtractor>;
    using TKeyFrame = map::Frame<TFeatureExtractor, TMeasurement>;
    using TVocabulary = bow::Vocabulary<TFeatureExtractor>;

    // FeatureVector nodes are this many levels above the words (ORB-SLAM3).
    static constexpr int kBowLevelsUp = 4;

    // `vocabulary` may be null: no bags of words.
    explicit Mapper(MapperOptions options = {},
                    std::shared_ptr<const TVocabulary> vocabulary = nullptr);
    // Stops the worker before any member it uses is destroyed.
    ~Mapper();

    // Lifecycle (D-10..D-12): Created -> Running -> Stopped, Stopped is terminal.
    // Start() twice or after Stop() throws UsageError; Stop() before Start() moves
    // to Stopped; Stop() is idempotent and every concurrent call returns only after
    // the worker is joined. Both are safe from any thread, except that Stop() on
    // the component's own worker thread throws UsageError (it would join itself).
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
    // FrameTracked events evaluated by the Frame creation rule (not skipped),
    // each counted once the Frame it created, if any, is published and refined.
    [[nodiscard]] size_t num_frame_evaluations() const {
        return num_frame_evaluations_.load(std::memory_order_acquire);
    }
    // FrameTracked events of the current map skipped, in total, because a
    // newer FrameTracked (or a final TrackingLost) of the same map was in the same
    // drained batch.
    [[nodiscard]] size_t backlog_skipped() const {
        return backlog_skipped_.load(std::memory_order_acquire);
    }
    // MapTransactions built (CreateTransaction), in total; every one is
    // published. Any thread.
    [[nodiscard]] uint64_t num_transactions_built() const {
        return num_transactions_built_.load(std::memory_order_acquire);
    }
    // FrameTracked events of the current map skipped, in total, because they
    // were tracked before a loop correction (an older map epoch).
    [[nodiscard]] size_t stale_epoch_skipped() const {
        return stale_epoch_skipped_.load(std::memory_order_acquire);
    }
    // InitializationMeasurement events handled (tried, or ignored while a map
    // is active), in total; each counted after the map it created, if any, is
    // published.
    [[nodiscard]] size_t num_initialization_measurements() const {
        return num_initialization_measurements_.load(std::memory_order_acquire);
    }
    // FrameTracked events skipped, in total, because their map was not the
    // active one: tracked in a map that a merge absorbed, or that was lost,
    // while the event was in flight.
    [[nodiscard]] size_t other_map_skipped() const {
        return other_map_skipped_.load(std::memory_order_acquire);
    }
    // GlobalBAFinished events merged, and ignored because they were of
    // another map or another correction epoch, or came with no map.
    [[nodiscard]] size_t num_global_ba_merges() const {
        return num_global_ba_merges_.load(std::memory_order_acquire);
    }
    [[nodiscard]] size_t num_global_ba_rejected() const {
        return num_global_ba_rejected_.load(std::memory_order_acquire);
    }
    // Merges applied (map merging).
    [[nodiscard]] size_t num_merges() const { return num_merges_.load(std::memory_order_acquire); }

private:
    friend struct MapperTestAccess;

    // The current map's observations. Mapper-thread state; tests read it through
    // MapperTestAccess; apps use mapping::MapFollower.
    [[nodiscard]] const map::CovisibilityIndex& covisibility() const { return ActiveOrEmpty().covisibility; }
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
    // The current map's Frame `frame`: its parent in the spanning tree and its
    // loop partners; none / empty if it is not in the map. Same thread rule.
    [[nodiscard]] std::optional<map::FrameId> Parent(map::FrameId frame) const;
    [[nodiscard]] std::set<map::FrameId> LoopEdges(map::FrameId frame) const;
    // The current map's Frame `frame`; nullptr if it is not in the map. Same
    // thread rule.
    [[nodiscard]] const TKeyFrame* KeyFrame(map::FrameId frame) const { return FindKeyFrame(frame); }
    // The Atlas: the live maps, ascending; the active one, none while
    // Initializing; map `id`'s Frames (culled ones excluded), 0 if it is not
    // live. Same thread rule.
    [[nodiscard]] std::vector<map::MapId> map_ids() const;
    [[nodiscard]] std::optional<map::MapId> active_map() const { return active_; }
    [[nodiscard]] size_t num_keyframes(map::MapId id) const;

    // A Frame of the current map with its bag of words (empty without a
    // vocabulary) and its links in the map graph.
    struct KeyFrameRecord {
        TKeyFrame frame;
        bow::BowVector bow;
        bow::FeatureVector features;
        std::optional<map::FrameId> parent;  // none for the map's first Frame
        std::set<map::FrameId> children;
        std::set<map::FrameId> loop_edges;
    };

    // A point created with Frame `created`, culled or graduated by
    // CullRecentPoints (spec section 4.1).
    struct RecentPoint {
        map::MapPointId id;
        map::FrameId created;
        bool stereo;  // created by a binocular Frame
    };

    // One map of the Atlas: its Frames, points and graph. Point ids restart at
    // 0 per map.
    struct MapState {
        uint32_t map_epoch = 0;  // the correction epoch
        // Frame slots by id from `first_slot` (the lowest Frame id the map
        // ever held); a culled Frame, or an id a merge did not bring, is
        // empty. Map merging interleaves two maps' ids, so the first slot
        // need not be the root. The root (the spanning tree's, the map's
        // first Frame or the merge target's root) and the last slot (the
        // newest Frame) are never culled.
        std::vector<std::optional<KeyFrameRecord>> keyframes;
        map::FrameId first_slot{};
        map::FrameId root{};
        map::MapPointStorage<TFeatureExtractor> map_points;
        // Observations (Frame, left feature), indexed per point and per Frame.
        // A binocular Frame's depth and right index of a feature are in its
        // measurement.
        map::CovisibilityIndex covisibility;
        // Per point id: the reference observation (nullopt once removed) and
        // the visible / found counters.
        std::vector<std::optional<map::PointReference>> references;
        std::vector<uint32_t> num_visible;
        std::vector<uint32_t> num_found;
        // Points created by the last recent_window Frames, oldest first.
        std::vector<RecentPoint> recent_points;
        // Timestamp of the map's latest FrameTracked with stage
        // Relocalization, evaluated or skipped.
        std::optional<double> relocalized_at;
    };

    // The active map; there must be one (state() == Mapping).
    [[nodiscard]] MapState& Active() {
        assert(active_state_ != nullptr);
        return *active_state_;
    }
    [[nodiscard]] const MapState& Active() const {
        assert(active_state_ != nullptr);
        return *active_state_;
    }
    [[nodiscard]] bool IsActive(map::MapId id) const { return active_ == id; }
    // The active map, or an empty one without: for the public accessors.
    [[nodiscard]] const MapState& ActiveOrEmpty() const {
        return active_state_ != nullptr ? *active_state_ : kEmptyMap;
    }
    inline static const MapState kEmptyMap;

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
    // A LoopDetected within one map (ApplyEvent sends one across two maps to
    // OnMergeDetected).
    void OnLoopDetected(const events::LoopDetected& event);
    // Ignores a result of another map, of an older correction epoch, or
    // without a map; merges the others (MergeGlobalBa).
    void OnGlobalBaFinished(const events::GlobalBAFinished& event);
    void SetState(MapperState state);
    // Creates map `id`, empty, and makes it the active one.
    void ActivateNewMap(map::MapId id);
    // Leaves no map active (the active one stays in maps_ unless erased).
    void Deactivate();

    // Appends Frame `id` to the current map, with its bag of words when there
    // is a vocabulary; returns it.
    const TKeyFrame& AddKeyFrame(map::FrameId id, const geometry::RigidTransformation& pose,
                                 TSharedMeasurement measurement);
    // Frame `id` of map `state` with its bag of words; nullptr if it is not in
    // that map (or was culled).
    [[nodiscard]] static const KeyFrameRecord* FindRecordIn(const MapState& state, map::FrameId id);
    // The current map's Frame `id` with its bag of words; nullptr if it is not
    // in the map (or was culled).
    [[nodiscard]] const KeyFrameRecord* FindRecord(map::FrameId id) const;
    [[nodiscard]] KeyFrameRecord* FindRecord(map::FrameId id);
    // The current map's Frame `id`; nullptr if it is not in the map (or was
    // culled).
    [[nodiscard]] const TKeyFrame* FindKeyFrame(map::FrameId id) const;
    [[nodiscard]] TKeyFrame* FindKeyFrame(map::FrameId id);
    // The FeatureVector of the current map's Frame `id` for SearchByBoW;
    // nullptr without a vocabulary or if the Frame is not in the map.
    [[nodiscard]] const bow::FeatureVector* BowFeaturesOf(map::FrameId id) const;
    // The current map's root Frame (its first, or after a merge the target's
    // root), never culled: the gauge of local BA.
    [[nodiscard]] const TKeyFrame& RootKeyFrame() const { return FindRecord(Active().root)->frame; }
    // The current map's newest Frame, never culled.
    [[nodiscard]] const TKeyFrame& LastKeyFrame() const { return Active().keyframes.back()->frame; }
    // `point`'s observations as ORB-SLAM3 counts them for ref_points: 2 for a
    // stereo-matched one (right index >= 0), 1 otherwise. Unlike
    // CovisibilityIndex::NumObservations, which counts Frames.
    [[nodiscard]] size_t WeightedObservations(map::MapPointId point) const;
    // Clears the scratch state of the transaction being built (added
    // observation keys, merges, touched points): the start of transaction 1
    // and of a loop correction.
    void ResetTransactionScratch();
    // A new transaction of the current map at the current epoch; fill it, then
    // publish it with observable_.Notify(std::move(...)) and do not touch it again.
    [[nodiscard]] std::shared_ptr<events::MapTransaction<TFeatureExtractor>> CreateTransaction();
    // Gives Frame `frame`, the newest one, its parent: the Frame sharing the
    // most points with it, the lower id on a tie; without shared points, the
    // previous newest Frame (ORB-SLAM3 leaves it without one). Returns it.
    map::FrameId ConnectToParent(map::FrameId frame);
    // Makes `parent` the parent of `frame` and lists the change in
    // `out_event`.
    void ChangeParent(map::FrameId frame, map::FrameId parent,
                      events::MapTransaction<TFeatureExtractor>& out_event);
    // Adds the loop edge between Frames `a` and `b` and lists it in
    // `out_event`.
    void AddLoopEdge(map::FrameId a, map::FrameId b,
                     events::MapTransaction<TFeatureExtractor>& out_event);
    // Before Frame `frame` is culled, its observations already gone: each
    // child goes to the candidate sharing the most points with it, the
    // candidates being `frame`'s parent and the children re-parented so far;
    // a child sharing none goes to `frame`'s parent (ORB-SLAM3
    // KeyFrame::SetBadFlag).
    void ReparentChildren(map::FrameId frame, events::MapTransaction<TFeatureExtractor>& out_event);
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
    // applies the decisions: duplicates are merged. With `loop` (ORB-SLAM3's
    // Fuse with a Sim3), the window is fuse_radius wide, there is no chi2 nor
    // right-image gate, and the projected point replaces the one it matches.
    void FuseInto(const TKeyFrame& frame, const std::vector<map::MapPointId>& points,
                  events::MapTransaction<TFeatureExtractor>& out_event, bool loop = false);
    // Merges duplicates `a` and `b`: the one with more weighted observations
    // (the lower id on a tie) wins and takes the loser's observations and
    // counters, except in Frames that already observe it; the loser is
    // removed. Returns the winner.
    map::MapPointId Merge(map::MapPointId a, map::MapPointId b,
                          events::MapTransaction<TFeatureExtractor>& out_event);
    // `winner` takes `loser`'s observations and counters, except in Frames
    // that already observe it; `loser` is removed (ORB-SLAM3
    // MapPoint::Replace).
    void Replace(map::MapPointId loser, map::MapPointId winner,
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
    // The first part of FinishTransaction: the observation records (of the
    // new Frame, if any, and the added ones) and the merges.
    void ResolveObservationRecords(events::MapTransaction<TFeatureExtractor>& out_event);
    // Writes `points` to `out_event`'s updated points.
    void WriteUpdatedPoints(const std::vector<map::MapPointId>& points,
                            events::MapTransaction<TFeatureExtractor>& out_event) const;

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
    // Frame nor one with a loop edge. The points that lost an observation are
    // appended to `out_changed`.
    void CullFrames(map::FrameId frame, events::MapTransaction<TFeatureExtractor>& out_event,
                    std::vector<map::MapPointId>& out_changed);
    // Drops the Frame and its observations, re-parenting its children; a
    // point left with fewer than 2 observations is removed, the others are
    // appended to `out_changed`.
    void RemoveFrame(map::FrameId frame, events::MapTransaction<TFeatureExtractor>& out_event,
                     std::vector<map::MapPointId>& out_changed);
    // After transaction 2's removals: each changed point still alive whose
    // reference observation is gone takes the remaining observation with the
    // smallest Frame id (current pose and position), as the replica does; its
    // statistics are recomputed and it is written to `out_event`'s updated
    // points.
    void FinishRefinement(std::vector<map::MapPointId> changed,
                          events::MapTransaction<TFeatureExtractor>& out_event);

    // Loop correction (mapper_loop.cpp; loop-closing spec section 3). Steps
    // 2-6 take a window and a similarity, so map merging can reuse them.

    // A window moved by CorrectWindow, per Frame (the query first): its
    // corrected S_cw, its pose before the correction and its connected Frames
    // before the loop fusion; per moved point, the window Frame whose
    // correction moved it (ORB-SLAM3 mnCorrectedReference).
    struct WindowCorrection {
        std::vector<map::FrameId> frames;
        std::vector<geometry::Sim3> corrected;
        std::vector<geometry::RigidTransformation> non_corrected;
        std::vector<std::vector<map::FrameId>> previous_connected;
        std::unordered_map<uint32_t, map::FrameId> corrected_reference;
    };

    // The Frames `frame` is connected to (ORB-SLAM3 UpdateConnections): those
    // sharing at least min_connection_weight points, by shared points, or else
    // its most covisible one.
    [[nodiscard]] std::vector<map::FrameId> Connected(map::FrameId frame) const { return Connected(Active(), frame); }
    // The same in map `state`.
    [[nodiscard]] std::vector<map::FrameId> Connected(const MapState& state, map::FrameId frame) const;
    // `frame` and its top num_covisibles covisible Frames (the LoopCloser's
    // window of a match).
    [[nodiscard]] std::vector<map::FrameId> MatchWindow(map::FrameId frame) const;
    // Review Focus 2: the loop of `query` with `match` is already closed:
    // a Frame of the match's window is covisible with the query (ORB-SLAM3
    // never detects such a candidate), or a loop edge links the query's window
    // to the match's.
    [[nodiscard]] bool IsClosedLoop(map::FrameId query, map::FrameId match) const;
    // Steps 2-3 (CorrectLoop), the start of every correction: resets the
    // transaction scratch state (ResetTransactionScratch) that FuseLoopPoints
    // and FinishCorrection build on, so the steps must run in order on one
    // transaction. `window`'s first Frame takes S_query_world,
    // every other one keeps its pose relative to it; the poses take the
    // corrected similarities with the scale folded into the translation, and
    // each point the window observes moves with its first observing window
    // Frame, X' = S_corrected^-1 S_old X. The caller publishes the poses and
    // points.
    WindowCorrection CorrectWindow(const std::vector<map::FrameId>& window,
                                   const geometry::Sim3& S_query_world);
    // Step 4: each (query point, match point) pair of `matched_points` that the
    // query still observes is replaced by its match point, then `loop_points`
    // are fused into every window Frame at its corrected pose (ORB-SLAM3
    // SearchAndFuse): a free feature observes the loop point, an observed one
    // has its point replaced by the loop point.
    void FuseLoopPoints(const WindowCorrection& correction,
                        const std::vector<std::pair<map::MapPointId, map::MapPointId>>& matched_points,
                        const std::vector<map::MapPointId>& loop_points,
                        events::MapTransaction<TFeatureExtractor>& out_event);
    // Step 5 (CorrectLoop LoopConnections): per window Frame, its covisible
    // Frames now, at any weight (ORB-SLAM3 GetConnectedKeyFrames), minus its
    // connected Frames before the fusion and the window; a pair with fewer than
    // min_essential_weight shared points is dropped, but the query's with
    // `match`. As (window Frame, other Frame) pairs.
    [[nodiscard]] std::vector<std::pair<map::FrameId, map::FrameId>> LoopConnections(
        const WindowCorrection& correction, map::FrameId match) const;
    // Steps 5-6: each window Frame's new connections beyond the window are the
    // loop connections; the essential graph (OptimizeEssentialGraph) of every
    // Frame, `match` fixed and every scale fixed if `fix_scale`, gives every
    // Frame its pose and every point moves with its reference Frame (its
    // window Frame if CorrectWindow moved it). Every pose goes into
    // `out_event`.
    void OptimizeGraph(const WindowCorrection& correction, map::FrameId match, bool fix_scale,
                       events::MapTransaction<TFeatureExtractor>& out_event);
    // Step 8: the observation records resolved, the merges listed, every
    // alive point's normal (and the touched points' descriptors) recomputed
    // and written to `out_event`.
    void FinishCorrection(events::MapTransaction<TFeatureExtractor>& out_event);

    // Global BA merge (mapper_global_ba.cpp; global-BA spec section 3).

    // ORB-SLAM3's post-GBA block: walking the spanning tree from its root,
    // parents first, a Frame in `frame_poses` (T_cw) takes its optimized
    // pose, another with a corrected parent keeps its pose relative to it,
    // T_new = T_cur T_parent,cur^-1 T_parent,new. An alive point in
    // `point_positions` takes its optimized position; another moves with its
    // reference Frame, if that was corrected, X_new = T_ref,new^-1 T_ref,cur X.
    // Culled Frames and removed points are skipped. Published as one
    // MapTransaction at the next correction epoch, with no loop edge. Map
    // merging (step 5) reuses it.
    void MergeGlobalBa(const std::vector<std::pair<map::FrameId, geometry::RigidTransformation>>& frame_poses,
                       const std::vector<std::pair<map::MapPointId, geometry::TPoint3DF>>& point_positions);

    // Map merging (mapper_merge.cpp; map-merging spec section 3, ORB-SLAM3
    // LoopClosing::MergeLocal, visual). The absorbed map is moved into the
    // target first (step 7's id rule, as every replica applies it), so the
    // loop-correction steps then run on one map, the target made active.

    // A LoopDetected of the active map (the query's) with an inactive one: the
    // active map is merged into the inactive one, which survives and becomes
    // active, as one MapTransaction (steps 1-8). Rejected (ignored) when the
    // query map is not the active one, the match map is not an existing
    // inactive map, the query or match Frame is no longer in its map, or the
    // similarity is invalid.
    void OnMergeDetected(const events::LoopDetected& event);
    // Step 7, done first: moves every Frame (records, graph links), alive point
    // (position, descriptor, normal, reference, counters) and observation of
    // `source` into `target`, point old id p taking AbsorbedPointIds' new id,
    // with the Mapper's bookkeeping (recent points remapped, the relocalization
    // guard); `source` is left moved-from, for the caller to erase. Returns
    // the new id per old id.
    static std::vector<std::optional<map::MapPointId>> AbsorbMap(MapState& source, MapState& target);
    // Step 1 (ORB-SLAM3 MergeLocal's windows), in map `state`: `frame`, its connected Frames
    // up to atlas.merge_window (GetBestCovisibilityKeyFrames), then, while
    // the window is smaller and for at most 5 rounds, each window Frame's
    // connected Frames up to merge_window / 2. `frame` first.
    [[nodiscard]] std::vector<map::FrameId> MergeWindowOf(const MapState& state, map::FrameId frame) const;
    // Step 3: `query` becomes a child of `match` and the parent chain from the
    // query to its old root is inverted (ORB-SLAM3's ChangeParent walk).
    void ReRootTree(map::FrameId query, map::FrameId match, events::MapTransaction<TFeatureExtractor>& out_event);
    // Step 5, the weld (ORB-SLAM3 LocalBundleAdjustment(current, current
    // window, merge window)): BundleAdjust of the `free` and `fixed` Frames
    // with every point they observe, through their observations only. Poses
    // and points are applied; outlier observations are removed once the robust
    // stage has finished, and their points appended to `out_changed`.
    void WeldWindows(const std::vector<map::FrameId>& free, const std::vector<map::FrameId>& fixed,
                     events::MapTransaction<TFeatureExtractor>& out_event, std::vector<map::MapPointId>& out_changed);
    // Step 6: each Frame of `absorbed_frames` outside the window takes the
    // merge similarity as CorrectWindow gives it, its points moving with their
    // reference Frame; then the essential graph of the absorbed Frames (the
    // window fixed at its welded poses; ORB-SLAM3's merge overload of
    // OptimizeEssentialGraph) moves them and their points again.
    void MoveRestOfMap(const WindowCorrection& correction, const std::vector<map::FrameId>& absorbed_frames,
                       bool fix_scale);

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
    std::vector<events::EventPtr> batch_;
    std::vector<bool> superseded_;
    size_t backlog_ = 0;  // FrameTracked skipped since the last evaluated one
    std::atomic<size_t> num_frame_evaluations_{0};
    std::atomic<size_t> backlog_skipped_{0};
    std::atomic<size_t> stale_epoch_skipped_{0};
    std::atomic<size_t> other_map_skipped_{0};
    std::atomic<size_t> num_initialization_measurements_{0};
    std::atomic<size_t> num_global_ba_merges_{0};
    std::atomic<size_t> num_global_ba_rejected_{0};
    std::atomic<size_t> num_merges_{0};

    std::atomic<MapperState> state_{MapperState::Initializing};  // Mapping iff a map is active
    // The map last made active (0 before the first): the active one while
    // Mapping (a merge's target after a merge); after a final loss, the lost
    // map's until the next new map.
    std::atomic<map::MapId> current_map_id_{map::MapId{0}};
    map::MapId next_map_id_{1};      // never reused
    map::FrameId next_frame_id_{0};  // never reset: frame ids are unique across maps
    // The Atlas: every live map by id, the inactive ones frozen; the active
    // one, if any, and its state (cached: hot paths reach it per point;
    // std::map nodes are stable).
    std::map<map::MapId, MapState> maps_;
    std::optional<map::MapId> active_;
    MapState* active_state_ = nullptr;
    // The active map's Frames (culled ones excluded) and points, 0 without
    // one; readable from any thread.
    std::atomic<size_t> num_keyframes_{0};
    std::atomic<size_t> num_map_points_{0};
    std::atomic<uint64_t> num_transactions_built_{0};
    // Transaction 1 in progress: the (Frame, point) keys of the observations
    // it added; loser -> winner of each merge, and the losers in merge order;
    // the points whose observations changed.
    std::unordered_set<uint64_t> added_keys_;
    std::unordered_map<uint32_t, map::MapPointId> replaced_;
    std::vector<map::MapPointId> losers_;
    std::vector<map::MapPointId> touched_;
    LocalBaSummary last_local_ba_;
    MapInitializer<TFeatureExtractor> initializer_;

    CLASS_LOGGER(Mapper);

    // Last member: constructed after, and stopped before, everything it uses.
    events::WorkerThread worker_{wake_signal()};
};

} // namespace voyahslam::mapping

#endif // VOYAHSLAM_MAPPING_MAPPER_H
