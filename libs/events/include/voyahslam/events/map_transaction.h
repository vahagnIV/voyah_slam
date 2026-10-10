#ifndef VOYAHSLAM_EVENTS_MAP_TRANSACTION_H
#define VOYAHSLAM_EVENTS_MAP_TRANSACTION_H

#include <cstdint>
#include <optional>
#include <utility>
#include <vector>
#include <voyahslam/bow/bow_vector.h>
#include <voyahslam/defs.h>
#include <voyahslam/events/event_types.h>
#include <voyahslam/features/measurement.h>
#include <voyahslam/geometry/rigid_transformation.h>
#include <voyahslam/map/frame_id.h>
#include <voyahslam/map/map_point_id.h>
#include <voyahslam/memory/column_buffer.h>
#include <voyahslam/memory/recyclable.h>

namespace voyahslam::events {

// One observation a transaction adds: (left) feature `feature` of Frame
// `frame` observes point `point`.
struct TransactionObservation {
    map::FrameId frame;
    uint32_t feature;  // (left) feature index in that Frame's measurement
    map::MapPointId point;
    precision_t depth = -1;    // stereo Frames: the feature's depth, -1 if none
    int16_t right_index = -1;  // stereo Frames: the right feature, -1 if none
};

// One observation a transaction removes.
struct RemovedObservation {
    map::FrameId frame;
    map::MapPointId point;
};

// A point merged into another: consumers holding `loser` switch to `winner`.
struct ReplacedPoint {
    map::MapPointId loser;
    map::MapPointId winner;
};

// A Frame a transaction adds, with its own observations as pairs: feature
// observed_features[i] of `measurement` observes point observed_points[i].
template <typename TFeatureExtractor>
struct NewFrame {
    map::FrameId id;
    geometry::RigidTransformation T_cw;  // world -> camera
    features::TMeasurement<TFeatureExtractor> measurement;  // carries the timestamp
    std::vector<uint32_t> observed_features;
    std::vector<map::MapPointId> observed_points;
    // The BoW data of the measurement's (left) descriptors: filled by a Mapper
    // that has a vocabulary, empty otherwise.
    bow::BowVector bow;
    bow::FeatureVector features;
    // Its parent in the map's spanning tree: the Frame sharing the most points
    // with it at creation; none for the map's first Frame.
    std::optional<map::FrameId> parent;
};

// Everything one Mapper step changes in map `map_id`: the only event the
// Mapper publishes. Every part may be empty. A consumer applies the parts in
// this order (spec section 2): new map, add Frames (with their parents), add
// loop edges, add points, add observations (the new Frames' own, then
// added_observations), update points, update poses, remove observations,
// remove points (removed_points and the losers of replaced_points), change
// parents, remove Frames (with their observations and loop edges). Parents
// and loop edges therefore only name Frames that exist when they are applied:
// a culled Frame's children are re-parented before it goes.
template <typename TFeatureExtractor>
struct MapTransaction
    : Event
    , memory::Recyclable<MapTransaction<TFeatureExtractor>, memory::ThreadSafePolicy> {
    using TDescriptorScalar = typename TFeatureExtractor::TDescriptorScalarType;
    static constexpr int kDescriptorSize = TFeatureExtractor::DescriptorSize;

    // A new map: consumers drop what they hold of older maps; the Tracker
    // starts tracking from Frame `start_frame`.
    bool new_map = false;
    std::optional<map::FrameId> start_frame;

    std::vector<NewFrame<TFeatureExtractor>> added_frames;

    // New points: the contiguous id range
    // [new_points_start_id, new_points_start_id + num_new_points()).
    map::MapPointId new_points_start_id{};
    memory::ColumnBuffer<precision_t, 3> new_positions;
    memory::ColumnBuffer<precision_t, 3> new_normals;
    memory::ColumnBuffer<TDescriptorScalar, kDescriptorSize> new_descriptors;

    // Observations added to any Frame, in addition to the new Frames' own.
    std::vector<TransactionObservation> added_observations;

    // Updated points: point updated_ids[i] takes column i of each buffer.
    std::vector<map::MapPointId> updated_ids;
    memory::ColumnBuffer<precision_t, 3> updated_positions;
    memory::ColumnBuffer<precision_t, 3> updated_normals;
    memory::ColumnBuffer<TDescriptorScalar, kDescriptorSize> updated_descriptors;

    std::vector<RemovedObservation> removed_observations;
    std::vector<ReplacedPoint> replaced_points;
    std::vector<map::MapPointId> removed_points;
    std::vector<std::pair<map::FrameId, geometry::RigidTransformation>> updated_poses;  // T_cw
    std::vector<map::FrameId> removed_frames;
    // The map graph: (frame, new parent) pairs, applied in order, and (frame,
    // frame) loop edges (undirected).
    std::vector<std::pair<map::FrameId, map::FrameId>> parent_changes;
    std::vector<std::pair<map::FrameId, map::FrameId>> added_loop_edges;
    // The map's correction epoch: 0 for a new map, incremented by every loop
    // correction.
    uint32_t map_epoch = 0;

    // Set the number of new / updated points in every column; allocate only
    // beyond capacity.
    void ResizeNewPoints(uint32_t count) {
        new_positions.Resize(count);
        new_normals.Resize(count);
        new_descriptors.Resize(count);
    }
    [[nodiscard]] uint32_t num_new_points() const {
        return static_cast<uint32_t>(new_positions.cols());
    }
    void ResizeUpdatedPoints(uint32_t count) {
        updated_ids.resize(count);
        updated_positions.Resize(count);
        updated_normals.Resize(count);
        updated_descriptors.Resize(count);
    }

    // Called by the pool on reuse; clears every part and keeps its capacity.
    void Reset() {
        new_map = false;
        start_frame.reset();
        added_frames.clear();
        new_points_start_id = {};
        ResizeNewPoints(0);
        added_observations.clear();
        ResizeUpdatedPoints(0);
        removed_observations.clear();
        replaced_points.clear();
        removed_points.clear();
        updated_poses.clear();
        removed_frames.clear();
        parent_changes.clear();
        added_loop_edges.clear();
        map_epoch = 0;
    }

    MapTransaction()
        : Event(EventType::MapTransaction,
                [](Event* e) {
                    static_cast<MapTransaction*>(e)->
                        memory::Recyclable<MapTransaction, memory::ThreadSafePolicy>::Recycle();
                },
                [](Event* e, int n) {
                    static_cast<MapTransaction*>(e)->SetRefCount(n);
                }) {}
};

} // namespace voyahslam::events

#endif // VOYAHSLAM_EVENTS_MAP_TRANSACTION_H
