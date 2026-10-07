#ifndef VOYAHSLAM_EVENTS_KEY_FRAME_CREATED_H
#define VOYAHSLAM_EVENTS_KEY_FRAME_CREATED_H

#include <cstdint>
#include <vector>
#include <voyahslam/defs.h>
#include <voyahslam/events/event_types.h>
#include <voyahslam/features/measurement.h>
#include <voyahslam/geometry/rigid_transformation.h>
#include <voyahslam/map/frame_id.h>
#include <voyahslam/map/map_point_id.h>
#include <voyahslam/memory/column_buffer.h>
#include <voyahslam/memory/recyclable.h>

namespace voyahslam::events {

// One observation that a Frame insertion adds to an existing Frame.
struct AddedObservation {
    map::FrameId frame_id;
    uint32_t feature;  // (left) feature index in that Frame's measurement
    map::MapPointId point;
    precision_t depth = -1;    // stereo Frames: the feature's depth, -1 if none
    int16_t right_index = -1;  // stereo Frames: the right feature, -1 if none
};

// Everything one Frame insertion changes in map `map_id`, as one transaction.
// A consumer applies the parts in this order: new points, updated points,
// observations on existing Frames, then the Frame itself.
template <typename TFeatureExtractor>
struct KeyFrameCreated
    : Event
    , memory::Recyclable<KeyFrameCreated<TFeatureExtractor>, memory::ThreadSafePolicy> {
    using TDescriptorScalar = typename TFeatureExtractor::TDescriptorScalarType;
    static constexpr int kDescriptorSize = TFeatureExtractor::DescriptorSize;

    // The Frame.
    map::FrameId frame_id;
    geometry::RigidTransformation pose;  // T_cw (world -> camera)
    features::TMeasurement<TFeatureExtractor> measurement;  // carries the timestamp
    // Observations as pairs: feature observed_features[i] of `measurement`
    // observes map point observed_points[i] of map `map_id`.
    std::vector<uint32_t> observed_features;
    std::vector<map::MapPointId> observed_points;

    // New points: the contiguous id range
    // [new_points_start_id, new_points_start_id + num_new_points()).
    map::MapPointId new_points_start_id{};
    memory::ColumnBuffer<precision_t, 3> new_positions;
    memory::ColumnBuffer<TDescriptorScalar, kDescriptorSize> new_descriptors;
    memory::ColumnBuffer<precision_t, 3> new_normals;

    // Observations added to existing Frames.
    std::vector<AddedObservation> added_observations;

    // Updated points: point updated_ids[i] takes column i of each buffer.
    std::vector<map::MapPointId> updated_ids;
    memory::ColumnBuffer<precision_t, 3> updated_positions;
    memory::ColumnBuffer<TDescriptorScalar, kDescriptorSize> updated_descriptors;
    memory::ColumnBuffer<precision_t, 3> updated_normals;

    // Set the number of new / updated points in every column; allocate only
    // beyond capacity.
    void ResizeNewPoints(uint32_t count) {
        new_positions.Resize(count);
        new_descriptors.Resize(count);
        new_normals.Resize(count);
    }
    [[nodiscard]] uint32_t num_new_points() const {
        return static_cast<uint32_t>(new_positions.cols());
    }
    void ResizeUpdatedPoints(uint32_t count) {
        updated_ids.resize(count);
        updated_positions.Resize(count);
        updated_descriptors.Resize(count);
        updated_normals.Resize(count);
    }

    // Called by the pool on reuse; keeps every part's capacity.
    void Reset() {
        observed_features.clear();
        observed_points.clear();
        new_points_start_id = {};
        ResizeNewPoints(0);
        added_observations.clear();
        ResizeUpdatedPoints(0);
    }

    KeyFrameCreated()
        : Event(EventType::KeyFrameCreated,
                [](Event* e) {
                    static_cast<KeyFrameCreated*>(e)->
                        memory::Recyclable<KeyFrameCreated, memory::ThreadSafePolicy>::Recycle();
                },
                [](Event* e, int n) {
                    static_cast<KeyFrameCreated*>(e)->SetRefCount(n);
                }) {}
};

} // namespace voyahslam::events

#endif // VOYAHSLAM_EVENTS_KEY_FRAME_CREATED_H
