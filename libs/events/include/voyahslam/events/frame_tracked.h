#ifndef VOYAHSLAM_EVENTS_FRAME_TRACKED_H
#define VOYAHSLAM_EVENTS_FRAME_TRACKED_H

#include <cstdint>
#include <optional>
#include <vector>
#include <voyahslam/defs.h>
#include <voyahslam/events/event_types.h>
#include <voyahslam/features/measurement.h>
#include <voyahslam/geometry/rigid_transformation.h>
#include <voyahslam/map/frame_id.h>
#include <voyahslam/map/map_point_id.h>
#include <voyahslam/memory/recyclable.h>

namespace voyahslam::events {

// The source of the initial pose that the local-map step refined into a
// FrameTracked's final pose: the last frame at the motion model's prediction
// or at zero motion, the reference Frame by bag of words, or relocalization.
enum class TrackingStage : uint8_t { MotionModel, ZeroMotion, ReferenceFrame, Relocalization };

// Published by the Tracker for every successfully tracked measurement.
template <typename TFeatureExtractor>
struct FrameTracked
    : Event
    , memory::Recyclable<FrameTracked<TFeatureExtractor>, memory::ThreadSafePolicy> {
    double timestamp = 0.0;
    geometry::RigidTransformation pose;  // T_cw (world -> camera), after pose optimization
    // Binocular: stereo-matched. Shared with the Tracker and, once it is a
    // Frame, with the map; immutable.
    features::TSharedMeasurement<TFeatureExtractor> measurement;
    // Inlier matches as pairs: (left) feature observed_features[i] of
    // `measurement` observes map point observed_points[i] of map `map_id`.
    std::vector<uint32_t> observed_features;
    std::vector<map::MapPointId> observed_points;
    // Local-map points that pass the projection gate, matched or not.
    std::vector<map::MapPointId> visible_points;
    std::optional<map::FrameId> reference_frame_id;
    std::vector<map::FrameId> local_frames;
    TrackingStage stage = TrackingStage::MotionModel;
    uint32_t num_matches = 0;  // before the final pose optimization
    uint32_t num_inliers = 0;  // after it
    // Weakly tracked (few inliers): the Mapper should answer promptly. Mirrors
    // Event::priority; the Tracker sets both on every publish.
    bool urgent = false;
    // The correction epoch of the map, as the Tracker's replica knew it.
    uint32_t map_epoch = 0;

    // Called by the pool on reuse; keeps the vectors' capacity and releases
    // the measurement.
    void Reset() {
        timestamp = 0.0;
        measurement.reset();
        observed_features.clear();
        observed_points.clear();
        visible_points.clear();
        reference_frame_id.reset();
        local_frames.clear();
        stage = TrackingStage::MotionModel;
        num_matches = 0;
        num_inliers = 0;
        urgent = false;
        map_epoch = 0;
        priority = false;
    }

    FrameTracked()
        : Event(EventType::FrameTracked,
                [](Event* e) {
                    static_cast<FrameTracked*>(e)->
                        memory::Recyclable<FrameTracked, memory::ThreadSafePolicy>::Recycle();
                },
                [](Event* e, int n) {
                    static_cast<FrameTracked*>(e)->SetRefCount(n);
                }) {}
};

} // namespace voyahslam::events

#endif // VOYAHSLAM_EVENTS_FRAME_TRACKED_H
