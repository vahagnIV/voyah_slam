#ifndef VOYAHSLAM_MAP_FRAME_H
#define VOYAHSLAM_MAP_FRAME_H

#include <cassert>
#include <memory>
#include <utility>
#include <voyahslam/defs.h>
#include <voyahslam/features/binocular_measurement.h>
#include <voyahslam/features/monocular_measurement.h>
#include <voyahslam/geometry/rigid_transformation.h>
#include <voyahslam/map/frame_id.h>

namespace voyahslam::map {

/// Represents a camera frame.
/// pose = T_cw (world → camera)
/// The measurement is immutable and shared (never null): copies of a Frame,
/// and every holder of shared_measurement(), see the same object.
template <typename TFeatureExtractor, typename TMeasurement>
class Frame {
public:
    using TSharedMeasurement = std::shared_ptr<const TMeasurement>;

    Frame(FrameId id, const geometry::RigidTransformation& pose, TSharedMeasurement measurement)
        : id_(id)
        , pose_(pose)
        , measurement_(std::move(measurement)) {
        assert(measurement_ != nullptr);
    }

    Frame(const Frame&) = default;
    Frame& operator=(const Frame&) = default;
    Frame(Frame&&) = default;
    Frame& operator=(Frame&&) = default;

    [[nodiscard]] FrameId id() const { return id_; }
    [[nodiscard]] const geometry::RigidTransformation& pose() const { return pose_; }
    geometry::RigidTransformation& mutable_pose() { return pose_; }
    [[nodiscard]] const TMeasurement& measurement() const { return *measurement_; }
    [[nodiscard]] const TSharedMeasurement& shared_measurement() const { return measurement_; }
    [[nodiscard]] double timestamp() const { return measurement_->timestamp(); }

private:
    FrameId id_;
    geometry::RigidTransformation pose_;
    TSharedMeasurement measurement_;
};

template <typename FE>
using MonoFrame = Frame<FE,
    features::MonocularMeasurement<typename FE::TDescriptorScalarType, FE::DescriptorSize>>;

template <typename FE>
using StereoFrame = Frame<FE,
    features::BinocularMeasurement<typename FE::TDescriptorScalarType, FE::DescriptorSize>>;

} // namespace voyahslam::map

#endif // VOYAHSLAM_MAP_FRAME_H
