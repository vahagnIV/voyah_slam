#ifndef VOYAHSLAM_MAP_FRAME_H
#define VOYAHSLAM_MAP_FRAME_H

#include <utility>
#include <voyahslam/defs.h>
#include <voyahslam/features/binocular_camera_frame.h>
#include <voyahslam/features/camera_frame.h>
#include <voyahslam/map/frame_id.h>

namespace voyahslam::map {

template <typename TFeatureExtractor, typename TCameraFrameType>
class Frame {
public:
    Frame(FrameId id, const geometry::TPose& pose,
          TCameraFrameType&& camera_frame, double timestamp)
        : id_(id)
        , pose_(pose)
        , camera_frame_(std::move(camera_frame))
        , timestamp_(timestamp) {}

    Frame(const Frame&) = default;
    Frame& operator=(const Frame&) = default;
    Frame(Frame&&) = default;
    Frame& operator=(Frame&&) = default;

    [[nodiscard]] FrameId id() const { return id_; }
    [[nodiscard]] const geometry::TPose& pose() const { return pose_; }
    geometry::TPose& mutable_pose() { return pose_; }
    [[nodiscard]] const TCameraFrameType& camera_frame() const { return camera_frame_; }
    [[nodiscard]] double timestamp() const { return timestamp_; }

private:
    FrameId id_;
    geometry::TPose pose_;
    TCameraFrameType camera_frame_;
    double timestamp_;
};

template <typename FE>
using MonoFrame = Frame<FE,
    features::CameraFrame<typename FE::TDescriptorScalarType, FE::DescriptorSize>>;

template <typename FE>
using StereoFrame = Frame<FE,
    features::BinocularCameraFrame<typename FE::TDescriptorScalarType, FE::DescriptorSize>>;

} // namespace voyahslam::map

#endif // VOYAHSLAM_MAP_FRAME_H
