#ifndef VOYAHSLAM_TRACKING_TRACKER_H
#define VOYAHSLAM_TRACKING_TRACKER_H

#include <voyahslam/events/event_node.h>
#include <voyahslam/features/binocular_camera_frame.h>
#include <voyahslam/features/camera_frame.h>

namespace voyahslam::tracking {

// Estimates the camera pose of every incoming frame against its own replica of
// the map. Publishes PositionUpdated and NewKeyFrameCandidate; consumes the
// Mapper's map diffs. Never modifies map objects.
template <typename TFeatureExtractor>
class Tracker : public events::EventNode {
public:
    using TCameraFrame = features::CameraFrame<typename TFeatureExtractor::TDescriptorScalarType,
                                               TFeatureExtractor::DescriptorSize>;
    using TBinocularCameraFrame =
        features::BinocularCameraFrame<typename TFeatureExtractor::TDescriptorScalarType,
                                       TFeatureExtractor::DescriptorSize>;

    void Start();
    void Stop();

    void Track(const TCameraFrame& frame);
    void Track(const TBinocularCameraFrame& frame);
};

} // namespace voyahslam::tracking

#endif // VOYAHSLAM_TRACKING_TRACKER_H
