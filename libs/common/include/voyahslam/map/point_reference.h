#ifndef VOYAHSLAM_MAP_POINT_REFERENCE_H
#define VOYAHSLAM_MAP_POINT_REFERENCE_H

#include <Eigen/Core>
#include <voyahslam/map/frame_id.h>

namespace voyahslam::map {

// The reference observation of a map point, used for scale prediction: its
// first observation; when that one is removed, the remaining observation with
// the smallest Frame id.
struct PointReference {
    FrameId keyframe;
    Eigen::Index feature;  // feature index in the keyframe's measurement
    int level;             // pyramid level of that feature
    double distance;       // ||X - C_kf|| when the observation became the reference
};

} // namespace voyahslam::map

#endif // VOYAHSLAM_MAP_POINT_REFERENCE_H
