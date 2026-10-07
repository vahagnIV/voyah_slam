#ifndef VOYAHSLAM_MAP_POINT_REFERENCE_H
#define VOYAHSLAM_MAP_POINT_REFERENCE_H

#include <Eigen/Core>
#include <voyahslam/map/frame_id.h>

namespace voyahslam::map {

// The first keyframe observation of a map point: where its descriptor was seen.
struct PointReference {
    FrameId keyframe;
    Eigen::Index feature;  // feature index in the keyframe's measurement
    int level;             // pyramid level of that feature
    double distance;       // ||X - C_kf|| when the keyframe was created
};

} // namespace voyahslam::map

#endif // VOYAHSLAM_MAP_POINT_REFERENCE_H
