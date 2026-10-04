#ifndef VOYAHSLAM_FEATURES_STEREO_RIG_H
#define VOYAHSLAM_FEATURES_STEREO_RIG_H

#include <voyahslam/defs.h>
#include <voyahslam/geometry/rigid_transformation.h>
#include <voyahslam/features/distortion_models/distortion_model.h>

namespace voyahslam::features {

struct StereoRig {
    geometry::TMatrix33F K_left;
    geometry::TMatrix33F K_right;
    geometry::RigidTransformation T_rl;  // left camera -> right camera: p_r = T_rl * p_l
    precision_t baseline = 0.0;         // ‖T_rl.translation()‖
    const distortion_models::DistortionModel* distortion_left = nullptr;
    const distortion_models::DistortionModel* distortion_right = nullptr;
};

} // namespace voyahslam::features

#endif // VOYAHSLAM_FEATURES_STEREO_RIG_H
