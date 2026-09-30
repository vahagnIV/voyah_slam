#ifndef VOYAHSLAM_FEATURES_STEREO_RIG_H
#define VOYAHSLAM_FEATURES_STEREO_RIG_H

#include <voyahslam/defs.h>
#include <voyahslam/features/distortion_models/distortion_model.h>

namespace voyahslam::features {

struct StereoRig {
    geometry::TMatrix33F K_left;
    geometry::TMatrix33F K_right;
    geometry::TMatrix33F R_rl;
    geometry::TVector3DF t_rl;
    precision_t baseline;
    const distortion_models::DistortionModel* distortion_left = nullptr;
    const distortion_models::DistortionModel* distortion_right = nullptr;
};

} // namespace voyahslam::features

#endif // VOYAHSLAM_FEATURES_STEREO_RIG_H
