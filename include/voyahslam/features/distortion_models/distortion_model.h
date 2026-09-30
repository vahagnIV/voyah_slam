#ifndef VOYAHSLAM_FEATURES_DISTORTION_MODELS_DISTORTION_MODEL_H
#define VOYAHSLAM_FEATURES_DISTORTION_MODELS_DISTORTION_MODEL_H

#include <voyahslam/defs.h>

namespace voyahslam::features::distortion_models {

class DistortionModel {
public:
    virtual ~DistortionModel() = default;
    virtual void Undistort(geometry::THomogenousPointsF &points) const = 0;
    virtual void Distort(geometry::THomogenousPointsF &points) const = 0;
};

} // namespace voyahslam::features::distortion_models

#endif // VOYAHSLAM_FEATURES_DISTORTION_MODELS_DISTORTION_MODEL_H
