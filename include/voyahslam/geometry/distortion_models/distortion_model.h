#ifndef VOYAHSLAM_GEOMETRY_DISTORTION_MODELS_DISTORTION_MODEL_H
#define VOYAHSLAM_GEOMETRY_DISTORTION_MODELS_DISTORTION_MODEL_H

#include <voyahslam/defs.h>

namespace voyahslam::geometry::distortion_models {

class DistortionModel {
public:
    virtual ~DistortionModel() = default;
    virtual void Undistort(geometry::THomogenousPointsF &points) const = 0;
    virtual void Distort(geometry::THomogenousPointsF &points) const = 0;
};

} // namespace voyahslam::geometry::distortion_models

#endif // VOYAHSLAM_GEOMETRY_DISTORTION_MODELS_DISTORTION_MODEL_H
