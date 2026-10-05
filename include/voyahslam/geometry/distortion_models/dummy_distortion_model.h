#ifndef VOYAHSLAM_GEOMETRY_DISTORTION_MODELS_DUMMY_DISTORTION_MODEL_H
#define VOYAHSLAM_GEOMETRY_DISTORTION_MODELS_DUMMY_DISTORTION_MODEL_H

#include <voyahslam/geometry/distortion_models/distortion_model.h>

namespace voyahslam::geometry::distortion_models {

class DummyDistortionModel : public DistortionModel {
public:
    DummyDistortionModel() = default;
    virtual ~DummyDistortionModel() = default;
    void Undistort(geometry::THomogenousPointsF& points) const override;
    void Distort(geometry::THomogenousPointsF& points) const override;
};

} // namespace voyahslam::geometry::distortion_models

#endif // VOYAHSLAM_GEOMETRY_DISTORTION_MODELS_DUMMY_DISTORTION_MODEL_H
