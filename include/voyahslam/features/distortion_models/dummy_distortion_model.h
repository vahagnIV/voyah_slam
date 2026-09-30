#ifndef VOYAHSLAM_FEATURES_DISTORTION_MODELS_DUMMY_DISTORTION_MODEL_H
#define VOYAHSLAM_FEATURES_DISTORTION_MODELS_DUMMY_DISTORTION_MODEL_H

#include <voyahslam/features/distortion_models/distortion_model.h>

namespace voyahslam::features::distortion_models {

class DummyDistortionModel : public DistortionModel {
public:
    DummyDistortionModel() = default;
    virtual ~DummyDistortionModel() = default;
    void Undistort(geometry::THomogenousPointsF& points) const override;
    void Distort(geometry::THomogenousPointsF& points) const override;
};

} // namespace voyahslam::features::distortion_models

#endif // VOYAHSLAM_FEATURES_DISTORTION_MODELS_DUMMY_DISTORTION_MODEL_H
