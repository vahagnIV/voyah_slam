#include "voyahslam/features/distortion_models/dummy_distortion_model.h"

namespace voyahslam::features::distortion_models {

void DummyDistortionModel::Undistort(geometry::THomogenousPointsF &points) const {
}

void DummyDistortionModel::Distort(geometry::THomogenousPointsF &points) const {
}

} // namespace voyahslam::features::distortion_models
