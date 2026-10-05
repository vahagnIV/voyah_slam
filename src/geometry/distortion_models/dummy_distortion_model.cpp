#include "voyahslam/geometry/distortion_models/dummy_distortion_model.h"

namespace voyahslam::geometry::distortion_models {

void DummyDistortionModel::Undistort(geometry::THomogenousPointsF &points) const {
}

void DummyDistortionModel::Distort(geometry::THomogenousPointsF &points) const {
}

} // namespace voyahslam::geometry::distortion_models
