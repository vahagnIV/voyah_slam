#include "voyahslam/geometry/distortion_models/dummy_distortion_model.h"

namespace voyahslam::geometry::distortion_models {

void DummyDistortionModel::Undistort(geometry::THomogenousPointsF &points) const {
}

void DummyDistortionModel::Distort(geometry::THomogenousPointsF &points) const {
}

geometry::TMatrix22F DummyDistortionModel::DistortJacobian(const geometry::TPoint2DF &undistorted) const {
    return geometry::TMatrix22F::Identity();
}

} // namespace voyahslam::geometry::distortion_models
