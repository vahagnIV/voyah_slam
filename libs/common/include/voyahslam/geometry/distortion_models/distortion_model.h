#ifndef VOYAHSLAM_GEOMETRY_DISTORTION_MODELS_DISTORTION_MODEL_H
#define VOYAHSLAM_GEOMETRY_DISTORTION_MODELS_DISTORTION_MODEL_H

#include <memory>

#include <voyahslam/defs.h>

namespace voyahslam::geometry::distortion_models {

class DistortionModel {
public:
    virtual ~DistortionModel() = default;
    virtual void Undistort(geometry::THomogenousPointsF &points) const = 0;
    virtual void Distort(geometry::THomogenousPointsF &points) const = 0;
    // d(distorted normalized) / d(undistorted normalized) at an undistorted
    // normalized point (z = 1 plane).
    [[nodiscard]] virtual geometry::TMatrix22F DistortJacobian(const geometry::TPoint2DF& undistorted) const = 0;
};

// Shared, immutable ownership of a model: every Camera, StereoRig and
// measurement that holds one keeps it alive (null = pinhole). Code that only
// uses a model during a call takes a const DistortionModel* or & instead.
using TDistortionModelPtr = std::shared_ptr<const DistortionModel>;

} // namespace voyahslam::geometry::distortion_models

#endif // VOYAHSLAM_GEOMETRY_DISTORTION_MODELS_DISTORTION_MODEL_H
