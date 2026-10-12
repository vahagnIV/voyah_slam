#ifndef VOYAHSLAM_GEOMETRY_CAMERA_H
#define VOYAHSLAM_GEOMETRY_CAMERA_H

#include <voyahslam/defs.h>
#include <voyahslam/geometry/distortion_models/distortion_model.h>

namespace voyahslam::geometry {

// Intrinsics of one camera: pinhole K plus an optional lens distortion model.
// The pixel is K * [distorted normalized; 1], i.e. raw (distorted) pixels.
struct Camera {
    TMatrix33F K;
    distortion_models::TDistortionModelPtr distortion;  // null = pinhole; shared by every Camera copy

    // Camera-frame point -> raw (distorted) pixel. Precondition: Xc.z() > 0.
    [[nodiscard]] TPoint2DF Project(const TPoint3DF& Xc) const;
    // d(raw pixel)/d(Xc), 2x3: K22 * DistortJacobian(Xc.xy/Xc.z) * [[1/z, 0, -x/z^2], [0, 1/z, -y/z^2]].
    [[nodiscard]] Eigen::Matrix<precision_t, 2, 3> ProjectJacobian(const TPoint3DF& Xc) const;
    // Raw pixel -> undistorted normalized ray (z = 1), via K^-1 and Undistort.
    [[nodiscard]] THomogenousPointF Unproject(const TPoint2DF& pixel) const;
};

}  // namespace voyahslam::geometry

#endif  // VOYAHSLAM_GEOMETRY_CAMERA_H
