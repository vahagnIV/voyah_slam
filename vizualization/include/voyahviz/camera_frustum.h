#ifndef VOYAHVIZ_CAMERA_FRUSTUM_H
#define VOYAHVIZ_CAMERA_FRUSTUM_H

#include <array>

#include <voyahslam/geometry/rigid_transformation.h>

namespace voyahviz {

/// Frustum geometry for a camera in world coordinates.
struct CameraFrustum {
    voyahslam::geometry::TPoint3DF centre;
    std::array<voyahslam::geometry::TPoint3DF, 4> corners;  // top-left, top-right, bottom-right, bottom-left in the camera image
};

/// Compute the camera frustum in world coordinates.
/// Frustum is defined in camera coordinates with:
///  - centre at (0, 0, 0)
///  - corners at (±w, ±h, size) where w = size * 2/3, h = size * 1/2
///  - x- is left, y- is top (y down)
/// All points are transformed to world coordinates using T_cw.Inverse().
/// @param T_cw Rigid transformation mapping world → camera (T_cw * p_world = p_camera)
/// @param size Depth (z-coordinate) of the frustum corners in camera frame
/// @return Camera frustum with all points in world coordinates
[[nodiscard]] inline CameraFrustum FrustumInWorld(const voyahslam::geometry::RigidTransformation& T_cw,
                                                  double size) {
    using voyahslam::geometry::TPoint3DF;

    const double w = size * 2.0 / 3.0;
    const double h = size * 1.0 / 2.0;

    // Camera frame points
    const TPoint3DF centre_camera = TPoint3DF(0.0, 0.0, 0.0);
    const std::array<TPoint3DF, 4> corners_camera = {
        TPoint3DF(-w, -h, size),  // top-left
        TPoint3DF(w, -h, size),   // top-right
        TPoint3DF(w, h, size),    // bottom-right
        TPoint3DF(-w, h, size),   // bottom-left
    };

    // Transform to world coordinates
    const voyahslam::geometry::RigidTransformation T_wc = T_cw.Inverse();

    CameraFrustum frustum;
    frustum.centre = T_wc.Transform(centre_camera);

    for (size_t i = 0; i < 4; ++i) {
        frustum.corners[i] = T_wc.Transform(corners_camera[i]);
    }

    return frustum;
}

} // namespace voyahviz

#endif // VOYAHVIZ_CAMERA_FRUSTUM_H
