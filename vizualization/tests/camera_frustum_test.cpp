#include <voyahviz/camera_frustum.h>

#include <gtest/gtest.h>

#include <voyahslam/geometry/rigid_transformation.h>

namespace voyahviz {
namespace {

using voyahslam::geometry::RigidTransformation;
using voyahslam::geometry::TMatrix33F;
using voyahslam::geometry::TPoint3DF;
using voyahslam::geometry::TVector3DF;

TEST(CameraFrustumTest, IdentityPose) {
    const double size = 0.1;
    const double w = size * 2.0 / 3.0;
    const double h = size * 1.0 / 2.0;

    const RigidTransformation T_cw = RigidTransformation::Identity();
    const CameraFrustum frustum = FrustumInWorld(T_cw, size);

    // Centre should be at origin
    EXPECT_DOUBLE_EQ(frustum.centre(0), 0.0);
    EXPECT_DOUBLE_EQ(frustum.centre(1), 0.0);
    EXPECT_DOUBLE_EQ(frustum.centre(2), 0.0);

    // Corners in camera frame: (±w, ±h, size)
    // top-left: (-w, -h, size)
    EXPECT_DOUBLE_EQ(frustum.corners[0](0), -w);
    EXPECT_DOUBLE_EQ(frustum.corners[0](1), -h);
    EXPECT_DOUBLE_EQ(frustum.corners[0](2), size);

    // top-right: (w, -h, size)
    EXPECT_DOUBLE_EQ(frustum.corners[1](0), w);
    EXPECT_DOUBLE_EQ(frustum.corners[1](1), -h);
    EXPECT_DOUBLE_EQ(frustum.corners[1](2), size);

    // bottom-right: (w, h, size)
    EXPECT_DOUBLE_EQ(frustum.corners[2](0), w);
    EXPECT_DOUBLE_EQ(frustum.corners[2](1), h);
    EXPECT_DOUBLE_EQ(frustum.corners[2](2), size);

    // bottom-left: (-w, h, size)
    EXPECT_DOUBLE_EQ(frustum.corners[3](0), -w);
    EXPECT_DOUBLE_EQ(frustum.corners[3](1), h);
    EXPECT_DOUBLE_EQ(frustum.corners[3](2), size);
}

TEST(CameraFrustumTest, NonTrivialPose) {
    const double size = 0.2;
    const double w = size * 2.0 / 3.0;
    const double h = size * 1.0 / 2.0;

    // Create a rotation matrix (90 degree rotation about z-axis)
    TMatrix33F R;
    R << 0, -1, 0,
         1, 0, 0,
         0, 0, 1;

    // Non-zero translation
    TVector3DF t(5.0, -3.0, 2.0);

    const RigidTransformation T_cw(R, t);
    const CameraFrustum frustum = FrustumInWorld(T_cw, size);

    // Centre should be approximately T_cw.Inverse().translation()
    const TVector3DF expected_centre = T_cw.Inverse().translation();
    EXPECT_NEAR(frustum.centre(0), expected_centre(0), 1e-12);
    EXPECT_NEAR(frustum.centre(1), expected_centre(1), 1e-12);
    EXPECT_NEAR(frustum.centre(2), expected_centre(2), 1e-12);

    // Camera frame corners
    std::array<TPoint3DF, 4> camera_corners = {
        TPoint3DF(-w, -h, size),
        TPoint3DF(w, -h, size),
        TPoint3DF(w, h, size),
        TPoint3DF(-w, h, size),
    };

    // When we transform world corners back to camera frame using T_cw.Transform(),
    // we should get back the camera frame corners
    for (int i = 0; i < 4; ++i) {
        const TPoint3DF transformed = T_cw.Transform(frustum.corners[i]);
        EXPECT_NEAR(transformed(0), camera_corners[i](0), 1e-12);
        EXPECT_NEAR(transformed(1), camera_corners[i](1), 1e-12);
        EXPECT_NEAR(transformed(2), camera_corners[i](2), 1e-12);
    }
}

} // namespace
} // namespace voyahviz
