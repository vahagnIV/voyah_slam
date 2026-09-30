#ifndef VOYAHSLAM_GEOMETRY_GEOMETRY_H
#define VOYAHSLAM_GEOMETRY_GEOMETRY_H

#include <voyahslam/defs.h>

namespace voyahslam::geometry {

// Hartley, Zisserman. Multiple View Geometry in Computer Vision. 2004. (sec 11.4.3)
TColVectorF SquaredSampsonError(const THomogenousPointsF &pts1,
                                const THomogenousPointsF &pts2,
                                const TMatrix33F &E);

TColVectorF SquaredSampsonError(const TPoints2DF &pts1,
                                const TPoints2DF &pts2,
                                const TMatrix33F &E);

precision_t SquaredReprojectionError(const TMatrix33F &a_K, const TMatrix34F &a_pose,
                                    const TPoint3DF &a_3d_point, const TPoint2DF a_image_point);

// Zisserman 12.2 Linear triangulation methods p. 312
bool Triangulate(const THomogenousPointF &pt1, const THomogenousPointF &pt2,
                 const TMatrix34F &P1, const TMatrix34F &P2,
                 TPoint3DF &out_triangulated);

std::array<TMatrix34F, 4> DecomposeEssentialToPoses(const TMatrix33F &E);

precision_t ComputeParallaxCos(const TPoint3DF &a_p3d, const TMatrix33F &a_R, const TColVector3DF &a_T);
precision_t ComputeParallaxCos(const TPoint3DF &a_p3d, const TMatrix34F &a_P2);

} // namespace voyahslam::geometry

#endif // VOYAHSLAM_GEOMETRY_GEOMETRY_H
