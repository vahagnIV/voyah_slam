#ifndef VOYAHSLAM_DEFS_H
#define VOYAHSLAM_DEFS_H

#include <Eigen/Dense>
#ifdef _WINDOWS
#define _USE_MATH_DEFINES
#include <math.h>
#endif

namespace voyahslam {
typedef double precision_t;
}

namespace voyahslam::geometry {

typedef Eigen::Matrix<precision_t, 3, 1> TPoint3DF;
typedef Eigen::Matrix<precision_t, 3, 1> THomogenousPointF;
typedef Eigen::Matrix<precision_t, 3, Eigen::Dynamic> TPoints3DF;
typedef Eigen::Matrix<precision_t, 3, Eigen::Dynamic> THomogenousPointsF;

typedef Eigen::Matrix<precision_t, 2, 1> TPoint2DF;
typedef Eigen::Matrix<int32_t, 2, 1> TPoint2DI;
typedef Eigen::Matrix<precision_t, 2, Eigen::Dynamic> TPoints2DF;

typedef Eigen::Matrix<precision_t, 2, 1> TVector2DF;
typedef Eigen::Matrix<precision_t, 3, 1> TVector3DF;
typedef Eigen::Matrix<precision_t, 4, 1> TVector4DF;

typedef Eigen::Matrix<precision_t, 2, 1> TColVector2DF;
typedef Eigen::Matrix<precision_t, 3, 1> TColVector3DF;
typedef Eigen::Matrix<precision_t, 4, 1> TColVector4DF;

typedef Eigen::RowVector<precision_t, Eigen::Dynamic> TRowVectorF;
typedef Eigen::Vector<precision_t, Eigen::Dynamic> TColVectorF;
typedef Eigen::Array<precision_t, 1, Eigen::Dynamic> TRowArrayF;
typedef Eigen::Array<precision_t, Eigen::Dynamic, 1> TColArrayF;

typedef Eigen::Matrix<precision_t, 2, 2> TMatrix22F;
typedef Eigen::Matrix<precision_t, 3, 3> TMatrix33F;
typedef Eigen::Matrix<precision_t, 3, 4> TMatrix34F;
typedef Eigen::Matrix<precision_t, 4, 4> TMatrix44F;

typedef Eigen::Matrix<precision_t, 3, 4> TPose;

typedef Eigen::Matrix<precision_t, Eigen::Dynamic, Eigen::Dynamic> TMatrixXF;

typedef Eigen::Matrix<precision_t, 1, 1> TScalarF;

} // namespace voyahslam::geometry

#endif // VOYAHSLAM_DEFS_H
