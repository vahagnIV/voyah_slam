#ifndef VOYAHSLAM_TEST_UTILS_TEST_H
#define VOYAHSLAM_TEST_UTILS_TEST_H

#include "voyahslam/defs.h"
#include "voyahslam/features/keypoint_matches.h"

namespace voyahslam::test {

struct TwoViewSystem
{
    geometry::TMatrix33F R;
    geometry::TVector3DF T;
    geometry::TMatrix33F K;
    geometry::TPoints3DF triangulated;
    // key points
    geometry::TPoints2DF kps1;
    geometry::TPoints2DF kps2;
    // normalized points
    geometry::THomogenousPointsF nps1;
    geometry::THomogenousPointsF nps2;
    features::KeypointMatches matches;
    geometry::TRowVectorF dists;
};

// Generates a random double matrix with elements in range [0, 1)
template <typename T = precision_t>
 Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> CreateRandomMatrix(unsigned int a_rows, unsigned int a_cols);

// Generates a_n random set of key points in the range [0, width) and [0, height)
geometry::TPoints2DF CreateRandomKeyPoints(int a_n, unsigned int a_width, unsigned int a_height);

// Creates a rotation matrix from Euler angles in degrees
geometry::TMatrix33F CreateRotationMatrixD(precision_t a_alpha, precision_t a_beta, precision_t a_gamma);

// Creates a rotation matrix from Euler angles in radians
geometry::TMatrix33F CreateRotationMatrix(precision_t alpha, precision_t beta, precision_t gamma);

// Returns matrix rank using JacobiSVD decomposition
unsigned int GetMatrixRank(const Eigen::MatrixXd& a_mat);

// Returns skew-symmetric matrix for cross product
geometry::TMatrix33F CrossProductMatrix(const geometry::TRowVectorF& a_v);
} // namespace voyahslam::test

#endif // VOYAHSLAM_TEST_UTILS_TEST_H
