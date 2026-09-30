#ifndef VOYAHSLAM_ALGEBRA_ALGEBRA_H
#define VOYAHSLAM_ALGEBRA_ALGEBRA_H

#include <voyahslam/defs.h>
#include <iostream>

namespace voyahslam::algebra {

template<typename Scalar, int Rows, int Cols>
Eigen::Matrix<Scalar, Cols, 1> SolveHomogeneousSystem(const Eigen::Matrix<Scalar, Rows, Cols> &a_A) {
    Eigen::BDCSVD<Eigen::Matrix<Scalar, Rows, Cols>> svd(a_A, Eigen::ComputeFullV);
    return svd.matrixV().rightCols(1);
}

template<typename Scalar, int Rows, int Cols>
Eigen::Matrix<Scalar, Rows, Cols> ReduceMatrixRank(const Eigen::Matrix<Scalar, Rows, Cols> &a_mat,
                                                   const unsigned int &a_rank_decrease_count,
                                                   const unsigned int &a_min_rank = 0) {
    if (a_rank_decrease_count == 0) return a_mat;
    Eigen::BDCSVD<Eigen::Matrix<Scalar, Rows, Cols>> svd(a_mat, Eigen::ComputeFullU | Eigen::ComputeFullV);
    auto singularValues = svd.singularValues();
    int rank = svd.rank();

    if (a_min_rank >= rank) return a_mat;
    Eigen::Matrix<Scalar, Rows, Cols> Sigma = Eigen::Matrix<Scalar, Rows, Cols>::Zero(a_mat.rows(), a_mat.cols());
    int new_rank = std::max((int)a_min_rank, rank - (int)a_rank_decrease_count);
    Sigma.topLeftCorner(new_rank, new_rank) = singularValues.topRows(new_rank).asDiagonal();
    auto A = svd.matrixU() * Sigma * svd.matrixV().transpose();
    return A;
}

// Hartley, (1997). In defense of the 8-point algorithm.
// Szeliski, Computer Vision: Algorithms and Applications, 2nd Ed. 2022. (p706)
// Hartley, Zisserman. Multiple View Geometry in Computer Vision. 2004. (sec 11.2)
geometry::TMatrix33F
    GetHartleyNormalizationMatrix(const geometry::TPoints2DF &a_pts);

} // namespace voyahslam::algebra

#endif // VOYAHSLAM_ALGEBRA_ALGEBRA_H
