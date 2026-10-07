#ifndef VOYAHSLAM_GEOMETRY_TWO_VIEW_CORRESPONDENCES_H
#define VOYAHSLAM_GEOMETRY_TWO_VIEW_CORRESPONDENCES_H

#include <Eigen/Dense>

#include "voyahslam/defs.h"

namespace voyahslam::geometry {

/// Matched points between two views, in normalized image coordinates (z = 1).
struct TwoViewCorrespondences {
    TPoints2DF x1;       ///< 2xN normalized coordinates in view 1.
    TPoints2DF x2;       ///< 2xN normalized coordinates in view 2 (column i matches x1 column i).
    double sigma = 0.0;  ///< Measurement noise std-dev in normalized units (sigma_px / f).

    [[nodiscard]] Eigen::Index size() const { return x1.cols(); }
};

} // namespace voyahslam::geometry

#endif // VOYAHSLAM_GEOMETRY_TWO_VIEW_CORRESPONDENCES_H
