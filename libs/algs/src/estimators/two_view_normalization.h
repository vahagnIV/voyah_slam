#ifndef VOYAHSLAM_SRC_ESTIMATORS_TWO_VIEW_NORMALIZATION_H
#define VOYAHSLAM_SRC_ESTIMATORS_TWO_VIEW_NORMALIZATION_H

#include <cmath>

#include "voyahslam/algebra/algebra.h"
#include "voyahslam/defs.h"

namespace voyahslam::estimators::internal {

/// Hartley normalization matrix of a_pts. Returns false if the points are (nearly)
/// identical or non-finite, so no normalization exists.
inline bool Normalize(const geometry::TPoints2DF& a_pts, geometry::TMatrix33F& out_T) {
    const geometry::TPoint2DF centroid = a_pts.rowwise().mean();
    const double spread = (a_pts.colwise() - centroid).colwise().squaredNorm().mean();
    if (!std::isfinite(spread) || spread <= 0.0) return false;
    out_T = algebra::GetHartleyNormalizationMatrix(a_pts);
    return out_T.allFinite();
}

} // namespace voyahslam::estimators::internal

#endif // VOYAHSLAM_SRC_ESTIMATORS_TWO_VIEW_NORMALIZATION_H
