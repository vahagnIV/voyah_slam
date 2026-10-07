#include "voyahslam/algebra/algebra.h"

namespace voyahslam::algebra {

geometry::TMatrix33F
    GetHartleyNormalizationMatrix(const geometry::TPoints2DF &a_pts) {
    auto centroid = a_pts.rowwise().mean();
    precision_t rms_dist = (a_pts.colwise() - centroid).colwise().squaredNorm().mean();
    rms_dist = std::sqrt(rms_dist);

    precision_t scale = std::sqrt(2) / rms_dist;
    geometry::TMatrix33F T;
    T << scale, 0, -scale * centroid(0),
         0, scale, -scale * centroid(1),
         0, 0, 1;

    return T;
}

} // namespace voyahslam::algebra
