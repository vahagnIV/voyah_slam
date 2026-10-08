#ifndef VOYAHSLAM_GEOMETRY_RANSAC_SUPPORT_H
#define VOYAHSLAM_GEOMETRY_RANSAC_SUPPORT_H

// Helpers shared by the reprojection-scored RANSACs (SolvePnPRansac, SolveSim3Ransac).

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>

#include "voyahslam/defs.h"
#include "voyahslam/geometry/camera.h"

namespace voyahslam::geometry::detail {

/// Raw-pixel χ² ‖(camera.Project(Xc) - pixel)‖² · inv_sigma2 of a camera-frame point; +inf when
/// it is not in front of the camera (Project's precondition) or the projection is not finite.
inline double ReprojectionChi2(const Camera& camera, const TPoint3DF& Xc, const TPoint2DF& pixel, double inv_sigma2) {
    if (!(Xc.z() > 0.0)) return std::numeric_limits<double>::infinity();
    const double chi2 = (camera.Project(Xc) - pixel).squaredNorm() * inv_sigma2;
    return std::isnan(chi2) ? std::numeric_limits<double>::infinity() : chi2;
}

/// Iterations for an all-inlier sample of sample_size with the given confidence,
/// log(1 - confidence) / log(1 - w^sample_size), w = num_inliers / n, capped at max_iterations.
/// w = 1 gives 0 (stop); a w^k that rounds 1 - w^k to 1 gives +inf and NaN fails the
/// comparison: both keep the cap.
inline int AdaptiveIterations(size_t num_inliers, size_t n, int sample_size, double confidence, int max_iterations) {
    const double w = static_cast<double>(num_inliers) / static_cast<double>(n);
    double all_inliers = 1.0;
    for (int i = 0; i < sample_size; ++i) all_inliers *= w;
    const double needed = std::ceil(std::log(1.0 - confidence) / std::log1p(-all_inliers));
    return needed < static_cast<double>(max_iterations) ? static_cast<int>(std::max(needed, 0.0)) : max_iterations;
}

}  // namespace voyahslam::geometry::detail

#endif  // VOYAHSLAM_GEOMETRY_RANSAC_SUPPORT_H
