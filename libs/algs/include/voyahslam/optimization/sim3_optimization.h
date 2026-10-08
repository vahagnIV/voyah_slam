#ifndef VOYAHSLAM_OPTIMIZATION_SIM3_OPTIMIZATION_H
#define VOYAHSLAM_OPTIMIZATION_SIM3_OPTIMIZATION_H

#include <cstddef>
#include <vector>

#include "voyahslam/defs.h"
#include "voyahslam/geometry/camera.h"
#include "voyahslam/geometry/sim3.h"

namespace voyahslam::optimization {

/// One matched point pair of two Frames: X1 in camera-1 coordinates, observed at the raw pixel
/// pixel1 with sigma1 px (sigma_px * 1.2^level), and X2 in camera-2 coordinates, observed at
/// pixel2 with sigma2. Both points are fixed; only S_12 is optimized.
struct Sim3Match {
    geometry::TPoint3DF X1;
    geometry::TPoint2DF pixel1;
    double sigma1;
    geometry::TPoint3DF X2;
    geometry::TPoint2DF pixel2;
    double sigma2;
};

struct Sim3OptimizationResult {
    bool success = false;
    geometry::Sim3 S_12;       ///< Maps camera-2 coordinates to camera 1; the start on failure.
    std::vector<bool> inlier;  ///< Per match; all false on failure.
    size_t num_inliers = 0;    ///< 0 on failure.
};

/// ORB-SLAM3 Optimizer::OptimizeSim3 in raw pixels through each Frame's Camera. Per match two
/// residuals: camera1.Project(S_12 X2) - pixel1 and camera2.Project(S_12⁻¹ X1) - pixel2, with
/// information 1/sigma². Schedule: 5 Levenberg iterations with a Huber kernel
/// (delta = sqrt(huber_chi2)); a match whose χ² exceeds huber_chi2 in either image, or whose
/// point lands behind a camera, is removed and the kernel dropped from the rest; fewer than 10
/// remaining matches fails; then 5 more iterations (10 if any match was removed), and the
/// matches are re-classified with the same test. With fix_scale (stereo) the scale stays at the
/// start's. A match with a point behind its own camera (X1.z or X2.z <= 0) takes no part.
/// Invalid input (a non-finite value, sigma <= 0, a start scale <= 0, huber_chi2 <= 0) fails
/// without optimizing.
[[nodiscard]] Sim3OptimizationResult OptimizeSim3(const geometry::Camera& camera1, const geometry::Camera& camera2,
                                                  const std::vector<Sim3Match>& matches,
                                                  const geometry::Sim3& initial_S_12, bool fix_scale,
                                                  double huber_chi2 = 10.0);

}  // namespace voyahslam::optimization

#endif  // VOYAHSLAM_OPTIMIZATION_SIM3_OPTIMIZATION_H
