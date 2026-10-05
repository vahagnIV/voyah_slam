#ifndef VOYAHSLAM_OPTIMIZATION_BUNDLE_ADJUSTMENT_H
#define VOYAHSLAM_OPTIMIZATION_BUNDLE_ADJUSTMENT_H

#include <cstddef>
#include <vector>

#include <Eigen/Dense>

#include "voyahslam/defs.h"
#include "voyahslam/geometry/rigid_transformation.h"

namespace voyahslam::optimization {

/// Two stages like ORB-SLAM3's local BA: robust_iterations with the Huber kernel, then the
/// edges with chi2 > huber_chi2 or depth <= 0 are deactivated and iterations more run without
/// a kernel. With robust == false: one plain pass of robust_iterations + iterations.
struct BundleAdjustmentOptions {
    int robust_iterations = 5;    ///< Stage 1 (Huber kernel).
    int iterations = 10;          ///< Stage 2 (no kernel, outlier edges deactivated).
    bool robust = true;
    double huber_chi2 = 5.991;    ///< delta^2 of the kernel and the outlier test (2 dof, 95 %).
};

struct BaObservation {
    size_t keyframe;                    ///< Index into keyframe_poses.
    size_t point;                       ///< Column of points.
    geometry::TPoint2DF measurement;    ///< Undistorted normalized coordinates (z = 1).
    double sigma;                       ///< Normalized units.
};

struct BaProblem {
    std::vector<geometry::RigidTransformation> keyframe_poses;   ///< T_cw.
    std::vector<bool> keyframe_fixed;
    geometry::TPoints3DF points;                                 ///< World frame.
    std::vector<BaObservation> observations;
};

struct BaResult {
    bool success = false;
    std::vector<geometry::RigidTransformation> keyframe_poses;
    geometry::TPoints3DF points;
    std::vector<double> chi2;      ///< Per observation (all, incl. deactivated), final, no kernel.
    std::vector<bool> inlier;      ///< chi2 <= huber_chi2 && camera-frame depth > 0.
    double initial_cost = 0.0;     ///< Sum of the plain chi2 over all observations at the start.
    double final_cost = 0.0;       ///< Sum of the plain chi2 over the edges of the last pass.
    int iterations = 0;            ///< Total over both stages.
};

/// Jointly refines keyframe poses (T_cw) and world points by minimizing the normalized
/// reprojection error of the observations (Levenberg-Marquardt, points marginalized).
/// Fixed keyframes keep their input pose. Invalid input (no keyframes, points or
/// observations; keyframe_fixed of the wrong size; a non-finite pose, point or measurement;
/// sigma <= 0; an out-of-range index) yields success == false without optimizing.
[[nodiscard]] BaResult BundleAdjust(const BaProblem& problem,
                                    const BundleAdjustmentOptions& options);

}  // namespace voyahslam::optimization

#endif  // VOYAHSLAM_OPTIMIZATION_BUNDLE_ADJUSTMENT_H
