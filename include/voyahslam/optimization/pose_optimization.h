#ifndef VOYAHSLAM_OPTIMIZATION_POSE_OPTIMIZATION_H
#define VOYAHSLAM_OPTIMIZATION_POSE_OPTIMIZATION_H

#include <cstddef>
#include <optional>
#include <vector>

#include "voyahslam/defs.h"
#include "voyahslam/geometry/camera.h"
#include "voyahslam/geometry/rigid_transformation.h"

namespace voyahslam::optimization {

/// A world point (fixed) observed by the frame whose pose is optimized.
struct PoseObservation {
    geometry::TPoint3DF point;                  ///< World frame, fixed.
    geometry::TPoint2DF left;                   ///< Raw (distorted) pixel, left camera.
    std::optional<geometry::TPoint2DF> right;   ///< Raw pixel in the right camera (stereo match).
    double sigma;                               ///< Pixels (sigma_px * scale^level).
};

/// The right camera of a stereo rig and the left -> right transform.
struct StereoCameras {
    geometry::Camera right;
    geometry::RigidTransformation T_rl;
};

/// ORB-SLAM3 PoseOptimization schedule: rounds of iterations_per_round iterations, every
/// observation re-classified after each round (chi2 above the threshold of its edge type or
/// depth <= 0 -> outlier, excluded from the next round). Huber kernel (delta = sqrt(chi2))
/// in rounds 0-1, no kernel from round 2.
struct PoseOptimizationOptions {
    int rounds = 4;
    int iterations_per_round = 10;
    double mono_chi2 = 5.991;    ///< 2 dof, 95 %.
    double stereo_chi2 = 9.488;  ///< 4 dof (left + right pixels), 95 %.
};

struct PoseOptimizationResult {
    bool success = false;
    geometry::RigidTransformation T_cw;
    std::vector<bool> inlier;    ///< Per observation, from the last round's classification.
    size_t num_inliers = 0;
};

/// Motion-only optimization of a frame pose T_cw (world -> left camera) against fixed world
/// points, in raw pixels: mono observations constrain the left camera
/// (e = left.Project(T_cw X) - z), stereo ones (right set) both cameras, the right one through
/// stereo->T_rl (left -> right) and stereo->right. success = num_inliers >= 3 and a finite pose.
/// Invalid input (no observations, a non-finite value or K, sigma <= 0, a right observation
/// without stereo) yields success == false without optimizing.
[[nodiscard]] PoseOptimizationResult OptimizePose(
    const geometry::RigidTransformation& initial_T_cw,
    const geometry::Camera& left,
    const std::optional<StereoCameras>& stereo,   ///< Required if any `right` is set.
    const std::vector<PoseObservation>& observations,
    const PoseOptimizationOptions& options);

}  // namespace voyahslam::optimization

#endif  // VOYAHSLAM_OPTIMIZATION_POSE_OPTIMIZATION_H
