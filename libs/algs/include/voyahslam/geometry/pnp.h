#ifndef VOYAHSLAM_GEOMETRY_PNP_H
#define VOYAHSLAM_GEOMETRY_PNP_H

#include <array>
#include <cstddef>
#include <cstdint>
#include <vector>

#include "voyahslam/defs.h"
#include "voyahslam/geometry/camera.h"
#include "voyahslam/geometry/rigid_transformation.h"

namespace voyahslam::geometry {

/// Lambda Twist P3P (Persson, Nordberg. "Lambda Twist: An Accurate Fast Robust Perspective
/// Three Point (P3P) Solver". ECCV 2018). Returns up to 4 poses T_cw mapping each world point
/// onto its unit bearing (camera frame) at positive depth. Bearings need not be normalized.
/// Returns none for degenerate input (collinear world points, parallel bearings).
[[nodiscard]] std::vector<RigidTransformation> SolveP3P(const std::array<TPoint3DF, 3>& world,
                                                        const std::array<TPoint3DF, 3>& bearings);

struct PnpOptions {
    int max_iterations = 300;
    double confidence = 0.99;
    size_t min_inliers = 10;
    double chi2 = 5.991;  ///< Raw-pixel χ² inlier threshold (2 dof, 95 %).
    uint64_t seed = 0;
};

struct PnpResult {
    bool success = false;
    RigidTransformation T_cw;   ///< Best hypothesis; identity on failure.
    std::vector<bool> inlier;   ///< Per point; all false on failure (empty on invalid input).
    size_t num_inliers = 0;     ///< 0 on failure.
    int iterations = 0;
};

/// RANSAC over Lambda Twist P3P. Each sample takes 3 distinct points to generate up to 4
/// candidates (on unit bearings from camera.Unproject) and a 4th to pick the one with the lowest
/// χ² there. The pick is scored on all points: inlier when the depth is > 0 and the raw-pixel
/// χ² = ‖(camera.Project(T_cw X) - pixel) / σ‖² < options.chi2. Best = most inliers, ties to the
/// lower inlier χ² sum. Iterations adapt as log(1 - confidence) / log(1 - w³), w the best inlier
/// ratio so far, capped at max_iterations. Deterministic for a given seed. Success means at least
/// min_inliers inliers; fewer than 4 points or mismatched sizes fail immediately. The pose is not
/// refined (relocalization runs OptimizePose on all of the candidate's matches, Ruling F).
[[nodiscard]] PnpResult SolvePnPRansac(const Camera& camera, const std::vector<TPoint3DF>& world,
                                       const std::vector<TPoint2DF>& pixels, const std::vector<double>& sigmas,
                                       const PnpOptions& options);

}  // namespace voyahslam::geometry

#endif  // VOYAHSLAM_GEOMETRY_PNP_H
