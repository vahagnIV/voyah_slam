#ifndef VOYAHSLAM_GEOMETRY_SIM3_SOLVER_H
#define VOYAHSLAM_GEOMETRY_SIM3_SOLVER_H

#include <cstddef>
#include <cstdint>
#include <optional>
#include <vector>

#include "voyahslam/defs.h"
#include "voyahslam/geometry/camera.h"
#include "voyahslam/geometry/sim3.h"

namespace voyahslam::geometry {

/// Horn's closed form (Horn 1987, "Closed-form solution of absolute orientation using unit
/// quaternions"), as ORB-SLAM3's Sim3Solver::ComputeSim3: the similarity S with to ≈ S · from in
/// the least-squares sense. The rotation is the unit quaternion of N's largest eigenvalue; the
/// scale is Σ (to - c_to)·R (from - c_from) / Σ ‖from - c_from‖² (exactly 1 with fix_scale); the
/// translation maps the centroids. Returns none for fewer than 3 pairs, mismatched sizes, a
/// (near-)collinear `from` or `to` set, a rotation that is not unique, or a scale that is not
/// positive and finite.
[[nodiscard]] std::optional<Sim3> AlignHorn(const std::vector<TPoint3DF>& from, const std::vector<TPoint3DF>& to,
                                            bool fix_scale);

struct Sim3RansacOptions {
    int max_iterations = 300;
    double confidence = 0.99;
    size_t min_inliers = 20;
    double chi2 = 9.21;  ///< Raw-pixel χ² inlier threshold in each image (2 dof, 99 %).
    bool fix_scale = false;  ///< Stereo / RGB-D: the maps are metric, the scale stays 1.
    uint64_t seed = 0;
};

struct Sim3RansacResult {
    bool success = false;
    Sim3 S_12;                 ///< Maps frame-2 camera coordinates to frame 1; identity on failure.
    std::vector<bool> inlier;  ///< Per pair; all false on failure (empty on invalid input).
    size_t num_inliers = 0;    ///< 0 on failure.
    int iterations = 0;
};

/// RANSAC over AlignHorn on 3-pair samples (ORB-SLAM3 Sim3Solver). Pair i is X1[i] in camera-1
/// coordinates, observed at pixels1[i] (camera1, σ = sigmas1[i]), and X2[i] in camera-2
/// coordinates, observed at pixels2[i]. A hypothesis S_12 = AlignHorn(X2 sample, X1 sample) is
/// scored on all pairs: inlier when both reprojections are in front of their camera and
///   ‖(camera1.Project(S_12 X2) - pixel1) / σ1‖² < chi2  and  ‖(camera2.Project(S_12⁻¹ X1) - pixel2) / σ2‖² < chi2.
/// Best = most inliers, ties to the lower χ² sum. Iterations adapt as log(1 - confidence) /
/// log(1 - w³), w the best inlier ratio so far, capped at max_iterations. Deterministic for a
/// given seed. Success means at least min_inliers inliers; fewer than 3 pairs or mismatched sizes
/// fail immediately. The similarity is not refined (loop detection runs OptimizeSim3 next).
[[nodiscard]] Sim3RansacResult SolveSim3Ransac(const Camera& camera1, const Camera& camera2,
                                               const std::vector<TPoint3DF>& X1, const std::vector<TPoint2DF>& pixels1,
                                               const std::vector<double>& sigmas1, const std::vector<TPoint3DF>& X2,
                                               const std::vector<TPoint2DF>& pixels2,
                                               const std::vector<double>& sigmas2, const Sim3RansacOptions& options);

}  // namespace voyahslam::geometry

#endif  // VOYAHSLAM_GEOMETRY_SIM3_SOLVER_H
