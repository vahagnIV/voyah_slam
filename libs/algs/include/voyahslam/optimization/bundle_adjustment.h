#ifndef VOYAHSLAM_OPTIMIZATION_BUNDLE_ADJUSTMENT_H
#define VOYAHSLAM_OPTIMIZATION_BUNDLE_ADJUSTMENT_H

#include <cstddef>
#include <functional>
#include <optional>
#include <vector>

#include <Eigen/Dense>

#include "voyahslam/defs.h"
#include "voyahslam/geometry/camera.h"
#include "voyahslam/geometry/rigid_transformation.h"
#include "voyahslam/optimization/pose_optimization.h"

namespace voyahslam::optimization {

/// Two stages like ORB-SLAM3's local BA: robust_iterations with the Huber kernel, then the
/// edges with chi2 above their threshold (huber_chi2 mono, stereo_chi2 stereo) or depth <= 0
/// are deactivated and iterations more run without a kernel. With robust == false: one plain
/// pass of robust_iterations + iterations.
struct BundleAdjustmentOptions {
    int robust_iterations = 5;    ///< Stage 0 (Huber kernel).
    int iterations = 10;          ///< Stage 1 (no kernel, outlier edges deactivated).
    bool robust = true;
    double huber_chi2 = 5.991;    ///< Mono kernel delta^2 and outlier test (2 dof, 95 %).
    double stereo_chi2 = 7.815;   ///< The same for stereo (u_l, v_l, u_r) (3 dof, 95 %).
    /// Interruption: evaluated once before the first iteration and after every LM iteration
    /// (on the calling thread); once it returns true nothing more is optimized and the result
    /// is interrupted. Empty = never stop.
    std::function<bool()> should_stop;
};

struct BaObservation {
    size_t keyframe;                    ///< Index into keyframe_poses.
    size_t point;                       ///< Column of points.
    geometry::TPoint2DF measurement;    ///< Raw (distorted) pixel in the keyframe's camera.
    double sigma;                       ///< Pixels (sigma_px * scale^level).
    /// Raw x in the keyframe's right image (stereo match); needs keyframe_stereo[keyframe].
    std::optional<double> right_x = std::nullopt;
};

struct BaProblem {
    std::vector<geometry::RigidTransformation> keyframe_poses;   ///< T_cw.
    std::vector<bool> keyframe_fixed;
    std::vector<geometry::Camera> keyframe_cameras;              ///< One per keyframe.
    geometry::TPoints3DF points;                                 ///< World frame.
    std::vector<BaObservation> observations;
    /// Per keyframe, its stereo rig (right camera, T_rl); empty = all mono.
    std::vector<std::optional<StereoCameras>> keyframe_stereo;
};

struct BaResult {
    bool success = false;
    std::vector<geometry::RigidTransformation> keyframe_poses;
    geometry::TPoints3DF points;
    std::vector<double> chi2;      ///< Per observation (all, incl. deactivated), final, no kernel.
    /// chi2 <= huber_chi2 (mono) / stereo_chi2 (stereo) && depth > 0 (stereo: in both cameras).
    std::vector<bool> inlier;
    double initial_cost = 0.0;     ///< Sum of the plain chi2 over all observations at the start.
    double final_cost = 0.0;       ///< Sum of the plain chi2 over the edges of the last pass.
    int iterations = 0;            ///< Total over both stages.
    bool interrupted = false;      ///< should_stop returned true.
    /// The stage that was running when interrupted (0 robust, before the start included;
    /// 1 plain, also the single pass of robust == false); 2 = finished.
    int stage_reached = 2;
};

/// Jointly refines keyframe poses (T_cw) and world points by minimizing the raw-pixel
/// reprojection error e = camera.Project(T_cw X) - measurement of the observations
/// (Levenberg-Marquardt, points marginalized), each through its keyframe's camera; a stereo
/// observation (right_x set) adds the row right.Project(T_rl T_cw X).x() - right_x.
/// Fixed keyframes keep their input pose. An interrupted run still returns the estimates
/// reached so far (success true if finite). Invalid input (no keyframes, points or
/// observations; keyframe_fixed, keyframe_cameras or a non-empty keyframe_stereo of the wrong
/// size; a non-finite pose, point, measurement, right_x, K or rig; sigma <= 0; an
/// out-of-range index; right_x on a keyframe without a rig) yields success == false without
/// optimizing.
[[nodiscard]] BaResult BundleAdjust(const BaProblem& problem,
                                    const BundleAdjustmentOptions& options);

}  // namespace voyahslam::optimization

#endif  // VOYAHSLAM_OPTIMIZATION_BUNDLE_ADJUSTMENT_H
