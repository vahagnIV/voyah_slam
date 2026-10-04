#ifndef VOYAHSLAM_GEOMETRY_TWO_VIEW_RECONSTRUCTION_H
#define VOYAHSLAM_GEOMETRY_TWO_VIEW_RECONSTRUCTION_H

#include <cstddef>
#include <span>

#include <Eigen/Dense>

#include "voyahslam/defs.h"
#include "voyahslam/estimators/ransac.h"
#include "voyahslam/geometry/rigid_transformation.h"
#include "voyahslam/geometry/two_view_correspondences.h"

namespace voyahslam::geometry {

enum class TwoViewModel { Homography, Essential };

struct TwoViewOptions {
    double homography_inlier_ratio = 0.8;      ///< Pick H if inliers_H >= this x inliers_E.
    double min_parallax_deg = 1.0;
    size_t min_triangulated = 50;
    double min_good_fraction = 0.9;            ///< Of the model's inliers.
    double essential_uniqueness = 0.7;         ///< Other candidates must have <= this x best good count.
    double homography_uniqueness = 0.75;
    double max_point_parallax_cos = 0.99998;   ///< A point needs parallax cos below this to count.
    estimators::RansacOptions ransac;
};

struct TwoViewReconstruction {
    bool success = false;
    TwoViewModel model = TwoViewModel::Essential;
    RigidTransformation T_21;                  ///< Camera 1 -> camera 2; ‖t‖ = 1.
    TPoints3DF points;                         ///< 3xN in camera 1's frame (valid where triangulated).
    Eigen::Array<bool, Eigen::Dynamic, 1> triangulated;
    size_t num_triangulated = 0;
    double parallax_deg = 0.0;
    double homography_score = 0.0;
    double essential_score = 0.0;
};

struct PoseSelection {
    bool success = false;
    size_t best_index = 0;
    TPoints3DF points;                         ///< 3xN in camera 1's frame (valid where triangulated).
    Eigen::Array<bool, Eigen::Dynamic, 1> triangulated;
    size_t num_good = 0;
    double parallax_deg = 0.0;
};

/// Picks the motion candidate (camera 1 -> camera 2) that places the most inliers in front of
/// both cameras with a reprojection error within 4σ² (ORB-SLAM3 CheckRT), and accepts it when it
/// explains enough inliers, is unambiguous (no other candidate has more than
/// a_uniqueness_ratio x its good count) and has enough parallax.
[[nodiscard]] PoseSelection SelectPose(std::span<const RigidTransformation> a_candidates,
                                       const TwoViewCorrespondences& a_correspondences,
                                       const Eigen::Array<bool, Eigen::Dynamic, 1>& a_inliers,
                                       double a_uniqueness_ratio, const TwoViewOptions& a_options);

/// Monocular initialization from two views (ORB-SLAM3 TwoViewReconstruction, normalized
/// coordinates): fits a homography and an essential matrix, picks the homography when it
/// explains nearly as many inliers (COLMAP rule), decomposes it and selects the pose with SelectPose.
class TwoViewReconstructor {
public:
    explicit TwoViewReconstructor(TwoViewOptions a_options = {});

    /// Fails for a non-positive sigma, when both RANSAC runs fail, or when pose selection fails.
    [[nodiscard]] TwoViewReconstruction Reconstruct(const TwoViewCorrespondences& a_correspondences) const;

private:
    TwoViewOptions options_;
};

} // namespace voyahslam::geometry

#endif // VOYAHSLAM_GEOMETRY_TWO_VIEW_RECONSTRUCTION_H
