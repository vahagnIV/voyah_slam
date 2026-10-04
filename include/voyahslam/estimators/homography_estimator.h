#ifndef VOYAHSLAM_ESTIMATORS_HOMOGRAPHY_ESTIMATOR_H
#define VOYAHSLAM_ESTIMATORS_HOMOGRAPHY_ESTIMATOR_H

#include <cstddef>
#include <span>
#include <vector>

#include <Eigen/Dense>

#include "voyahslam/defs.h"
#include "voyahslam/estimators/two_view_estimator.h"
#include "voyahslam/geometry/two_view_correspondences.h"

namespace voyahslam::estimators {

/// Planar homography H_21 (x2 ~ H x1) from 4 correspondences (normalized DLT),
/// scored like ORB-SLAM3 CheckHomography (symmetric transfer error).
class HomographyEstimator {
public:
    using TModel = geometry::TMatrix33F;
    static constexpr size_t kSampleSize = 4;
    static constexpr double kChi2Threshold = 5.991;

    /// Returns one model, or none for a degenerate sample.
    [[nodiscard]] std::vector<TModel> Estimate(const geometry::TwoViewCorrespondences& a_correspondences,
                                               std::span<const Eigen::Index> a_sample) const;

    [[nodiscard]] ModelScore Score(const TModel& a_model,
                                   const geometry::TwoViewCorrespondences& a_correspondences) const;

    /// Levenberg-Marquardt refinement of H over the inliers (8 dof: ‖H‖ = 1 with an update
    /// orthogonal to H), minimizing the symmetric transfer error. Returns a_model unchanged if
    /// there are fewer than kSampleSize inliers or the result is non-finite or ill-conditioned.
    [[nodiscard]] TModel Refine(const TModel& a_model, const geometry::TwoViewCorrespondences& a_correspondences,
                                const Eigen::Array<bool, Eigen::Dynamic, 1>& a_inliers) const;
};

} // namespace voyahslam::estimators

#endif // VOYAHSLAM_ESTIMATORS_HOMOGRAPHY_ESTIMATOR_H
