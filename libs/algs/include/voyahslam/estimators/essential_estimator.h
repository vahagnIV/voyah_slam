#ifndef VOYAHSLAM_ESTIMATORS_ESSENTIAL_ESTIMATOR_H
#define VOYAHSLAM_ESTIMATORS_ESSENTIAL_ESTIMATOR_H

#include <cstddef>
#include <span>
#include <vector>

#include <Eigen/Dense>

#include "voyahslam/defs.h"
#include "voyahslam/estimators/two_view_estimator.h"
#include "voyahslam/geometry/two_view_correspondences.h"

namespace voyahslam::estimators {

/// Essential matrix E_21 (x2^T E x1 = 0, normalized coordinates) from 8 correspondences
/// (normalized 8-point algorithm projected onto the essential manifold),
/// scored like ORB-SLAM3 CheckFundamental (point-to-epipolar-line distance).
class EssentialEstimator {
public:
    using TModel = geometry::TMatrix33F;
    static constexpr size_t kSampleSize = 8;
    static constexpr double kChi2Threshold = 3.841;
    static constexpr double kScoreThreshold = 5.991;

    /// Returns one model, or none for a degenerate sample.
    [[nodiscard]] std::vector<TModel> Estimate(const geometry::TwoViewCorrespondences& a_correspondences,
                                               std::span<const Eigen::Index> a_sample) const;

    [[nodiscard]] ModelScore Score(const TModel& a_model,
                                   const geometry::TwoViewCorrespondences& a_correspondences) const;

    /// Levenberg-Marquardt refinement of E = [t]x R over the inliers (5 dof: rotation and unit
    /// translation direction), minimizing the Sampson error. Returns E with unit Frobenius norm,
    /// or a_model unchanged if there are fewer than kSampleSize inliers or the result is non-finite.
    [[nodiscard]] TModel Refine(const TModel& a_model, const geometry::TwoViewCorrespondences& a_correspondences,
                                const Eigen::Array<bool, Eigen::Dynamic, 1>& a_inliers) const;
};

} // namespace voyahslam::estimators

#endif // VOYAHSLAM_ESTIMATORS_ESSENTIAL_ESTIMATOR_H
