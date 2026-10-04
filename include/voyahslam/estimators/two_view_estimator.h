#ifndef VOYAHSLAM_ESTIMATORS_TWO_VIEW_ESTIMATOR_H
#define VOYAHSLAM_ESTIMATORS_TWO_VIEW_ESTIMATOR_H

#include <concepts>
#include <cstddef>
#include <span>
#include <vector>

#include <Eigen/Dense>

#include "voyahslam/geometry/two_view_correspondences.h"

namespace voyahslam::estimators {

/// Result of scoring a model against a set of correspondences.
struct ModelScore {
    double score = 0.0;
    Eigen::Array<bool, Eigen::Dynamic, 1> inliers;
    size_t num_inliers = 0;
};

/// A minimal-sample model estimator usable by Ransac.
/// Estimate returns 0..n candidate models (0 = degenerate sample).
template <typename T>
concept TwoViewEstimator = requires(const T estimator,
                                    const geometry::TwoViewCorrespondences& correspondences,
                                    std::span<const Eigen::Index> sample,
                                    const typename T::TModel& model) {
    { T::kSampleSize } -> std::convertible_to<size_t>;
    { estimator.Estimate(correspondences, sample) } -> std::same_as<std::vector<typename T::TModel>>;
    { estimator.Score(model, correspondences) } -> std::same_as<ModelScore>;
};

} // namespace voyahslam::estimators

#endif // VOYAHSLAM_ESTIMATORS_TWO_VIEW_ESTIMATOR_H
