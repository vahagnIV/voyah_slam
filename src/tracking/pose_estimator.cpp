#include <variant>
#include <voyahslam/features/instantiation.h>
#include <voyahslam/tracking/pose_estimator.h>

namespace voyahslam::tracking {

template <typename TFeatureExtractor>
PoseEstimate PoseEstimator<TFeatureExtractor>::Estimate(
    const features::TMeasurement<TFeatureExtractor>& measurement,
    const geometry::RigidTransformation& last_pose) const {
    const size_t num_features = std::visit(
        [](const auto& m) -> size_t {
            if constexpr (std::is_same_v<std::decay_t<decltype(m)>,
                                         features::TBinocularMeasurement<TFeatureExtractor>>) {
                return static_cast<size_t>(m.left().points().cols());
            } else {
                return static_cast<size_t>(m.points().cols());
            }
        },
        measurement);
    return PoseEstimate{num_features >= kMinTrackedFeatures, last_pose};
}

INSTANTIATE_CLASS(PoseEstimator)

} // namespace voyahslam::tracking
