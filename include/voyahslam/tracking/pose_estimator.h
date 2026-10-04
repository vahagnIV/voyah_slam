#ifndef VOYAHSLAM_TRACKING_POSE_ESTIMATOR_H
#define VOYAHSLAM_TRACKING_POSE_ESTIMATOR_H

#include <cstddef>
#include <voyahslam/defs.h>
#include <voyahslam/features/measurement.h>
#include <voyahslam/geometry/rigid_transformation.h>

namespace voyahslam::tracking {

struct PoseEstimate {
    bool success;
    geometry::RigidTransformation pose;
};

// Rule-based placeholder; the geometry spec replaces the body in place.
// Succeeds iff the measurement (binocular: left camera) has at least
// kMinTrackedFeatures features; the pose is the last pose.
template <typename TFeatureExtractor>
class PoseEstimator {
public:
    static constexpr size_t kMinTrackedFeatures = 50;

    [[nodiscard]] PoseEstimate Estimate(const features::TMeasurement<TFeatureExtractor>& measurement,
                                        const geometry::RigidTransformation& last_pose) const;
};

} // namespace voyahslam::tracking

#endif // VOYAHSLAM_TRACKING_POSE_ESTIMATOR_H
