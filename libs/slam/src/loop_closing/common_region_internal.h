#ifndef VOYAHSLAM_LOOP_CLOSING_COMMON_REGION_INTERNAL_H
#define VOYAHSLAM_LOOP_CLOSING_COMMON_REGION_INTERNAL_H

#include <cmath>
#include <optional>
#include <variant>
#include <vector>
#include <voyahslam/features/measurement.h>
#include <voyahslam/geometry/rigid_transformation.h>
#include <voyahslam/geometry/sim3.h>
#include <voyahslam/map/map_point_id.h>
#include <voyahslam/tracking/map_replica.h>

// Helpers shared by the common-region detection and the LoopCloser.
namespace voyahslam::loop_closing::detail {

using PointMatches = std::vector<std::optional<map::MapPointId>>;

// Stereo maps are metric: their similarities keep scale 1.
template <typename TFeatureExtractor>
bool FixesScale(const tracking::ReplicaKeyFrame<TFeatureExtractor>& keyframe) {
    return std::holds_alternative<features::TBinocularMeasurement<TFeatureExtractor>>(keyframe.measurement);
}

// Review Focus 4: a finite similarity with a positive scale.
inline bool IsValid(const geometry::Sim3& S) {
    return std::isfinite(S.scale) && S.scale > 0.0 && S.R.allFinite() && S.t.allFinite();
}

inline geometry::Sim3 ToSim3(const geometry::RigidTransformation& T) {
    return {1.0, T.rotation(), T.translation()};
}

// Per (left) feature of `keyframe`, the alive point it observes.
template <typename TFeatureExtractor>
PointMatches PointsOf(const tracking::MapReplica<TFeatureExtractor>& replica,
                      const tracking::ReplicaKeyFrame<TFeatureExtractor>& keyframe) {
    PointMatches points(
        static_cast<size_t>(features::LeftOf<TFeatureExtractor>(keyframe.measurement).points().cols()));
    for (const auto& [feature, point] : keyframe.observations) {
        if (feature < points.size() && replica.IsAlive(point)) {
            points[feature] = point;
        }
    }
    return points;
}

} // namespace voyahslam::loop_closing::detail

#endif // VOYAHSLAM_LOOP_CLOSING_COMMON_REGION_INTERNAL_H
