#ifndef VOYAHSLAM_MAPPING_MAPPER_INTERNAL_H
#define VOYAHSLAM_MAPPING_MAPPER_INTERNAL_H

#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <variant>
#include <voyahslam/features/measurement.h>
#include <voyahslam/features/stereo_rig.h>
#include <voyahslam/map/frame_id.h>
#include <voyahslam/map/map_point_id.h>
#include <voyahslam/optimization/bundle_adjustment.h>

// Helpers shared by the Mapper's translation units (the BA observation
// builders also by the global BA, global_ba/global_ba.cpp).
namespace voyahslam::mapping::detail {

using features::LeftOf;

// The binocular measurement, nullptr for a monocular one.
template <typename TFeatureExtractor>
const features::TBinocularMeasurement<TFeatureExtractor>* BinocularOf(
    const features::TMeasurement<TFeatureExtractor>& measurement) {
    return std::get_if<features::TBinocularMeasurement<TFeatureExtractor>>(&measurement);
}

// A Frame's stereo rig for BA (right camera, T_rl); none for a monocular
// measurement.
template <typename TFeatureExtractor>
std::optional<optimization::StereoCameras> BaStereoOf(
    const features::TMeasurement<TFeatureExtractor>& measurement) {
    const auto* binocular = BinocularOf<TFeatureExtractor>(measurement);
    if (binocular == nullptr) return std::nullopt;
    return optimization::StereoCameras{binocular->right().camera(), binocular->rig().T_rl};
}

// The BA observation of feature `feature` of a Frame's measurement, by the
// Frame at index `keyframe` of point column `point`: the distorted left pixel,
// sigma_px * level_scale^level, and the right x of a stereo-matched feature.
template <typename TFeatureExtractor>
optimization::BaObservation BaObservationOf(const features::TMeasurement<TFeatureExtractor>& measurement,
                                            uint32_t feature, size_t keyframe, size_t point,
                                            double sigma_px, double level_scale) {
    const auto& left = LeftOf<TFeatureExtractor>(measurement);
    const auto f = static_cast<Eigen::Index>(feature);
    optimization::BaObservation observation{
        keyframe, point, left.distorted_points().col(f),
        sigma_px * std::pow(level_scale, static_cast<double>(left.levels()(f)))};
    if (const auto* binocular = BinocularOf<TFeatureExtractor>(measurement)) {
        const int right = binocular->right_indices()(f);
        if (right >= 0) {
            observation.right_x = binocular->right().distorted_points()(0, right);
        }
    }
    return observation;
}

// The distance between a rig's two cameras.
inline double RigBaseline(const features::StereoRig& rig) {
    return static_cast<double>(rig.T_rl.translation().norm());
}

// A key of the observation of `point` by `frame`.
inline uint64_t ObservationKey(map::FrameId frame, map::MapPointId point) {
    return (static_cast<uint64_t>(frame.value) << 32) | point.value;
}

// Milliseconds elapsed since `start`.
inline double MillisecondsSince(std::chrono::steady_clock::time_point start) {
    return std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start)
        .count();
}

} // namespace voyahslam::mapping::detail

#endif // VOYAHSLAM_MAPPING_MAPPER_INTERNAL_H
