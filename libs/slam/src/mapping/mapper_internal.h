#ifndef VOYAHSLAM_MAPPING_MAPPER_INTERNAL_H
#define VOYAHSLAM_MAPPING_MAPPER_INTERNAL_H

#include <cstdint>
#include <variant>
#include <voyahslam/features/measurement.h>
#include <voyahslam/features/stereo_rig.h>
#include <voyahslam/map/frame_id.h>
#include <voyahslam/map/map_point_id.h>

// Helpers shared by the Mapper's translation units.
namespace voyahslam::mapping::detail {

// The left image of a measurement (the image itself for a monocular one).
template <typename TFeatureExtractor>
const features::TMonocularMeasurement<TFeatureExtractor>& LeftOf(
    const features::TMeasurement<TFeatureExtractor>& measurement) {
    if (const auto* binocular =
            std::get_if<features::TBinocularMeasurement<TFeatureExtractor>>(&measurement)) {
        return binocular->left();
    }
    return std::get<features::TMonocularMeasurement<TFeatureExtractor>>(measurement);
}

// The binocular measurement, nullptr for a monocular one.
template <typename TFeatureExtractor>
const features::TBinocularMeasurement<TFeatureExtractor>* BinocularOf(
    const features::TMeasurement<TFeatureExtractor>& measurement) {
    return std::get_if<features::TBinocularMeasurement<TFeatureExtractor>>(&measurement);
}

// The distance between a rig's two cameras.
inline double RigBaseline(const features::StereoRig& rig) {
    return static_cast<double>(rig.T_rl.translation().norm());
}

// A key of the observation of `point` by `frame`.
inline uint64_t ObservationKey(map::FrameId frame, map::MapPointId point) {
    return (static_cast<uint64_t>(frame.value) << 32) | point.value;
}

} // namespace voyahslam::mapping::detail

#endif // VOYAHSLAM_MAPPING_MAPPER_INTERNAL_H
