#ifndef VOYAHSLAM_MAPPING_MAP_INITIALIZER_H
#define VOYAHSLAM_MAPPING_MAP_INITIALIZER_H

#include <cstddef>
#include <optional>
#include <vector>
#include <voyahslam/defs.h>
#include <voyahslam/features/measurement.h>
#include <voyahslam/geometry/rigid_transformation.h>

namespace voyahslam::mapping {

template <typename TFeatureExtractor>
struct InitialKeyFrame {
    geometry::RigidTransformation T_cw;
    features::TMeasurement<TFeatureExtractor> measurement;
};

// Rule-based placeholder; the geometry spec replaces the bodies in place.
//   - binocular: initializes at once with one identity-pose keyframe;
//   - monocular: the first measurement (or one with too few features) becomes
//     the reference; once the time since the reference reaches
//     min_mono_init_interval, both become identity-pose keyframes.
template <typename TFeatureExtractor>
class MapInitializer {
public:
    using TMeasurement = features::TMeasurement<TFeatureExtractor>;

    static constexpr size_t kMinInitFeatures = 100;

    explicit MapInitializer(double min_mono_init_interval)
        : min_mono_init_interval_(min_mono_init_interval) {}

    std::optional<std::vector<InitialKeyFrame<TFeatureExtractor>>> TryInitialize(
        TMeasurement measurement);

    // Drops the reference measurement.
    void Reset();

private:
    double min_mono_init_interval_;
    std::optional<TMeasurement> reference_;
};

} // namespace voyahslam::mapping

#endif // VOYAHSLAM_MAPPING_MAP_INITIALIZER_H
