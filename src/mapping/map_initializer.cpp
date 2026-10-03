#include <utility>
#include <variant>
#include <voyahslam/features/instantiation.h>
#include <voyahslam/mapping/map_initializer.h>

namespace voyahslam::mapping {

namespace {

template <typename TMeasurement>
size_t FeatureCount(const TMeasurement& measurement) {
    return std::visit(
        [](const auto& m) -> size_t {
            if constexpr (requires { m.left(); }) {
                return static_cast<size_t>(m.left().points().cols());
            } else {
                return static_cast<size_t>(m.points().cols());
            }
        },
        measurement);
}

template <typename TMeasurement>
double Timestamp(const TMeasurement& measurement) {
    return std::visit([](const auto& m) { return m.timestamp(); }, measurement);
}

} // namespace

template <typename TFeatureExtractor>
std::optional<std::vector<InitialKeyFrame<TFeatureExtractor>>>
MapInitializer<TFeatureExtractor>::TryInitialize(TMeasurement measurement) {
    using TKeyFrames = std::vector<InitialKeyFrame<TFeatureExtractor>>;

    if (std::holds_alternative<features::TBinocularMeasurement<TFeatureExtractor>>(measurement)) {
        reference_.reset();
        TKeyFrames key_frames;
        key_frames.push_back({geometry::TPose::Identity(), std::move(measurement)});
        return key_frames;
    }

    if (!reference_.has_value() || FeatureCount(*reference_) < kMinInitFeatures ||
        FeatureCount(measurement) < kMinInitFeatures) {
        reference_ = std::move(measurement);
        return std::nullopt;
    }

    if (Timestamp(measurement) - Timestamp(*reference_) < min_mono_init_interval_) {
        return std::nullopt;
    }

    TKeyFrames key_frames;
    key_frames.push_back({geometry::TPose::Identity(), std::move(*reference_)});
    key_frames.push_back({geometry::TPose::Identity(), std::move(measurement)});
    reference_.reset();
    return key_frames;
}

template <typename TFeatureExtractor>
void MapInitializer<TFeatureExtractor>::Reset() {
    reference_.reset();
}

INSTANTIATE_CLASS(MapInitializer)

} // namespace voyahslam::mapping
