#include <cassert>
#include <cstdint>
#include <utility>
#include <variant>
#include <vector>
#include <voyahslam/features/instantiation.h>
#include <voyahslam/mapping/mapper.h>

namespace voyahslam::mapping {
namespace {

const char* ToString(MapperState state) {
    switch (state) {
        case MapperState::Initializing: return "Initializing";
        case MapperState::Mapping: return "Mapping";
    }
    return "?";
}

} // namespace

template <typename TFeatureExtractor>
Mapper<TFeatureExtractor>::Mapper(MapperOptions options)
    : initializer_(std::move(options.initializer)) {}

template <typename TFeatureExtractor>
Mapper<TFeatureExtractor>::~Mapper() {
    worker_.Stop();
}

template <typename TFeatureExtractor>
void Mapper<TFeatureExtractor>::Start() {
    worker_.Start([this] { return ProcessPending(); });
}

template <typename TFeatureExtractor>
void Mapper<TFeatureExtractor>::Stop() {
    worker_.Stop();
}

template <typename TFeatureExtractor>
bool Mapper<TFeatureExtractor>::ProcessPending() {
    bool processed = false;
    while (events::Event* event = inbox_listener_.GetNextEvent()) {
        processed = true;
        ApplyEvent(*event);
    }
    return processed;
}

template <typename TFeatureExtractor>
void Mapper<TFeatureExtractor>::ApplyEvent(const events::Event& event) {
    switch (event.type) {
        case events::EventType::InitializationMeasurement:
            OnInitializationMeasurement(
                static_cast<const events::InitializationMeasurement<TFeatureExtractor>&>(event));
            return;
        case events::EventType::TrackingLost:
            OnTrackingLost(static_cast<const events::TrackingLost&>(event));
            return;
        default:
            return;  // FrameTracked, LoopDetected: handled later
    }
}

template <typename TFeatureExtractor>
void Mapper<TFeatureExtractor>::OnInitializationMeasurement(
    const events::InitializationMeasurement<TFeatureExtractor>& event) {
    if (state() != MapperState::Initializing) {
        return;
    }
    // A copy: the event is shared with the other consumers.
    auto initial = initializer_.TryInitialize(TMeasurement(event.measurement));
    if (!initial || initial->keyframes.empty()) {
        return;
    }

    for (auto& keyframe : initial->keyframes) {
        keyframes_.emplace_back(next_frame_id_, keyframe.T_cw, std::move(keyframe.measurement));
        ++next_frame_id_.value;
    }
    map_points_ = {};
    observations_ = {};
    stereo_observations_ = {};
    AddInitialMapPoints(initial->map_points);
    num_keyframes_.store(keyframes_.size(), std::memory_order_release);
    num_map_points_.store(map_points_.size(), std::memory_order_release);
    current_map_id_.store(next_map_id_, std::memory_order_release);
    ++next_map_id_.value;

    // The state changes before the events that announce it are published, so
    // anyone who observes MapCreated also observes state() == Mapping.
    SetState(MapperState::Mapping);
    // The first keyframe carries every initial point; a monocular second
    // keyframe only its observations.
    for (size_t i = 0; i < keyframes_.size(); ++i) {
        PublishKeyFrameCreated(keyframes_[i], i == 0);
    }
    PublishMapCreated(keyframes_.back());
}

template <typename TFeatureExtractor>
void Mapper<TFeatureExtractor>::OnTrackingLost(const events::TrackingLost& event) {
    if (event.map_id != current_map_id() || state() != MapperState::Mapping) {
        return;  // stale: an older map
    }
    keyframes_.clear();
    map_points_ = {};
    observations_ = {};
    stereo_observations_ = {};
    num_keyframes_.store(0, std::memory_order_release);
    num_map_points_.store(0, std::memory_order_release);
    initializer_.Reset();
    SetState(MapperState::Initializing);
}

template <typename TFeatureExtractor>
void Mapper<TFeatureExtractor>::SetState(MapperState state) {
    const MapperState previous = state_.exchange(state, std::memory_order_acq_rel);
    log_->info("{} -> {} (map {}, {} keyframes, {} points)", ToString(previous), ToString(state),
               current_map_id().value, keyframes_.size(), map_points_.size());
}

template <typename TFeatureExtractor>
void Mapper<TFeatureExtractor>::AddInitialMapPoints(
    const std::vector<InitialMapPoint>& map_points) {
    if (map_points.empty()) {
        return;
    }
    using TMonocularMeasurement = features::TMonocularMeasurement<TFeatureExtractor>;
    using TBinocularMeasurement = features::TBinocularMeasurement<TFeatureExtractor>;
    const TKeyFrame& first = keyframes_[0];
    const auto* binocular = std::get_if<TBinocularMeasurement>(&first.measurement());
    // Monocular: a two-view initialization; binocular: one stereo keyframe.
    assert(keyframes_.size() == (binocular ? 1u : 2u));
    const auto& descriptors = binocular
                                  ? binocular->left().descriptors()
                                  : std::get<TMonocularMeasurement>(first.measurement()).descriptors();
    std::vector<geometry::TVector3DF> centres;
    for (const TKeyFrame& keyframe : keyframes_) {
        centres.push_back(keyframe.pose().Inverse().translation());
    }

    for (const InitialMapPoint& point : map_points) {
        assert(point.feature_in_first >= 0 && point.feature_in_first < 65536);
        // The normalized mean of the viewing directions of the keyframes.
        geometry::TPoint3DF normal = geometry::TPoint3DF::Zero();
        for (const geometry::TVector3DF& centre : centres) {
            normal += (point.position - centre).normalized();
        }
        normal.normalize();
        const map::MapPointId id =
            map_points_.Add(point.position, descriptors.col(point.feature_in_first), normal);
        const auto feature = static_cast<uint16_t>(point.feature_in_first);
        if (binocular) {
            const auto i = static_cast<int>(point.feature_in_first);
            const int right = binocular->right_indices()(i);
            assert(right <= 32767);
            stereo_observations_.Add(id, first.id(), feature, binocular->depth(i),
                                     static_cast<int16_t>(right));
        } else {
            assert(point.feature_in_second >= 0 && point.feature_in_second < 65536);
            observations_.Add(id, first.id(), feature);
            observations_.Add(id, keyframes_[1].id(),
                              static_cast<uint16_t>(point.feature_in_second));
        }
    }
}

template <typename TFeatureExtractor>
void Mapper<TFeatureExtractor>::PublishKeyFrameCreated(const TKeyFrame& keyframe,
                                                       bool with_new_points) {
    auto* event = key_frame_created_pool_.Create();
    event->map_id = current_map_id();
    event->frame_id = keyframe.id();
    event->pose = keyframe.pose();
    event->measurement = keyframe.measurement();
    // (left) feature index, point id; a stereo keyframe's depths and right
    // indices are in its measurement.
    const auto add_observations = [&](const auto& observations) {
        for (size_t i = 0; i < observations.size(); ++i) {
            if (observations.frame_id(i) == keyframe.id()) {
                event->observed_features.push_back(observations.feature_index(i));
                event->observed_points.push_back(observations.mappoint_id(i));
            }
        }
    };
    add_observations(observations_);
    add_observations(stereo_observations_);
    if (with_new_points) {
        // Fresh storage of a new map: the ids are exactly [0, num_ids()).
        const uint32_t count = map_points_.num_ids();
        event->new_points_start_id = map::MapPointId{0};
        event->ResizeNewPoints(count);
        for (uint32_t i = 0; i < count; ++i) {
            const map::MapPointId id{i};
            event->new_positions.col(i) = map_points_.Position(id);
            event->new_descriptors.col(i) = map_points_.Descriptor(id);
            event->new_normals.col(i) = map_points_.Normal(id);
        }
    }
    observable_.Notify(event);
}

template <typename TFeatureExtractor>
void Mapper<TFeatureExtractor>::PublishMapCreated(const TKeyFrame& keyframe) {
    auto* event = map_created_pool_.Create();
    event->map_id = current_map_id();
    event->frame_id = keyframe.id();
    event->pose = keyframe.pose();
    observable_.Notify(event);
}

INSTANTIATE_CLASS(Mapper)

} // namespace voyahslam::mapping
