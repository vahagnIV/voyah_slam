#include <cassert>
#include <utility>
#include <variant>
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
            return;  // NewKeyFrameCandidate, LoopDetected: handled later
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
    AddInitialMapPoints(initial->map_points);
    num_keyframes_.store(keyframes_.size(), std::memory_order_release);
    num_map_points_.store(map_points_.size(), std::memory_order_release);
    current_map_id_.store(next_map_id_, std::memory_order_release);
    ++next_map_id_.value;

    // The state changes before the events that announce it are published, so
    // anyone who observes MapCreated also observes state() == Mapping.
    SetState(MapperState::Mapping);
    if (map_points_.size() > 0) {
        PublishMapPointsAdded();
    }
    for (const TKeyFrame& keyframe : keyframes_) {
        PublishKeyFrameCreated(keyframe);
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
    // Map points come only from a monocular two-view initialization.
    assert(keyframes_.size() == 2);
    using TMonocularMeasurement = features::TMonocularMeasurement<TFeatureExtractor>;
    const auto& first = std::get<TMonocularMeasurement>(keyframes_[0].measurement());
    const geometry::TVector3DF centre_first = keyframes_[0].pose().Inverse().translation();
    const geometry::TVector3DF centre_second = keyframes_[1].pose().Inverse().translation();

    for (const InitialMapPoint& point : map_points) {
        assert(point.feature_in_first < 65536 && point.feature_in_second < 65536);
        const geometry::TPoint3DF normal = ((point.position - centre_first).normalized() +
                                            (point.position - centre_second).normalized())
                                               .normalized();
        const map::MapPointId id =
            map_points_.Add(point.position, first.descriptors().col(point.feature_in_first), normal);
        observations_.Add(id, keyframes_[0].id(), static_cast<uint16_t>(point.feature_in_first));
        observations_.Add(id, keyframes_[1].id(), static_cast<uint16_t>(point.feature_in_second));
    }
}

template <typename TFeatureExtractor>
void Mapper<TFeatureExtractor>::PublishMapPointsAdded() {
    // Fresh storage: the ids are exactly [0, size()).
    const uint32_t count = map_points_.size();
    auto* event = map_points_added_pool_.Create();
    event->map_id = current_map_id();
    event->start_id = map::MapPointId{0};
    event->Resize(count);
    for (uint32_t i = 0; i < count; ++i) {
        const map::MapPointId id{i};
        event->positions.col(i) = map_points_.Position(id);
        event->descriptors.col(i) = map_points_.Descriptor(id);
        event->normals.col(i) = map_points_.Normal(id);
    }
    observable_.Notify(event);
}

template <typename TFeatureExtractor>
void Mapper<TFeatureExtractor>::PublishKeyFrameCreated(const TKeyFrame& keyframe) {
    auto* event = key_frame_created_pool_.Create();
    event->map_id = current_map_id();
    event->frame_id = keyframe.id();
    event->pose = keyframe.pose();
    event->measurement = keyframe.measurement();
    for (size_t i = 0; i < observations_.size(); ++i) {
        if (observations_.frame_id(i) == keyframe.id()) {
            event->observed_features.push_back(observations_.feature_index(i));
            event->observed_points.push_back(observations_.mappoint_id(i));
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
