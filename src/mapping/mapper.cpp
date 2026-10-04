#include <utility>
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

    // initial->map_points are not stored yet: the Mapper has no map points.
    for (auto& keyframe : initial->keyframes) {
        keyframes_.emplace_back(next_frame_id_, keyframe.T_cw, std::move(keyframe.measurement));
        ++next_frame_id_.value;
    }
    num_keyframes_.store(keyframes_.size(), std::memory_order_release);
    current_map_id_.store(next_map_id_, std::memory_order_release);
    ++next_map_id_.value;

    // The state changes before the events that announce it are published, so
    // anyone who observes MapCreated also observes state() == Mapping.
    SetState(MapperState::Mapping);
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
    num_keyframes_.store(0, std::memory_order_release);
    initializer_.Reset();
    SetState(MapperState::Initializing);
}

template <typename TFeatureExtractor>
void Mapper<TFeatureExtractor>::SetState(MapperState state) {
    const MapperState previous = state_.exchange(state, std::memory_order_acq_rel);
    log_->info("{} -> {} (map {}, {} keyframes)", ToString(previous), ToString(state),
               current_map_id().value, keyframes_.size());
}

template <typename TFeatureExtractor>
void Mapper<TFeatureExtractor>::PublishKeyFrameCreated(const TKeyFrame& keyframe) {
    auto* event = key_frame_created_pool_.Create();
    event->map_id = current_map_id();
    event->frame_id = keyframe.id();
    event->pose = keyframe.pose();
    event->measurement = keyframe.measurement();
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
