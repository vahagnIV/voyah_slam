#include <voyahviz/scene_state.h>

#include <voyahslam/features/instantiation.h>

namespace voyahviz {

using voyahslam::events::EventType;

template <typename TFeatureExtractor>
void SceneState<TFeatureExtractor>::Apply(const voyahslam::events::Event& event) {
    // No default case: adding an EventType must produce a -Wswitch warning here.
    // Payload decoding (map points, keyframe poses, current pose) comes with the
    // Pangolin renderer; for now every event is only counted.
    switch (event.type) {
        case EventType::NewKeyFrameCandidate:
        case EventType::MapPointsAdded:
        case EventType::MapPointsUpdated:
        case EventType::MapPointsRemoved:
        case EventType::KeyFrameCreated:
        case EventType::KeyFramePoseUpdated:
        case EventType::LoopDetected:
        case EventType::PositionUpdated:
        case EventType::InitializationMeasurement:
        case EventType::MapCreated:
        case EventType::TrackingLost:
            Count(event.type);
            break;
    }
}

template <typename TFeatureExtractor>
size_t SceneState<TFeatureExtractor>::applied_count(EventType type) const {
    return applied_counts_[static_cast<size_t>(type)];
}

template <typename TFeatureExtractor>
void SceneState<TFeatureExtractor>::Count(EventType type) {
    ++applied_counts_[static_cast<size_t>(type)];
    ++total_applied_;
}

INSTANTIATE_CLASS(SceneState)

} // namespace voyahviz
