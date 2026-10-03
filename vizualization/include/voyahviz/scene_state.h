#ifndef VOYAHVIZ_SCENE_STATE_H
#define VOYAHVIZ_SCENE_STATE_H

#include <array>
#include <cstddef>
#include <voyahslam/events/event_types.h>

namespace voyahviz {

// What the map viewer knows, built only from SLAM events (never from Slam or
// its components), so the same code can later be fed by a deserializer on a
// remote machine. No threads, no GUI: only the viewer thread touches it.
//
// For now it only recognizes and counts events; map points, keyframe poses and
// the current pose are decoded when the Pangolin renderer arrives.
template <typename TFeatureExtractor>
class SceneState {
public:
    void Apply(const voyahslam::events::Event& event);

    [[nodiscard]] size_t applied_count(voyahslam::events::EventType type) const;
    [[nodiscard]] size_t total_applied() const { return total_applied_; }

private:
    static constexpr size_t kNumEventTypes =
        static_cast<size_t>(voyahslam::events::EventType::TrackingLost) + 1;

    void Count(voyahslam::events::EventType type);

    std::array<size_t, kNumEventTypes> applied_counts_{};
    size_t total_applied_ = 0;
};

} // namespace voyahviz

#endif // VOYAHVIZ_SCENE_STATE_H
