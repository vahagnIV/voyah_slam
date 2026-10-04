#ifndef VOYAHVIZ_SCENE_STATE_H
#define VOYAHVIZ_SCENE_STATE_H

#include <array>
#include <cstddef>
#include <cstdint>
#include <map>
#include <optional>
#include <vector>
#include <voyahslam/events/event_types.h>
#include <voyahslam/geometry/rigid_transformation.h>
#include <voyahslam/map/frame_id.h>
#include <voyahslam/map/map_id.h>

namespace voyahviz {

enum class TrackingStatus : uint8_t { None, Tracking, Lost };

// What the map viewer knows, built only from SLAM events (never from Slam or
// its components), so the same code can later be fed by a deserializer on a
// remote machine. No threads, no GUI: only the viewer thread touches it.
//
// It decodes map points, keyframe poses (T_cw), the current camera pose and its
// trajectory for the newest map seen; events of older maps are ignored and an
// event of a newer map clears the scene first. Events with map 0 are counted only.
template <typename TFeatureExtractor>
class SceneState {
public:
    void Apply(const voyahslam::events::Event& event);

    [[nodiscard]] size_t applied_count(voyahslam::events::EventType type) const;
    [[nodiscard]] size_t total_applied() const { return total_applied_; }

    [[nodiscard]] voyahslam::map::MapId current_map_id() const { return current_map_id_; }
    // Indexed by MapPointId::value; ids restart at 0 per map, so storage is dense.
    [[nodiscard]] const std::vector<voyahslam::geometry::TPoint3DF>& point_positions() const {
        return point_positions_;
    }
    [[nodiscard]] const std::vector<bool>& point_alive() const { return point_alive_; }
    [[nodiscard]] size_t num_alive_points() const { return num_alive_points_; }
    // Keyframe poses are T_cw (world -> camera).
    [[nodiscard]] const std::map<voyahslam::map::FrameId, voyahslam::geometry::RigidTransformation>&
    keyframes() const {
        return keyframes_;
    }
    [[nodiscard]] const std::optional<voyahslam::geometry::RigidTransformation>& current_pose() const {
        return current_pose_;
    }
    [[nodiscard]] TrackingStatus status() const { return status_; }
    // Camera centres of the current map, in the order PositionUpdated arrived.
    [[nodiscard]] const std::vector<voyahslam::geometry::TPoint3DF>& trajectory() const {
        return trajectory_;
    }

private:
    static constexpr size_t kNumEventTypes =
        static_cast<size_t>(voyahslam::events::EventType::TrackingLost) + 1;

    void Count(voyahslam::events::EventType type);
    // Applies the multi-map rule; returns false if the event must not be applied.
    bool AdmitMap(voyahslam::map::MapId map_id);
    void ClearScene();
    void SetPoint(uint32_t id, const voyahslam::geometry::TPoint3DF& position);

    std::array<size_t, kNumEventTypes> applied_counts_{};
    size_t total_applied_ = 0;

    voyahslam::map::MapId current_map_id_{0};
    std::vector<voyahslam::geometry::TPoint3DF> point_positions_;
    std::vector<bool> point_alive_;
    size_t num_alive_points_ = 0;
    std::map<voyahslam::map::FrameId, voyahslam::geometry::RigidTransformation> keyframes_;
    std::optional<voyahslam::geometry::RigidTransformation> current_pose_;
    TrackingStatus status_ = TrackingStatus::None;
    std::vector<voyahslam::geometry::TPoint3DF> trajectory_;
};

} // namespace voyahviz

#endif // VOYAHVIZ_SCENE_STATE_H
