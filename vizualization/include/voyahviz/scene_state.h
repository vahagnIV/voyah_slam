#ifndef VOYAHVIZ_SCENE_STATE_H
#define VOYAHVIZ_SCENE_STATE_H

#include <array>
#include <cstddef>
#include <cstdint>
#include <map>
#include <optional>
#include <vector>
#include <voyahslam/defs.h>
#include <voyahslam/events/event_types.h>
#include <voyahslam/events/map_transaction.h>
#include <voyahslam/geometry/rigid_transformation.h>
#include <voyahslam/map/frame_id.h>
#include <voyahslam/map/map_id.h>
#include <voyahslam/map/map_point_id.h>
#include <voyahslam/memory/column_buffer.h>

namespace voyahviz {

// RecentlyLost: a non-final TrackingLost (the Tracker tries to relocalize in
// the map, which the scene keeps); Lost: a final one.
enum class TrackingStatus : uint8_t { None, Tracking, RecentlyLost, Lost };

// What the map viewer knows, built only from SLAM events (never from Slam or
// its components), so the same code can later be fed by a deserializer on a
// remote machine. No threads, no GUI: only the viewer thread touches it.
//
// It decodes map points, keyframe poses (T_cw), the current camera pose and its
// trajectory for the newest map seen; events of older maps are ignored and an
// event of a newer map clears the scene first. Events with map 0 are counted
// only. Of a MapTransaction it applies every part it holds: points and Frames
// are added, moved and dropped. It keeps no observations, so it relies on the
// Mapper listing a point left without observations in removed_points.
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
    // Camera centres of the current map, in the order FrameTracked arrived.
    [[nodiscard]] const std::vector<voyahslam::geometry::TPoint3DF>& trajectory() const {
        return trajectory_;
    }

    // From the last FrameTracked; cleared on a map switch and on TrackingLost.
    // Points matched in the current frame.
    [[nodiscard]] const std::vector<voyahslam::map::MapPointId>& tracked_points() const {
        return tracked_points_;
    }
    // Local-map points that pass the projection gate, matched or not.
    [[nodiscard]] const std::vector<voyahslam::map::MapPointId>& visible_points() const {
        return visible_points_;
    }
    [[nodiscard]] const std::vector<voyahslam::map::FrameId>& local_frames() const {
        return local_frames_;
    }

private:
    static constexpr size_t kNumEventTypes =
        static_cast<size_t>(voyahslam::events::EventType::TrackingLost) + 1;

    void Count(voyahslam::events::EventType type);
    // Applies the multi-map rule; returns false if the event must not be applied.
    bool AdmitMap(voyahslam::map::MapId map_id);
    void ClearScene();
    void ApplyTransaction(const voyahslam::events::MapTransaction<TFeatureExtractor>& transaction);
    void SetPoint(uint32_t id, const voyahslam::geometry::TPoint3DF& position);
    // Marks the point dead; an unknown or dead point is skipped.
    void RemovePoint(uint32_t id);
    // Moves point ids[i] to column i of `positions`; unknown ids are skipped.
    void UpdatePositions(const std::vector<voyahslam::map::MapPointId>& ids,
                         const voyahslam::memory::ColumnBuffer<voyahslam::precision_t, 3>& positions);

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
    std::vector<voyahslam::map::MapPointId> tracked_points_;
    std::vector<voyahslam::map::MapPointId> visible_points_;
    std::vector<voyahslam::map::FrameId> local_frames_;
};

} // namespace voyahviz

#endif // VOYAHVIZ_SCENE_STATE_H
