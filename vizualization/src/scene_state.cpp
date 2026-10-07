#include <voyahviz/scene_state.h>

#include <voyahslam/events/events.h>
#include <voyahslam/features/instantiation.h>

namespace voyahviz {

namespace events = voyahslam::events;
using voyahslam::events::EventType;

template <typename TFeatureExtractor>
void SceneState<TFeatureExtractor>::Apply(const voyahslam::events::Event& event) {
    Count(event.type);
    if (!AdmitMap(event.map_id)) return;
    // No default case: adding an EventType must produce a -Wswitch warning here.
    switch (event.type) {
        case EventType::MapTransaction:
            ApplyTransaction(static_cast<const events::MapTransaction<TFeatureExtractor>&>(event));
            break;
        case EventType::FrameTracked: {
            const auto& tracked = static_cast<const events::FrameTracked<TFeatureExtractor>&>(event);
            current_pose_ = tracked.pose;
            status_ = TrackingStatus::Tracking;
            trajectory_.push_back(tracked.pose.Inverse().translation());
            tracked_points_ = tracked.observed_points;
            visible_points_ = tracked.visible_points;
            local_frames_ = tracked.local_frames;
            break;
        }
        case EventType::TrackingLost:
            status_ = TrackingStatus::Lost;
            tracked_points_.clear();
            visible_points_.clear();
            local_frames_.clear();
            break;
        case EventType::LoopDetected:
        case EventType::InitializationMeasurement:
            break;
    }
}

template <typename TFeatureExtractor>
void SceneState<TFeatureExtractor>::ApplyTransaction(
    const voyahslam::events::MapTransaction<TFeatureExtractor>& transaction) {
    if (transaction.new_map) {
        ClearScene();
    }
    for (const auto& frame : transaction.added_frames) {
        keyframes_.insert_or_assign(frame.id, frame.T_cw);
    }
    const uint32_t start = transaction.new_points_start_id.value;
    const size_t end = static_cast<size_t>(start) + transaction.num_new_points();
    if (end > point_positions_.size()) {
        point_positions_.resize(end, voyahslam::geometry::TPoint3DF::Zero());
        point_alive_.resize(end, false);
    }
    for (uint32_t i = 0; i < transaction.num_new_points(); ++i) {
        SetPoint(start + i, transaction.new_positions.col(i));
    }
    UpdatePositions(transaction.updated_ids, transaction.updated_positions);
    for (const auto& [id, T_cw] : transaction.updated_poses) {
        const auto it = keyframes_.find(id);
        if (it != keyframes_.end()) {
            it->second = T_cw;
        }
    }
    for (const auto& replaced : transaction.replaced_points) {
        RemovePoint(replaced.loser.value);
    }
    for (const auto& id : transaction.removed_points) {
        RemovePoint(id.value);
    }
    for (const auto& id : transaction.removed_frames) {
        keyframes_.erase(id);
    }
    if (transaction.new_map && transaction.start_frame) {
        const auto it = keyframes_.find(*transaction.start_frame);
        if (it != keyframes_.end()) {
            current_pose_ = it->second;
            status_ = TrackingStatus::Tracking;
        }
    }
}

template <typename TFeatureExtractor>
bool SceneState<TFeatureExtractor>::AdmitMap(voyahslam::map::MapId map_id) {
    if (map_id.value == 0 || map_id < current_map_id_) return false;
    if (map_id > current_map_id_) {
        ClearScene();
        current_map_id_ = map_id;
    }
    return true;
}

template <typename TFeatureExtractor>
void SceneState<TFeatureExtractor>::ClearScene() {
    point_positions_.clear();
    point_alive_.clear();
    num_alive_points_ = 0;
    keyframes_.clear();
    trajectory_.clear();
    tracked_points_.clear();
    visible_points_.clear();
    local_frames_.clear();
    current_pose_.reset();
    status_ = TrackingStatus::None;
}

template <typename TFeatureExtractor>
void SceneState<TFeatureExtractor>::SetPoint(uint32_t id,
                                             const voyahslam::geometry::TPoint3DF& position) {
    point_positions_[id] = position;
    if (!point_alive_[id]) {
        point_alive_[id] = true;
        ++num_alive_points_;
    }
}

template <typename TFeatureExtractor>
void SceneState<TFeatureExtractor>::RemovePoint(uint32_t id) {
    if (id < point_alive_.size() && point_alive_[id]) {
        point_alive_[id] = false;
        --num_alive_points_;
    }
}

template <typename TFeatureExtractor>
void SceneState<TFeatureExtractor>::UpdatePositions(
    const std::vector<voyahslam::map::MapPointId>& ids,
    const voyahslam::memory::ColumnBuffer<voyahslam::precision_t, 3>& positions) {
    for (size_t i = 0; i < ids.size(); ++i) {
        const uint32_t id = ids[i].value;
        if (id < point_positions_.size()) {
            point_positions_[id] = positions.col(static_cast<Eigen::Index>(i));
        }
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
