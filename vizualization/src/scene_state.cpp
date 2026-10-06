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
        case EventType::MapPointsUpdated: {
            const auto& updated =
                static_cast<const events::MapPointsUpdated<TFeatureExtractor>&>(event);
            UpdatePositions(updated.ids, updated.positions);
            break;
        }
        case EventType::MapPointsRemoved: {
            const auto& removed = static_cast<const events::MapPointsRemoved&>(event);
            for (const auto& id : removed.ids) {
                if (id.value < point_alive_.size() && point_alive_[id.value]) {
                    point_alive_[id.value] = false;
                    --num_alive_points_;
                }
            }
            break;
        }
        case EventType::KeyFrameCreated: {
            const auto& created =
                static_cast<const events::KeyFrameCreated<TFeatureExtractor>&>(event);
            const uint32_t start = created.new_points_start_id.value;
            const size_t end = static_cast<size_t>(start) + created.num_new_points();
            if (end > point_positions_.size()) {
                point_positions_.resize(end, voyahslam::geometry::TPoint3DF::Zero());
                point_alive_.resize(end, false);
            }
            for (uint32_t i = 0; i < created.num_new_points(); ++i) {
                SetPoint(start + i, created.new_positions.col(i));
            }
            UpdatePositions(created.updated_ids, created.updated_positions);
            keyframes_.insert_or_assign(created.frame_id, created.pose);
            break;
        }
        case EventType::KeyFramePoseUpdated: {
            const auto& updated = static_cast<const events::KeyFramePoseUpdated&>(event);
            keyframes_.insert_or_assign(updated.frame_id, updated.pose);
            break;
        }
        case EventType::MapCreated: {
            current_pose_ = static_cast<const events::MapCreated&>(event).pose;
            status_ = TrackingStatus::Tracking;
            break;
        }
        case EventType::FrameTracked: {
            const auto& tracked = static_cast<const events::FrameTracked<TFeatureExtractor>&>(event);
            current_pose_ = tracked.pose;
            status_ = TrackingStatus::Tracking;
            trajectory_.push_back(tracked.pose.Inverse().translation());
            break;
        }
        case EventType::TrackingLost:
            status_ = TrackingStatus::Lost;
            break;
        case EventType::LoopDetected:
        case EventType::InitializationMeasurement:
            break;
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
