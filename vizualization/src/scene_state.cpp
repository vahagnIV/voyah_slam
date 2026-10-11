#include <voyahviz/scene_state.h>

#include <voyahslam/events/events.h>
#include <voyahslam/features/instantiation.h>
#include <voyahslam/map/absorbed_point_ids.h>

namespace voyahviz {

namespace events = voyahslam::events;
using voyahslam::events::EventType;

template <typename TFeatureExtractor>
void SceneState<TFeatureExtractor>::Apply(const voyahslam::events::Event& event) {
    Count(event.type);
    if (event.type == EventType::MapTransaction) {
        ApplyTransaction(static_cast<const events::MapTransaction<TFeatureExtractor>&>(event));
        return;
    }
    // The camera follows the active map only (map 0's events are counted only).
    if (!active_map_ || event.map_id != *active_map_) return;
    // No default case: adding an EventType must produce a -Wswitch warning here.
    switch (event.type) {
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
            status_ = static_cast<const events::TrackingLost&>(event).final
                          ? TrackingStatus::Lost
                          : TrackingStatus::RecentlyLost;
            tracked_points_.clear();
            visible_points_.clear();
            local_frames_.clear();
            break;
        case EventType::MapTransaction:
        case EventType::LoopDetected:
        case EventType::InitializationMeasurement:
        case EventType::GlobalBAFinished:
            break;
    }
}

template <typename TFeatureExtractor>
void SceneState<TFeatureExtractor>::ApplyTransaction(
    const voyahslam::events::MapTransaction<TFeatureExtractor>& transaction) {
    // The replica rule (tracking::AtlasReplica's).
    const voyahslam::map::MapId id = transaction.map_id;
    if (id.value == 0 || removed_maps_.contains(id)) return;
    auto it = maps_.find(id);
    if (it == maps_.end()) {
        if (!transaction.new_map) return;
        it = maps_.try_emplace(id).first;
    }
    if (transaction.new_map) {
        it->second = {};
    }
    // A merge: the absorbed map moves in first (it is in removed_maps).
    if (transaction.absorbed_map && *transaction.absorbed_map != id) {
        const auto absorbed = maps_.find(*transaction.absorbed_map);
        if (absorbed != maps_.end()) {
            it->second.Absorb(std::move(absorbed->second));
        }
    }
    ApplyParts(transaction, it->second);
    for (const voyahslam::map::MapId removed : transaction.removed_maps) {
        maps_.erase(removed);
        removed_maps_.insert(removed);
    }
    const std::optional<voyahslam::map::MapId> previous = active_map_;
    active_map_ = transaction.active_map;
    if (active_map_ && !maps_.contains(*active_map_)) {
        active_map_.reset();
    }
    if (active_map_ != previous) {
        // Another map: nothing of the camera carries over; with none left
        // active the status stays (a final loss shows Lost).
        ClearTracking();
        if (active_map_) status_ = TrackingStatus::None;
    }
    if (transaction.new_map && transaction.start_frame && active_map_ == id) {
        const auto frame = it->second.keyframes.find(*transaction.start_frame);
        if (frame != it->second.keyframes.end()) {
            current_pose_ = frame->second;
            status_ = TrackingStatus::Tracking;
        }
    }
}

template <typename TFeatureExtractor>
void SceneState<TFeatureExtractor>::ApplyParts(
    const voyahslam::events::MapTransaction<TFeatureExtractor>& transaction, MapScene& out_scene) {
    for (const auto& frame : transaction.added_frames) {
        out_scene.keyframes.insert_or_assign(frame.id, frame.T_cw);
    }
    const uint32_t start = transaction.new_points_start_id.value;
    const size_t end = static_cast<size_t>(start) + transaction.num_new_points();
    if (end > out_scene.point_positions.size()) {
        out_scene.point_positions.resize(end, voyahslam::geometry::TPoint3DF::Zero());
        out_scene.point_alive.resize(end, false);
    }
    for (uint32_t i = 0; i < transaction.num_new_points(); ++i) {
        out_scene.SetPoint(start + i, transaction.new_positions.col(i));
    }
    out_scene.UpdatePositions(transaction.updated_ids, transaction.updated_positions);
    for (const auto& [id, T_cw] : transaction.updated_poses) {
        const auto it = out_scene.keyframes.find(id);
        if (it != out_scene.keyframes.end()) {
            it->second = T_cw;
        }
    }
    for (const auto& replaced : transaction.replaced_points) {
        out_scene.RemovePoint(replaced.loser.value);
    }
    for (const auto& id : transaction.removed_points) {
        out_scene.RemovePoint(id.value);
    }
    for (const auto& id : transaction.removed_frames) {
        out_scene.keyframes.erase(id);
    }
}

template <typename TFeatureExtractor>
std::vector<voyahslam::map::MapId> SceneState<TFeatureExtractor>::map_ids() const {
    std::vector<voyahslam::map::MapId> ids;
    ids.reserve(maps_.size());
    for (const auto& [id, scene] : maps_) ids.push_back(id);
    return ids;
}

template <typename TFeatureExtractor>
auto SceneState<TFeatureExtractor>::Shown() const -> const MapScene& {
    if (!active_map_) return kNoScene;
    const auto it = maps_.find(*active_map_);
    return it == maps_.end() ? kNoScene : it->second;
}

template <typename TFeatureExtractor>
void SceneState<TFeatureExtractor>::ClearTracking() {
    trajectory_.clear();
    tracked_points_.clear();
    visible_points_.clear();
    local_frames_.clear();
    current_pose_.reset();
}

template <typename TFeatureExtractor>
void SceneState<TFeatureExtractor>::MapScene::SetPoint(uint32_t id,
                                                       const voyahslam::geometry::TPoint3DF& position) {
    point_positions[id] = position;
    if (!point_alive[id]) {
        point_alive[id] = true;
        ++num_alive_points;
    }
}

template <typename TFeatureExtractor>
void SceneState<TFeatureExtractor>::MapScene::Absorb(MapScene&& absorbed) {
    const std::vector<std::optional<voyahslam::map::MapPointId>> new_ids = voyahslam::map::AbsorbedPointIds(
        static_cast<uint32_t>(absorbed.point_alive.size()),
        voyahslam::map::MapPointId{static_cast<uint32_t>(point_alive.size())},
        [&](voyahslam::map::MapPointId point) { return static_cast<bool>(absorbed.point_alive[point.value]); });
    for (uint32_t p = 0; p < new_ids.size(); ++p) {
        if (!new_ids[p]) continue;
        const uint32_t id = new_ids[p]->value;
        if (id >= point_alive.size()) {
            point_positions.resize(id + 1, voyahslam::geometry::TPoint3DF::Zero());
            point_alive.resize(id + 1, false);
        }
        SetPoint(id, absorbed.point_positions[p]);
    }
    keyframes.merge(absorbed.keyframes);
    absorbed = {};
}

template <typename TFeatureExtractor>
void SceneState<TFeatureExtractor>::MapScene::RemovePoint(uint32_t id) {
    if (id < point_alive.size() && point_alive[id]) {
        point_alive[id] = false;
        --num_alive_points;
    }
}

template <typename TFeatureExtractor>
void SceneState<TFeatureExtractor>::MapScene::UpdatePositions(
    const std::vector<voyahslam::map::MapPointId>& ids,
    const voyahslam::memory::ColumnBuffer<voyahslam::precision_t, 3>& positions) {
    for (size_t i = 0; i < ids.size(); ++i) {
        const uint32_t id = ids[i].value;
        if (id < point_positions.size()) {
            point_positions[id] = positions.col(static_cast<Eigen::Index>(i));
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
