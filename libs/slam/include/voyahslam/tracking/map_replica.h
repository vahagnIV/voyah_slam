#ifndef VOYAHSLAM_TRACKING_MAP_REPLICA_H
#define VOYAHSLAM_TRACKING_MAP_REPLICA_H

#include <cstddef>
#include <cstdint>
#include <map>
#include <optional>
#include <utility>
#include <variant>
#include <vector>
#include <voyahslam/defs.h>
#include <voyahslam/events/events.h>
#include <voyahslam/features/measurement.h>
#include <voyahslam/geometry/rigid_transformation.h>
#include <voyahslam/map/covisibility_index.h>
#include <voyahslam/map/frame_id.h>
#include <voyahslam/map/map_id.h>
#include <voyahslam/map/map_point_id.h>
#include <voyahslam/map/point_reference.h>

namespace voyahslam::tracking {

template <typename TFeatureExtractor>
struct ReplicaKeyFrame {
    map::FrameId id{};
    geometry::RigidTransformation T_cw;  // world -> camera
    features::TMeasurement<TFeatureExtractor> measurement;
    std::vector<std::pair<uint32_t, map::MapPointId>> observations;  // (feature, point)
};

// The Tracker's private copy of the map, built only from Mapper events and
// touched only on the Tracker thread (no locks). Same map-id rule as the viewer's
// SceneState: map 0 is exempt (ignored), events of older maps are ignored, and an
// event of a newer map clears the replica and adopts the id.
//
// A KeyFrameCreated is applied whole, in one Apply(): new points, updated
// points, observations on existing Frames, then the Frame. Afterwards every
// observation refers to a known point and Frame, and the per-Frame
// observations and covisibility() agree.
template <typename TFeatureExtractor>
class MapReplica {
public:
    using TDescriptor = typename TFeatureExtractor::TDescriptor;

    void Apply(const events::Event& event);

    [[nodiscard]] map::MapId map_id() const { return map_id_; }
    // Number of ids issued; ids restart at 0 per map, so storage is dense.
    [[nodiscard]] size_t num_points() const { return positions_.size(); }
    [[nodiscard]] bool IsAlive(map::MapPointId id) const {
        return id.value < alive_.size() && alive_[id.value];
    }
    [[nodiscard]] const geometry::TPoint3DF& Position(map::MapPointId id) const {
        return positions_[id.value];
    }
    [[nodiscard]] TDescriptor Descriptor(map::MapPointId id) const { return descriptors_[id.value]; }
    [[nodiscard]] const geometry::TPoint3DF& Normal(map::MapPointId id) const {
        return normals_[id.value];
    }
    [[nodiscard]] const std::optional<map::PointReference>& Reference(map::MapPointId id) const {
        return references_[id.value];
    }
    // nullptr if unknown.
    [[nodiscard]] const ReplicaKeyFrame<TFeatureExtractor>* keyframe(map::FrameId id) const {
        const auto it = keyframes_.find(id);
        return it == keyframes_.end() ? nullptr : &it->second;
    }
    [[nodiscard]] std::vector<map::FrameId> keyframe_ids() const {
        std::vector<map::FrameId> ids;
        ids.reserve(keyframes_.size());
        for (const auto& [id, keyframe] : keyframes_) ids.push_back(id);
        return ids;
    }
    // The observations indexed per point and per Frame.
    [[nodiscard]] const map::CovisibilityIndex& covisibility() const { return covisibility_; }

private:
    // Applies the multi-map rule; returns false if the event must not be applied.
    bool AdmitMap(map::MapId map_id);
    void Clear();
    void ApplyKeyFrameCreated(const events::KeyFrameCreated<TFeatureExtractor>& created);
    // Sets point ids[i] from column i of the buffers; unknown ids are skipped.
    template <typename TPositions, typename TDescriptors, typename TNormals>
    void UpdatePoints(const std::vector<map::MapPointId>& ids, const TPositions& positions,
                      const TDescriptors& descriptors, const TNormals& normals);

    map::MapId map_id_{0};
    std::vector<geometry::TPoint3DF> positions_;
    std::vector<TDescriptor> descriptors_;
    std::vector<geometry::TPoint3DF> normals_;
    std::vector<bool> alive_;
    std::vector<std::optional<map::PointReference>> references_;
    std::map<map::FrameId, ReplicaKeyFrame<TFeatureExtractor>> keyframes_;
    map::CovisibilityIndex covisibility_;
};

template <typename TFeatureExtractor>
void MapReplica<TFeatureExtractor>::Apply(const events::Event& event) {
    using events::EventType;
    if (!AdmitMap(event.map_id)) return;
    // No default case: adding an EventType must produce a -Wswitch warning here.
    switch (event.type) {
        case EventType::MapPointsUpdated: {
            const auto& updated = static_cast<const events::MapPointsUpdated<TFeatureExtractor>&>(event);
            UpdatePoints(updated.ids, updated.positions, updated.descriptors, updated.normals);
            break;
        }
        case EventType::MapPointsRemoved: {
            for (const auto& id : static_cast<const events::MapPointsRemoved&>(event).ids) {
                if (id.value < alive_.size()) alive_[id.value] = false;
            }
            break;
        }
        case EventType::KeyFrameCreated:
            ApplyKeyFrameCreated(static_cast<const events::KeyFrameCreated<TFeatureExtractor>&>(event));
            break;
        case EventType::KeyFramePoseUpdated: {
            const auto& updated = static_cast<const events::KeyFramePoseUpdated&>(event);
            const auto it = keyframes_.find(updated.frame_id);
            if (it != keyframes_.end()) it->second.T_cw = updated.pose;
            break;
        }
        case EventType::MapCreated:
        case EventType::FrameTracked:
        case EventType::TrackingLost:
        case EventType::LoopDetected:
        case EventType::InitializationMeasurement:
            break;
    }
}

template <typename TFeatureExtractor>
void MapReplica<TFeatureExtractor>::ApplyKeyFrameCreated(
    const events::KeyFrameCreated<TFeatureExtractor>& created) {
    // New points.
    const uint32_t num_new = created.num_new_points();
    const size_t end = static_cast<size_t>(created.new_points_start_id.value) + num_new;
    if (end > positions_.size()) {
        positions_.resize(end, geometry::TPoint3DF::Zero());
        descriptors_.resize(end, TDescriptor::Zero());
        normals_.resize(end, geometry::TPoint3DF::Zero());
        alive_.resize(end, false);
        references_.resize(end);
    }
    for (uint32_t i = 0; i < num_new; ++i) {
        const uint32_t id = created.new_points_start_id.value + i;
        positions_[id] = created.new_positions.col(i);
        descriptors_[id] = created.new_descriptors.col(i);
        normals_[id] = created.new_normals.col(i);
        alive_[id] = true;
    }

    UpdatePoints(created.updated_ids, created.updated_positions, created.updated_descriptors,
                 created.updated_normals);

    // Observations on existing Frames; an unknown Frame or point is skipped.
    for (const events::AddedObservation& added : created.added_observations) {
        const auto it = keyframes_.find(added.frame_id);
        if (it == keyframes_.end() || added.point.value >= positions_.size()) continue;
        if (covisibility_.Contains(added.frame_id, added.point)) continue;
        it->second.observations.emplace_back(added.feature, added.point);
        covisibility_.Add(added.frame_id, added.feature, added.point);
    }

    // The Frame.
    ReplicaKeyFrame<TFeatureExtractor> keyframe;
    keyframe.id = created.frame_id;
    keyframe.T_cw = created.pose;
    keyframe.measurement = created.measurement;
    const size_t num = created.observed_features.size();
    keyframe.observations.reserve(num);
    for (size_t i = 0; i < num; ++i) {
        keyframe.observations.emplace_back(created.observed_features[i], created.observed_points[i]);
        covisibility_.Add(created.frame_id, created.observed_features[i], created.observed_points[i]);
    }

    // A point's reference is the first Frame whose own observations include
    // it. For a binocular keyframe the reference level is the left feature's level.
    const auto& levels = std::visit(
        [](const auto& measurement) -> const features::TLevels& {
            if constexpr (requires { measurement.left(); }) return measurement.left().levels();
            else return measurement.levels();
        },
        keyframe.measurement);
    const geometry::TPoint3DF centre = keyframe.T_cw.Inverse().translation();
    for (const auto& [feature, point] : keyframe.observations) {
        if (point.value >= positions_.size() || references_[point.value]) continue;
        references_[point.value] = map::PointReference{
            keyframe.id, static_cast<Eigen::Index>(feature), static_cast<int>(levels(feature)),
            static_cast<double>((positions_[point.value] - centre).norm())};
    }
    keyframes_.insert_or_assign(created.frame_id, std::move(keyframe));
}

template <typename TFeatureExtractor>
template <typename TPositions, typename TDescriptors, typename TNormals>
void MapReplica<TFeatureExtractor>::UpdatePoints(const std::vector<map::MapPointId>& ids,
                                                 const TPositions& positions,
                                                 const TDescriptors& descriptors,
                                                 const TNormals& normals) {
    for (size_t i = 0; i < ids.size(); ++i) {
        const uint32_t id = ids[i].value;
        if (id >= positions_.size()) continue;
        const auto col = static_cast<Eigen::Index>(i);
        positions_[id] = positions.col(col);
        descriptors_[id] = descriptors.col(col);
        normals_[id] = normals.col(col);
    }
}

template <typename TFeatureExtractor>
bool MapReplica<TFeatureExtractor>::AdmitMap(map::MapId map_id) {
    if (map_id.value == 0 || map_id < map_id_) return false;
    if (map_id > map_id_) {
        Clear();
        map_id_ = map_id;
    }
    return true;
}

template <typename TFeatureExtractor>
void MapReplica<TFeatureExtractor>::Clear() {
    positions_.clear();
    descriptors_.clear();
    normals_.clear();
    alive_.clear();
    references_.clear();
    keyframes_.clear();
    covisibility_.Clear();
}

} // namespace voyahslam::tracking

#endif // VOYAHSLAM_TRACKING_MAP_REPLICA_H
