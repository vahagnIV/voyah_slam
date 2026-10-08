#ifndef VOYAHSLAM_TRACKING_MAP_REPLICA_H
#define VOYAHSLAM_TRACKING_MAP_REPLICA_H

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <map>
#include <optional>
#include <utility>
#include <variant>
#include <vector>
#include <voyahslam/bow/bow_vector.h>
#include <voyahslam/bow/frame_database.h>
#include <voyahslam/defs.h>
#include <voyahslam/events/event_types.h>
#include <voyahslam/events/map_transaction.h>
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
    // The Mapper's BoW data of the left descriptors; empty without a vocabulary.
    bow::BowVector bow;
    bow::FeatureVector features;
};

// The Tracker's private copy of the map, built only from Mapper events and
// touched only on the Tracker thread (no locks). Same map-id rule as the viewer's
// SceneState: map 0 is exempt (ignored), events of older maps are ignored, and an
// event of a newer map clears the replica and adopts the id.
//
// A MapTransaction is applied whole, in one Apply(), in the spec's order: new
// map, Frames, points, observations (each new Frame's own, then the added
// ones), updated points, poses, removed observations, removed points (the
// losers of replaced points too), removed Frames. An observation of an unknown
// Frame or of a point that is not alive is skipped.
//
// Point references. A point's reference is its first observation in the
// transaction that adds it. When the reference observation is removed, the
// remaining observation with the smallest Frame id takes over, with the level
// and distance of that Frame's feature and current pose. A point left without
// observations is removed.
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
    // The inverted word index of the Frames' BoW vectors (place recognition); Frames
    // without a vocabulary are not in it.
    [[nodiscard]] const bow::FrameDatabase& database() const { return database_; }

    // The invariant after every transaction (spec section 2): every
    // observation refers to an existing Frame and an alive point, the
    // per-Frame observations and covisibility() agree, and every alive point
    // has an observation, its reference among them. Linear in the number of
    // observations; for tests.
    [[nodiscard]] bool CheckConsistency() const;

private:
    // Applies the multi-map rule; returns false if the event must not be applied.
    bool AdmitMap(map::MapId map_id);
    void Clear();
    void ApplyTransaction(const events::MapTransaction<TFeatureExtractor>& transaction);
    // Sets point ids[i] from column i of the buffers; unknown ids are skipped.
    template <typename TPositions, typename TDescriptors, typename TNormals>
    void UpdatePoints(const std::vector<map::MapPointId>& ids, const TPositions& positions,
                      const TDescriptors& descriptors, const TNormals& normals);
    // Adds the observation unless its Frame is unknown, its point is not
    // alive, its feature is out of range or the Frame already observes the
    // point. A point without a reference takes it.
    void AddObservation(map::FrameId frame, uint32_t feature, map::MapPointId point);
    void RemoveObservation(map::FrameId frame, map::MapPointId point);
    // Drops the point and all its observations; a no-op unless it is alive.
    void RemovePoint(map::MapPointId point);
    // Drops the Frame and all its observations.
    void RemoveFrame(map::FrameId frame);
    // After `point` lost its observation by `frame`: removes the point if it
    // has none left, else moves its reference if it was that observation.
    void OnObservationRemoved(map::FrameId frame, map::MapPointId point);
    void SetReference(map::MapPointId point, const ReplicaKeyFrame<TFeatureExtractor>& keyframe,
                      uint32_t feature);

    map::MapId map_id_{0};
    std::vector<geometry::TPoint3DF> positions_;
    std::vector<TDescriptor> descriptors_;
    std::vector<geometry::TPoint3DF> normals_;
    std::vector<bool> alive_;
    std::vector<std::optional<map::PointReference>> references_;
    std::map<map::FrameId, ReplicaKeyFrame<TFeatureExtractor>> keyframes_;
    map::CovisibilityIndex covisibility_;
    bow::FrameDatabase database_;
};

namespace detail {

// The pyramid levels of the (left) features of `measurement`.
template <typename TMeasurement>
const features::TLevels& LeftLevels(const TMeasurement& measurement) {
    return std::visit(
        [](const auto& m) -> const features::TLevels& {
            if constexpr (requires { m.left(); }) return m.left().levels();
            else return m.levels();
        },
        measurement);
}

} // namespace detail

template <typename TFeatureExtractor>
void MapReplica<TFeatureExtractor>::Apply(const events::Event& event) {
    using events::EventType;
    if (!AdmitMap(event.map_id)) return;
    // No default case: adding an EventType must produce a -Wswitch warning here.
    switch (event.type) {
        case EventType::MapTransaction:
            ApplyTransaction(static_cast<const events::MapTransaction<TFeatureExtractor>&>(event));
            break;
        case EventType::FrameTracked:
        case EventType::TrackingLost:
        case EventType::LoopDetected:
        case EventType::InitializationMeasurement:
            break;
    }
}

template <typename TFeatureExtractor>
void MapReplica<TFeatureExtractor>::ApplyTransaction(
    const events::MapTransaction<TFeatureExtractor>& transaction) {
    // 1. New map: nothing of an earlier map of the same id survives.
    if (transaction.new_map) Clear();

    // 2. Frames; their observations follow the points.
    for (const events::NewFrame<TFeatureExtractor>& added : transaction.added_frames) {
        if (keyframes_.contains(added.id)) RemoveFrame(added.id);
        ReplicaKeyFrame<TFeatureExtractor>& keyframe = keyframes_[added.id];
        keyframe.id = added.id;
        keyframe.T_cw = added.T_cw;
        keyframe.measurement = added.measurement;
        keyframe.bow = added.bow;
        keyframe.features = added.features;
        database_.Add(added.id, added.bow);
        keyframe.observations.reserve(added.observed_features.size());
    }

    // 3. Points.
    const uint32_t num_new = transaction.num_new_points();
    const size_t end = static_cast<size_t>(transaction.new_points_start_id.value) + num_new;
    if (end > positions_.size()) {
        positions_.resize(end, geometry::TPoint3DF::Zero());
        descriptors_.resize(end, TDescriptor::Zero());
        normals_.resize(end, geometry::TPoint3DF::Zero());
        alive_.resize(end, false);
        references_.resize(end);
    }
    for (uint32_t i = 0; i < num_new; ++i) {
        const uint32_t id = transaction.new_points_start_id.value + i;
        positions_[id] = transaction.new_positions.col(i);
        descriptors_[id] = transaction.new_descriptors.col(i);
        normals_[id] = transaction.new_normals.col(i);
        alive_[id] = true;
        references_[id].reset();
    }

    // 4. Observations: the new Frames' own first, so they give new points
    // their references.
    for (const events::NewFrame<TFeatureExtractor>& added : transaction.added_frames) {
        const size_t num = std::min(added.observed_features.size(), added.observed_points.size());
        for (size_t i = 0; i < num; ++i) {
            AddObservation(added.id, added.observed_features[i], added.observed_points[i]);
        }
    }
    for (const events::TransactionObservation& added : transaction.added_observations) {
        AddObservation(added.frame, added.feature, added.point);
    }

    // 5. Updated points.
    UpdatePoints(transaction.updated_ids, transaction.updated_positions,
                 transaction.updated_descriptors, transaction.updated_normals);

    // 6. Poses.
    for (const auto& [id, T_cw] : transaction.updated_poses) {
        const auto it = keyframes_.find(id);
        if (it != keyframes_.end()) it->second.T_cw = T_cw;
    }

    // 7.-9. Removals.
    for (const events::RemovedObservation& removed : transaction.removed_observations) {
        RemoveObservation(removed.frame, removed.point);
    }
    for (const events::ReplacedPoint& replaced : transaction.replaced_points) {
        RemovePoint(replaced.loser);
    }
    for (const map::MapPointId point : transaction.removed_points) {
        RemovePoint(point);
    }
    for (const map::FrameId frame : transaction.removed_frames) {
        RemoveFrame(frame);
    }
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
void MapReplica<TFeatureExtractor>::AddObservation(map::FrameId frame, uint32_t feature,
                                                   map::MapPointId point) {
    const auto it = keyframes_.find(frame);
    if (it == keyframes_.end() || !IsAlive(point) || covisibility_.Contains(frame, point)) return;
    ReplicaKeyFrame<TFeatureExtractor>& keyframe = it->second;
    if (feature >= static_cast<uint32_t>(detail::LeftLevels(keyframe.measurement).size())) return;
    keyframe.observations.emplace_back(feature, point);
    covisibility_.Add(frame, feature, point);
    if (!references_[point.value]) SetReference(point, keyframe, feature);
}

template <typename TFeatureExtractor>
void MapReplica<TFeatureExtractor>::RemoveObservation(map::FrameId frame, map::MapPointId point) {
    if (!covisibility_.Contains(frame, point)) return;
    std::erase_if(keyframes_.at(frame).observations,
                  [&](const auto& observation) { return observation.second == point; });
    covisibility_.Remove(frame, point);
    OnObservationRemoved(frame, point);
}

template <typename TFeatureExtractor>
void MapReplica<TFeatureExtractor>::RemovePoint(map::MapPointId point) {
    if (!IsAlive(point)) return;
    for (const map::PointObservation& observation : covisibility_.Observations(point)) {
        std::erase_if(keyframes_.at(observation.frame).observations,
                      [&](const auto& o) { return o.second == point; });
    }
    covisibility_.RemovePoint(point);
    alive_[point.value] = false;
    references_[point.value].reset();
}

template <typename TFeatureExtractor>
void MapReplica<TFeatureExtractor>::RemoveFrame(map::FrameId frame) {
    const auto it = keyframes_.find(frame);
    if (it == keyframes_.end()) return;
    const std::vector<std::pair<uint32_t, map::MapPointId>> observations =
        std::move(it->second.observations);
    keyframes_.erase(it);
    database_.Remove(frame);
    covisibility_.RemoveFrame(frame);
    for (const auto& [feature, point] : observations) OnObservationRemoved(frame, point);
}

template <typename TFeatureExtractor>
void MapReplica<TFeatureExtractor>::OnObservationRemoved(map::FrameId frame, map::MapPointId point) {
    const std::vector<map::PointObservation>& remaining = covisibility_.Observations(point);
    if (remaining.empty()) {
        alive_[point.value] = false;
        references_[point.value].reset();
        return;
    }
    const std::optional<map::PointReference>& reference = references_[point.value];
    if (reference && reference->keyframe != frame) return;
    const auto next = std::min_element(
        remaining.begin(), remaining.end(),
        [](const map::PointObservation& a, const map::PointObservation& b) { return a.frame < b.frame; });
    SetReference(point, keyframes_.at(next->frame), next->feature);
}

template <typename TFeatureExtractor>
void MapReplica<TFeatureExtractor>::SetReference(map::MapPointId point,
                                                 const ReplicaKeyFrame<TFeatureExtractor>& keyframe,
                                                 uint32_t feature) {
    // A binocular keyframe's reference level is the left feature's level.
    const auto& levels = detail::LeftLevels(keyframe.measurement);
    const geometry::TPoint3DF centre = keyframe.T_cw.Inverse().translation();
    references_[point.value] = map::PointReference{
        keyframe.id, static_cast<Eigen::Index>(feature), static_cast<int>(levels(feature)),
        static_cast<double>((positions_[point.value] - centre).norm())};
}

template <typename TFeatureExtractor>
bool MapReplica<TFeatureExtractor>::CheckConsistency() const {
    // Frame side: alive points, each observation indexed with its feature.
    size_t num_observations = 0;
    for (const auto& [id, keyframe] : keyframes_) {
        if (covisibility_.Points(id).size() != keyframe.observations.size()) return false;
        for (const auto& [feature, point] : keyframe.observations) {
            if (!IsAlive(point)) return false;
            const std::vector<map::PointObservation>& indexed = covisibility_.Observations(point);
            if (std::none_of(indexed.begin(), indexed.end(), [&](const map::PointObservation& o) {
                    return o.frame == id && o.feature == feature;
                })) {
                return false;
            }
        }
        num_observations += keyframe.observations.size();
    }
    // Point side: known Frames only, an alive point observed, its reference
    // among its observations; a dead point unobserved.
    size_t num_indexed = 0;
    for (uint32_t p = 0; p < positions_.size(); ++p) {
        const std::vector<map::PointObservation>& indexed = covisibility_.Observations({p});
        num_indexed += indexed.size();
        if (!alive_[p]) {
            if (!indexed.empty() || references_[p]) return false;
            continue;
        }
        if (indexed.empty() || !references_[p]) return false;
        for (const map::PointObservation& o : indexed) {
            if (!keyframes_.contains(o.frame)) return false;
        }
        const map::PointReference& reference = *references_[p];
        if (std::none_of(indexed.begin(), indexed.end(), [&](const map::PointObservation& o) {
                return o.frame == reference.keyframe &&
                       static_cast<Eigen::Index>(o.feature) == reference.feature;
            })) {
            return false;
        }
    }
    return num_indexed == num_observations;
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
    database_.Clear();
}

} // namespace voyahslam::tracking

#endif // VOYAHSLAM_TRACKING_MAP_REPLICA_H
