#ifndef VOYAHSLAM_TRACKING_ATLAS_REPLICA_H
#define VOYAHSLAM_TRACKING_ATLAS_REPLICA_H

#include <cassert>
#include <map>
#include <optional>
#include <set>
#include <unordered_map>
#include <utility>
#include <vector>
#include <voyahslam/events/map_transaction.h>
#include <voyahslam/map/absorbed_point_ids.h>
#include <voyahslam/map/map_id.h>
#include <voyahslam/map/map_point_id.h>
#include <voyahslam/tracking/map_replica.h>

namespace voyahslam::tracking {

// A private copy of the Mapper's Atlas: every live map by id, and which one is
// active. Built only from the Mapper's transactions and touched only on its
// owner's thread (no locks). The replica rule (spec section 1), per
// transaction of map `map_id`:
// - map 0 is no map: ignored;
// - new_map creates map `map_id` (or clears it), unless it was removed;
// - any other transaction applies to the live map `map_id`; one naming an
//   unknown or removed map is ignored, Atlas parts included;
// - then the maps in removed_maps are dropped, and their ids remembered:
//   they are never admitted again;
// - then active_map is taken: the active map comes only from it (none if it
//   names no live map), never from the newest id.
// A merge transaction (absorbed_map set, map merging spec section 3) first
// moves the absorbed map's Frames, observations and points into map `map_id`
// by the id rule (map::AbsorbedPointIds, as the Mapper applies it), then
// applies the transaction to that map; the absorbed map is in removed_maps,
// so it is dropped after. A merge whose absorbed map is not live absorbs
// nothing.
template <typename TFeatureExtractor>
class AtlasReplica {
public:
    using TMap = MapReplica<TFeatureExtractor>;

    // Applies `transaction` by the replica rule; true iff it was admitted
    // (not ignored).
    bool Apply(const events::MapTransaction<TFeatureExtractor>& transaction);

    [[nodiscard]] std::optional<map::MapId> active_map() const { return active_map_; }
    // The active map; null if none.
    [[nodiscard]] const TMap* active() const { return active_map_ ? map(*active_map_) : nullptr; }
    // The active map, or an empty map if none: for holders that work on the
    // active map only and have nothing to do without one.
    [[nodiscard]] const TMap& active_or_empty() const {
        const TMap* found = active();
        return found != nullptr ? *found : kEmpty;
    }
    // Map `id`; null if unknown or removed.
    [[nodiscard]] const TMap* map(map::MapId id) const {
        const auto it = maps_.find(id);
        return it == maps_.end() ? nullptr : &it->second;
    }
    // The live maps, ascending.
    [[nodiscard]] std::vector<map::MapId> map_ids() const {
        std::vector<map::MapId> ids;
        ids.reserve(maps_.size());
        for (const auto& [id, replica] : maps_) ids.push_back(id);
        return ids;
    }

    // Point `old` of map `absorbed` after the merge that absorbed it: its id in
    // the map that absorbed it (the id rule), then the point it was fused into
    // by that merge (replaced_points). nullopt if `absorbed` was not absorbed
    // by a merge this replica applied, if `old` was not alive then, or if the
    // point is not alive now (Task 6: the Tracker's references become
    // unmatched).
    [[nodiscard]] std::optional<map::MapPointId> TranslatePoint(map::MapId absorbed, map::MapPointId old) const;

private:
    // A merge applied: the map that absorbed the map, the new id per old id,
    // and the merge's fusions (loser -> final winner).
    struct Absorption {
        map::MapId target;
        std::vector<std::optional<map::MapPointId>> new_ids;
        std::unordered_map<uint32_t, map::MapPointId> replaced;
    };

    inline static const TMap kEmpty;

    std::map<map::MapId, TMap> maps_;  // node-based: a map's address is stable
    std::set<map::MapId> removed_;
    std::optional<map::MapId> active_map_;
    std::map<map::MapId, Absorption> absorptions_;  // by absorbed map
};

template <typename TFeatureExtractor>
bool AtlasReplica<TFeatureExtractor>::Apply(const events::MapTransaction<TFeatureExtractor>& transaction) {
    const map::MapId id = transaction.map_id;
    if (id.value == 0 || removed_.contains(id)) {
        return false;
    }
    auto it = maps_.find(id);
    if (it == maps_.end()) {
        if (!transaction.new_map) {
            return false;
        }
        it = maps_.try_emplace(id).first;
    }
    if (transaction.absorbed_map && *transaction.absorbed_map != id) {
        const auto absorbed = maps_.find(*transaction.absorbed_map);
        if (absorbed != maps_.end()) {
            TMap& source = absorbed->second;
            Absorption& absorption = absorptions_[*transaction.absorbed_map];
            absorption.target = id;
            absorption.new_ids = map::AbsorbedPointIds(
                static_cast<uint32_t>(source.num_points()), map::MapPointId{static_cast<uint32_t>(it->second.num_points())},
                [&](map::MapPointId point) { return source.IsAlive(point); });
            for (const events::ReplacedPoint& replaced : transaction.replaced_points) {
                absorption.replaced.emplace(replaced.loser.value, replaced.winner);
            }
            it->second.Absorb(std::move(source), absorption.new_ids);
        }
    }
    it->second.Apply(transaction);
    for (const map::MapId removed : transaction.removed_maps) {
        maps_.erase(removed);
        removed_.insert(removed);
    }
    active_map_ = transaction.active_map;
    if (active_map_ && !maps_.contains(*active_map_)) {
        active_map_.reset();
    }
    return true;
}

template <typename TFeatureExtractor>
std::optional<map::MapPointId> AtlasReplica<TFeatureExtractor>::TranslatePoint(map::MapId absorbed,
                                                                               map::MapPointId old) const {
    const auto it = absorptions_.find(absorbed);
    if (it == absorptions_.end() || old.value >= it->second.new_ids.size() || !it->second.new_ids[old.value]) {
        return std::nullopt;
    }
    const Absorption& absorption = it->second;
    map::MapPointId point = *absorption.new_ids[old.value];
    // The merge lists each loser with its final winner.
    if (const auto replaced = absorption.replaced.find(point.value); replaced != absorption.replaced.end()) {
        point = replaced->second;
    }
    const TMap* target = map(absorption.target);
    if (target == nullptr || !target->IsAlive(point)) {
        return std::nullopt;
    }
    return point;
}

} // namespace voyahslam::tracking

#endif // VOYAHSLAM_TRACKING_ATLAS_REPLICA_H
