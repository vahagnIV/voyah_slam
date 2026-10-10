#ifndef VOYAHSLAM_TRACKING_ATLAS_REPLICA_H
#define VOYAHSLAM_TRACKING_ATLAS_REPLICA_H

#include <cassert>
#include <map>
#include <optional>
#include <set>
#include <vector>
#include <voyahslam/events/map_transaction.h>
#include <voyahslam/map/map_id.h>
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
// absorbed_map (map merging) must be unset.
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

private:
    inline static const TMap kEmpty;

    std::map<map::MapId, TMap> maps_;  // node-based: a map's address is stable
    std::set<map::MapId> removed_;
    std::optional<map::MapId> active_map_;
};

template <typename TFeatureExtractor>
bool AtlasReplica<TFeatureExtractor>::Apply(const events::MapTransaction<TFeatureExtractor>& transaction) {
    assert(!transaction.absorbed_map.has_value());  // map merging: not yet
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

} // namespace voyahslam::tracking

#endif // VOYAHSLAM_TRACKING_ATLAS_REPLICA_H
