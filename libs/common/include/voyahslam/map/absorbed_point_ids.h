#ifndef VOYAHSLAM_MAP_ABSORBED_POINT_IDS_H
#define VOYAHSLAM_MAP_ABSORBED_POINT_IDS_H

#include <cstdint>
#include <optional>
#include <vector>
#include <voyahslam/map/map_point_id.h>

namespace voyahslam::map {

// The id rule of a map merge (map-merging spec section 3, step 7), applied
// alike by the Mapper and by every replica: the absorbed map's alive points,
// by old id ascending, take the ids first_new, first_new + 1, ... of the map
// that absorbs them (its next id). Returns per old id in [0, num_ids) its new
// id, nullopt for a point that is not alive. `is_alive(MapPointId)` tells
// whether an old id is alive in the absorbed map.
template <typename TIsAlive>
std::vector<std::optional<MapPointId>> AbsorbedPointIds(uint32_t num_ids, MapPointId first_new,
                                                        TIsAlive&& is_alive) {
    std::vector<std::optional<MapPointId>> new_ids(num_ids);
    uint32_t next = first_new.value;
    for (uint32_t old = 0; old < num_ids; ++old) {
        if (is_alive(MapPointId{old})) {
            new_ids[old] = MapPointId{next++};
        }
    }
    return new_ids;
}

} // namespace voyahslam::map

#endif // VOYAHSLAM_MAP_ABSORBED_POINT_IDS_H
