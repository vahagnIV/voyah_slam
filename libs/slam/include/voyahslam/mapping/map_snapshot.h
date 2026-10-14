#ifndef VOYAHSLAM_MAPPING_MAP_SNAPSHOT_H
#define VOYAHSLAM_MAPPING_MAP_SNAPSHOT_H

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <vector>
#include <voyahslam/defs.h>
#include <voyahslam/geometry/rigid_transformation.h>
#include <voyahslam/map/frame_id.h>
#include <voyahslam/map/map_id.h>
#include <voyahslam/map/map_point_id.h>

namespace voyahslam::mapping {

// A copy of the Atlas at one moment, as plain data (D-13): no pointer into any
// replica, no shared mutable state. It is immutable once published as a
// std::shared_ptr<const MapSnapshot> (MapFollower::Snapshot()), so it stays
// valid after the follower moves on or is destroyed, and any thread may read
// it without a lock.
struct MapSnapshot {
    struct Map {
        map::MapId id;
        uint32_t epoch = 0;                                      // the map's correction epoch
        std::vector<map::MapPointId> point_ids;                  // alive points, ascending
        std::vector<geometry::TPoint3DF> point_positions;        // world frame, same order
        std::vector<map::FrameId> frame_ids;                     // ascending
        std::vector<geometry::RigidTransformation> frame_poses;  // T_cw, same order
    };

    std::vector<Map> maps;  // every live map, ascending id
    std::optional<map::MapId> active_map;
    size_t num_transactions_applied = 0;

    // Map `id`; nullptr if it is not live.
    [[nodiscard]] const Map* Find(map::MapId id) const {
        const auto it = std::find_if(maps.begin(), maps.end(), [id](const Map& map) { return map.id == id; });
        return it == maps.end() ? nullptr : &*it;
    }
};

}  // namespace voyahslam::mapping

#endif  // VOYAHSLAM_MAPPING_MAP_SNAPSHOT_H
