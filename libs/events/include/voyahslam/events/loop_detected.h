#ifndef VOYAHSLAM_EVENTS_LOOP_DETECTED_H
#define VOYAHSLAM_EVENTS_LOOP_DETECTED_H

#include <utility>
#include <vector>
#include <voyahslam/events/event_types.h>
#include <voyahslam/geometry/sim3.h>
#include <voyahslam/map/frame_id.h>
#include <voyahslam/map/map_id.h>
#include <voyahslam/map/map_point_id.h>

namespace voyahslam::events {

// Published by the LoopCloser: Frame `query_frame` of map `query_map_id`
// revisits the place of Frame `match_frame` of map `match_map_id` (equal for a
// loop; the Event's map_id is query_map_id).
struct LoopDetected : Event {
    map::MapId query_map_id{0};
    map::MapId match_map_id{0};
    map::FrameId query_frame{};
    map::FrameId match_frame{};
    // ORB-SLAM3 Scm: the similarity from the match camera to the query
    // camera, x_query = S_query_match * x_match (camera coordinates). The
    // query's corrected pose is S_query_match composed with the match Frame's
    // T_cw. Scale 1 for a binocular map.
    geometry::Sim3 S_query_match;
    // (query point, match point): the query Frame's points matched to the
    // match side's points.
    std::vector<std::pair<map::MapPointId, map::MapPointId>> matched_points;

    LoopDetected() : Event(EventType::LoopDetected) {}
};

} // namespace voyahslam::events

#endif // VOYAHSLAM_EVENTS_LOOP_DETECTED_H
