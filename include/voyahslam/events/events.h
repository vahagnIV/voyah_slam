#ifndef VOYAHSLAM_EVENTS_EVENTS_H
#define VOYAHSLAM_EVENTS_EVENTS_H

// Convenience header: all concrete event types. Prefer including the specific
// event header where only one or two are needed.

#include <voyahslam/events/initialization_measurement.h>
#include <voyahslam/events/key_frame_created.h>
#include <voyahslam/events/key_frame_pose_updated.h>
#include <voyahslam/events/loop_detected.h>
#include <voyahslam/events/map_created.h>
#include <voyahslam/events/map_points_added.h>
#include <voyahslam/events/map_points_removed.h>
#include <voyahslam/events/map_points_updated.h>
#include <voyahslam/events/new_key_frame_candidate.h>
#include <voyahslam/events/position_updated.h>
#include <voyahslam/events/tracking_lost.h>

#endif // VOYAHSLAM_EVENTS_EVENTS_H
