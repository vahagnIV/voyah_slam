#ifndef VOYAHSLAM_MAP_COVISIBILITY_INDEX_H
#define VOYAHSLAM_MAP_COVISIBILITY_INDEX_H

#include <cstddef>
#include <cstdint>
#include <unordered_map>
#include <utility>
#include <vector>
#include <voyahslam/map/frame_id.h>
#include <voyahslam/map/map_point_id.h>

namespace voyahslam::map {

// One Frame's observation of a map point.
struct PointObservation {
    FrameId frame;
    uint32_t feature;  // (left) feature index in the Frame's measurement
};

// The observations of one map, indexed both ways: per point, the Frames that
// observe it; per Frame, the points it observes. Covisibility between two
// Frames is the number of points they share, computed on demand. Point ids are
// dense per map, so the per-point side is a vector.
class CovisibilityIndex {
public:
    // Ignores an observation whose (frame, point) is already present.
    void Add(FrameId frame, uint32_t feature, MapPointId point);
    void Clear();

    // Empty if the point is unknown.
    [[nodiscard]] const std::vector<PointObservation>& Observations(MapPointId point) const;
    [[nodiscard]] size_t NumObservations(MapPointId point) const {
        return Observations(point).size();
    }
    // Empty if the Frame is unknown.
    [[nodiscard]] const std::vector<MapPointId>& Points(FrameId frame) const;
    [[nodiscard]] bool Contains(FrameId frame, MapPointId point) const;

    // The Frames sharing points with `frame` (excluding it), with the number of
    // shared points: by that count descending, ties by FrameId ascending; at
    // most `max_count` of them.
    [[nodiscard]] std::vector<std::pair<FrameId, uint32_t>> Covisible(FrameId frame,
                                                                      size_t max_count) const;

private:
    std::vector<std::vector<PointObservation>> observations_;  // by MapPointId::value
    std::unordered_map<uint32_t, std::vector<MapPointId>> points_;  // by FrameId::value
};

} // namespace voyahslam::map

#endif // VOYAHSLAM_MAP_COVISIBILITY_INDEX_H
