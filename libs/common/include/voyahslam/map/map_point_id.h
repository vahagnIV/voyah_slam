#ifndef VOYAHSLAM_MAP_MAP_POINT_ID_H
#define VOYAHSLAM_MAP_MAP_POINT_ID_H

#include <compare>
#include <cstdint>

namespace voyahslam::map {

struct MapPointId {
    uint32_t value;
    bool operator==(const MapPointId&) const = default;
    auto operator<=>(const MapPointId&) const = default;
};

} // namespace voyahslam::map

#endif // VOYAHSLAM_MAP_MAP_POINT_ID_H
