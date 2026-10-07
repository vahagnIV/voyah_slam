#ifndef VOYAHSLAM_MAP_MAP_ID_H
#define VOYAHSLAM_MAP_MAP_ID_H

#include <compare>
#include <cstdint>

namespace voyahslam::map {

struct MapId {
    uint32_t value;
    bool operator==(const MapId&) const = default;
    auto operator<=>(const MapId&) const = default;
};

} // namespace voyahslam::map

#endif // VOYAHSLAM_MAP_MAP_ID_H
