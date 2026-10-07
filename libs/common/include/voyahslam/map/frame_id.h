#ifndef VOYAHSLAM_MAP_FRAME_ID_H
#define VOYAHSLAM_MAP_FRAME_ID_H

#include <compare>
#include <cstdint>

namespace voyahslam::map {

struct FrameId {
    uint32_t value;
    bool operator==(const FrameId&) const = default;
    auto operator<=>(const FrameId&) const = default;
};

} // namespace voyahslam::map

#endif // VOYAHSLAM_MAP_FRAME_ID_H
