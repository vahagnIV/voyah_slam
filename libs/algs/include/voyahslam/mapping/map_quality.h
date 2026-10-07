#ifndef VOYAHSLAM_MAPPING_MAP_QUALITY_H
#define VOYAHSLAM_MAPPING_MAP_QUALITY_H

#include <cstddef>
#include <cstdint>
#include <vector>

namespace voyahslam::mapping {

/// Thresholds of the map-quality passes (fusion, culling, local BA). Defaults are ORB-SLAM3's.
struct MapQualityOptions {
    size_t fuse_neighbours_mono = 30;    ///< Covisible Frames searched for fusion, monocular.
    size_t fuse_neighbours_stereo = 10;  ///< Same, stereo.
    size_t fuse_second_order = 20;       ///< Neighbours of neighbours added to the fusion search.
    size_t recent_window = 3;            ///< Frames a point stays on the recent list.
    double min_found_ratio = 0.25;       ///< Recent point removed when found / visible < this.
    size_t min_obs_mono = 2;             ///< Removed if weighted observations <= this after 2 Frames.
    size_t min_obs_stereo = 3;           ///< Same, stereo.
    double redundancy_ratio = 0.9;       ///< Frame redundant when this fraction of points is covered.
    size_t redundancy_observers = 3;     ///< Other observers needed to cover a point.
    size_t ba_min_shared = 15;           ///< Local BA: min shared points for a covisible Frame.
    size_t ba_max_optimized = 20;        ///< Local BA: cap on optimized Frames.
    int ba_robust_iterations = 5;        ///< Local BA: robust-stage iterations.
    int ba_iterations = 10;              ///< Local BA: plain-stage iterations.
};

/// Per-point counters of a recently created map point, kept by the Mapper.
struct RecentPointStats {
    size_t frames_since_creation = 0;
    uint32_t visible = 0;
    uint32_t found = 0;
    size_t weighted_obs = 0;  ///< Observation count, stereo observations weighted as in ORB-SLAM3.
    bool stereo = false;
};

enum class RecentPointVerdict { Keep, Remove, Graduate };

/// Pure decision for a recent point (spec section 4.1). Graduate: leaves the recent list.
[[nodiscard]] RecentPointVerdict JudgeRecentPoint(const RecentPointStats& stats,
                                                  const MapQualityOptions& options);

/// One point of a candidate Frame, as measured by the Mapper.
struct FramePointStats {
    int level = 0;
    bool counted = true;  ///< False for stereo far points, which are ignored.
    size_t other_observers_at_level = 0;  ///< Other Frames observing at level <= level + 1.
};

/// Pure decision: is this Frame redundant? (spec section 4.2) False with no counted points.
[[nodiscard]] bool IsRedundantFrame(const std::vector<FramePointStats>& points,
                                    const MapQualityOptions& options);

}  // namespace voyahslam::mapping

#endif  // VOYAHSLAM_MAPPING_MAP_QUALITY_H
