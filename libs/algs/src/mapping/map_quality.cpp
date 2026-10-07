#include <voyahslam/mapping/map_quality.h>

namespace voyahslam::mapping {

RecentPointVerdict JudgeRecentPoint(const RecentPointStats& stats, const MapQualityOptions& options) {
    // found / visible < ratio, cross-multiplied so that 1/4 is exactly on the keep side.
    if (stats.visible > 0 &&
        static_cast<double>(stats.found) < options.min_found_ratio * static_cast<double>(stats.visible)) {
        return RecentPointVerdict::Remove;
    }
    const size_t min_obs = stats.stereo ? options.min_obs_stereo : options.min_obs_mono;
    if (stats.frames_since_creation >= 2 && stats.weighted_obs <= min_obs) {
        return RecentPointVerdict::Remove;
    }
    if (stats.frames_since_creation >= options.recent_window) return RecentPointVerdict::Graduate;
    return RecentPointVerdict::Keep;
}

bool IsRedundantFrame(const std::vector<FramePointStats>& points, const MapQualityOptions& options) {
    size_t counted = 0;
    size_t covered = 0;
    for (const auto& p : points) {
        if (!p.counted) continue;
        ++counted;
        if (p.other_observers_at_level >= options.redundancy_observers) ++covered;
    }
    if (counted == 0) return false;
    return static_cast<double>(covered) >= options.redundancy_ratio * static_cast<double>(counted);
}

}  // namespace voyahslam::mapping
