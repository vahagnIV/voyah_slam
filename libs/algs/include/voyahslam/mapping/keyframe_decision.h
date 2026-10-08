#ifndef VOYAHSLAM_MAPPING_KEYFRAME_DECISION_H
#define VOYAHSLAM_MAPPING_KEYFRAME_DECISION_H

#include <cstddef>

namespace voyahslam::mapping {

/// Thresholds of the Frame (keyframe) creation rule. Defaults are ORB-SLAM3's.
struct KeyFrameOptions {
    double max_interval = 1.0;        ///< Seconds since the last Frame that force a candidate.
    double ratio_mono = 0.9;          ///< Monocular: create when inliers < ratio * ref_points.
    double ratio_stereo = 0.75;       ///< Stereo: same, once num_frames >= 2.
    double ratio_stereo_early = 0.4;  ///< Stereo with fewer than 2 Frames in the map.
    double weak_ratio = 0.25;         ///< Stereo: tracking this weak creates even while busy.
    double close_depth_baselines = 40.0;  ///< A point is "close" when depth < this * baseline.
    size_t close_tracked_max = 100;       ///< need_close: fewer close points tracked than this ...
    size_t close_untracked_min = 70;      ///< ... and more close points available untracked than this.
    size_t min_inliers = 15;              ///< Floor: never create from this few inliers.
    /// Applied by the Mapper, not ShouldCreateFrame: in a map with more than
    /// relocalization_guard_min_frames Frames, no Frame from a tracked
    /// measurement less than relocalization_guard_seconds after the latest
    /// relocalized one, that one included (ORB-SLAM3 NeedNewKeyFrame:
    /// mnLastRelocFrameId + mMaxFrames, only while nKFs > mMaxFrames, mMaxFrames
    /// = fps). A smaller map must keep growing to keep tracking.
    double relocalization_guard_seconds = 1.0;
    size_t relocalization_guard_min_frames = 20;
};

/// Inputs to ShouldCreateFrame, measured by the Mapper for one tracked measurement.
struct KeyFrameSignals {
    bool stereo = false;
    size_t num_frames = 0;        ///< Frames already in the map.
    size_t ref_points = 0;        ///< Points observed by the reference Frame.
    double since_last = 0.0;      ///< Seconds since the last Frame was created.
    size_t backlog = 0;           ///< Pending Mapper work.
    size_t inliers = 0;           ///< Pose-inlier matches of this measurement.
    size_t close_tracked = 0;     ///< Stereo: close points tracked.
    size_t close_untracked = 0;   ///< Stereo: close points with valid depth but not tracked.
};

/// Pure decision: should this tracked measurement become a Frame? (spec section 3.3)
[[nodiscard]] bool ShouldCreateFrame(const KeyFrameSignals& signals, const KeyFrameOptions& options);

}  // namespace voyahslam::mapping

#endif  // VOYAHSLAM_MAPPING_KEYFRAME_DECISION_H
