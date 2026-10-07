#include <voyahslam/mapping/keyframe_decision.h>

namespace voyahslam::mapping {

bool ShouldCreateFrame(const KeyFrameSignals& signals, const KeyFrameOptions& options) {
    const double inliers = static_cast<double>(signals.inliers);
    const double ref_points = static_cast<double>(signals.ref_points);

    const bool need_close = signals.stereo && signals.close_tracked < options.close_tracked_max &&
                            signals.close_untracked > options.close_untracked_min;

    const bool c1a = signals.since_last >= options.max_interval;
    const bool c1b = signals.backlog == 0;
    const bool c1c =
        signals.stereo && (inliers < options.weak_ratio * ref_points || need_close);

    const double ratio = !signals.stereo ? options.ratio_mono
                         : signals.num_frames < 2 ? options.ratio_stereo_early
                                                  : options.ratio_stereo;
    const bool c2 = (inliers < ratio * ref_points || need_close) && signals.inliers > options.min_inliers;

    return (c1a || c1b || c1c) && c2;
}

}  // namespace voyahslam::mapping
