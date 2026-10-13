#ifndef VOYAHSLAM_GLOBAL_BA_GLOBAL_BA_H
#define VOYAHSLAM_GLOBAL_BA_GLOBAL_BA_H

#include <cstddef>
#include <functional>
#include <optional>
#include <utility>
#include <vector>
#include <voyahslam/defs.h>
#include <voyahslam/geometry/rigid_transformation.h>
#include <voyahslam/map/frame_id.h>
#include <voyahslam/map/map_point_id.h>
#include <voyahslam/tracking/map_replica.h>

namespace voyahslam::global_ba {

// Global bundle adjustment after a loop correction (ORB-SLAM3
// Optimizer::GlobalBundleAdjustemnt with bRobust = false).
struct GlobalBaOptions {
    // Takes effect only when loop closing is on.
    bool enabled = true;
    // LM iterations of the single plain stage (ORB-SLAM3 runs 10).
    int iterations = 10;
    // Observation sigma at pyramid level 0 and the level scale: sigma =
    // sigma_px * level_scale^level (the defaults of MapperOptions::fuse).
    double sigma_px = 1.0;
    double level_scale = 1.2;
};

// The optimized map: the pose (T_cw) of every Frame and the world position of
// every point of the problem, and the Frame that was held fixed (its pose is
// the input pose, bitwise).
struct GlobalBaSolution {
    std::vector<std::pair<map::FrameId, geometry::RigidTransformation>> frame_poses;
    std::vector<std::pair<map::MapPointId, geometry::TPoint3DF>> point_positions;
    map::FrameId fixed_frame{};
};

// Bundle-adjusts every Frame of `replica` and every alive point with at
// least one observation, with all their observations (mono and stereo), in
// one plain stage of options.iterations: no robust kernel, no outlier
// removal, residuals in raw pixels. Gauge: the spanning tree's root (the
// oldest Frame without a parent; after a merge it need not be the oldest
// Frame), else the oldest surviving Frame, is fixed; mono poses are SE3, with
// no scale variable. `should_stop` is polled before the first iteration and
// after every iteration. nullopt if interrupted, on a map without Frames,
// points or observations, or on a non-finite result. Reads the replica only,
// and only before the first should_stop poll: the problem is a self-contained
// snapshot, so the replica may change once should_stop is first called (the
// GlobalBundleAdjuster applies arriving transactions there).
template <typename TFeatureExtractor>
std::optional<GlobalBaSolution> RunGlobalBa(const tracking::MapReplica<TFeatureExtractor>& replica,
                                            const GlobalBaOptions& options,
                                            const std::function<bool()>& should_stop);

} // namespace voyahslam::global_ba

#endif // VOYAHSLAM_GLOBAL_BA_GLOBAL_BA_H
