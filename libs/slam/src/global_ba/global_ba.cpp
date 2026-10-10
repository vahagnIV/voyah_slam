#include <unordered_map>
#include <voyahslam/features/instantiation.h>
#include <voyahslam/global_ba/global_ba.h>
#include <voyahslam/optimization/bundle_adjustment.h>

#include "mapping/mapper_internal.h"

namespace voyahslam::global_ba {

template <typename TFeatureExtractor>
std::optional<GlobalBaSolution> RunGlobalBa(const tracking::MapReplica<TFeatureExtractor>& replica,
                                            const GlobalBaOptions& options,
                                            const std::function<bool()>& should_stop) {
    // Frames in id order: the first is the oldest surviving one, the gauge
    // (ORB-SLAM3 fixes the map's init keyframe).
    const std::vector<map::FrameId> frames = replica.keyframe_ids();
    if (frames.empty()) {
        return std::nullopt;
    }
    std::unordered_map<uint32_t, size_t> frame_index;
    optimization::BaProblem problem;
    problem.keyframe_poses.reserve(frames.size());
    problem.keyframe_fixed.reserve(frames.size());
    problem.keyframe_cameras.reserve(frames.size());
    problem.keyframe_stereo.reserve(frames.size());
    for (size_t i = 0; i < frames.size(); ++i) {
        const tracking::ReplicaKeyFrame<TFeatureExtractor>& keyframe = *replica.keyframe(frames[i]);
        frame_index.emplace(frames[i].value, i);
        problem.keyframe_poses.push_back(keyframe.T_cw);
        problem.keyframe_fixed.push_back(i == 0);
        problem.keyframe_cameras.push_back(features::LeftOf<TFeatureExtractor>(keyframe.measurement()).camera());
        problem.keyframe_stereo.push_back(mapping::detail::BaStereoOf<TFeatureExtractor>(keyframe.measurement()));
    }

    // Every alive point with an observation, with all its observations.
    std::vector<map::MapPointId> points;
    for (uint32_t value = 0; value < replica.num_points(); ++value) {
        const map::MapPointId point{value};
        if (replica.IsAlive(point) && replica.covisibility().NumObservations(point) > 0) {
            points.push_back(point);
        }
    }
    problem.points.resize(3, static_cast<Eigen::Index>(points.size()));
    for (size_t p = 0; p < points.size(); ++p) {
        problem.points.col(static_cast<Eigen::Index>(p)) = replica.Position(points[p]);
        for (const map::PointObservation& observation : replica.covisibility().Observations(points[p])) {
            const size_t k = frame_index.at(observation.frame.value);
            problem.observations.push_back(mapping::detail::BaObservationOf<TFeatureExtractor>(
                replica.keyframe(observation.frame)->measurement(), observation.feature, k, p,
                options.sigma_px, options.level_scale));
        }
    }

    // One plain stage of `iterations` (bundle_adjustment.h: robust == false
    // runs robust_iterations + iterations without a kernel and deactivates
    // nothing).
    optimization::BundleAdjustmentOptions ba_options;
    ba_options.robust = false;
    ba_options.robust_iterations = 0;
    ba_options.iterations = options.iterations;
    ba_options.should_stop = should_stop;
    // Invalid input (no points or observations) and a non-finite result fail.
    const optimization::BaResult result = optimization::BundleAdjust(problem, ba_options);
    if (!result.success || result.interrupted) {
        return std::nullopt;
    }

    GlobalBaSolution solution;
    solution.fixed_frame = frames.front();
    solution.frame_poses.reserve(frames.size());
    for (size_t i = 0; i < frames.size(); ++i) {
        solution.frame_poses.emplace_back(frames[i], result.keyframe_poses[i]);
    }
    solution.point_positions.reserve(points.size());
    for (size_t p = 0; p < points.size(); ++p) {
        solution.point_positions.emplace_back(points[p], result.points.col(static_cast<Eigen::Index>(p)));
    }
    return solution;
}

#define INSTANTIATE_RUN_GLOBAL_BA(FE)                                                                      \
    template std::optional<GlobalBaSolution> RunGlobalBa<FE>(const tracking::MapReplica<FE>&,              \
                                                             const GlobalBaOptions&,                       \
                                                             const std::function<bool()>&);

INSTANTIATE_RUN_GLOBAL_BA(features::SIFTFeatureExtractor)
INSTANTIATE_RUN_GLOBAL_BA(features::OrbFeatureExtractor)

#undef INSTANTIATE_RUN_GLOBAL_BA

} // namespace voyahslam::global_ba
