#include <algorithm>
#include <bit>
#include <cmath>
#include <optional>
#include <unordered_map>
#include <utility>
#include <variant>
#include <voyahslam/features/bow_matcher.h>
#include <voyahslam/features/instantiation.h>
#include <voyahslam/geometry/pnp.h>
#include <voyahslam/mapping/mapper.h>
#include <voyahslam/tracking/local_map.h>
#include <voyahslam/tracking/relocalizer.h>

namespace voyahslam::tracking {
namespace {

// The monocular measurement matched against the map: the left image of a
// binocular one.
template <typename TFeatureExtractor>
const features::TMonocularMeasurement<TFeatureExtractor>& LeftOf(
    const features::TMeasurement<TFeatureExtractor>& measurement) {
    if (const auto* binocular =
            std::get_if<features::TBinocularMeasurement<TFeatureExtractor>>(&measurement)) {
        return binocular->left();
    }
    return std::get<features::TMonocularMeasurement<TFeatureExtractor>>(measurement);
}

bool IsFinite(const geometry::RigidTransformation& T) {
    return T.rotation().allFinite() && T.translation().allFinite();
}

// SplitMix64 finalizer.
uint64_t Mix(uint64_t x) {
    x += 0x9E3779B97F4A7C15ULL;
    x = (x ^ (x >> 30U)) * 0xBF58476D1CE4E5B9ULL;
    x = (x ^ (x >> 27U)) * 0x94D049BB133111EBULL;
    return x ^ (x >> 31U);
}

// The RANSAC seed of one candidate in one round: reproducible per
// measurement, different per candidate and round.
uint64_t RansacSeed(double timestamp, map::FrameId frame, int round) {
    return Mix(Mix(Mix(std::bit_cast<uint64_t>(timestamp)) ^ frame.value) ^ static_cast<uint64_t>(round));
}

// One candidate Frame with its SearchByBoW matches and its RANSAC state.
struct Candidate {
    map::FrameId frame;
    std::vector<int64_t> matches;  // per current (left) feature: MapPointId value or -1
    // The RANSAC input, per match.
    std::vector<geometry::TPoint3DF> world;
    std::vector<geometry::TPoint2DF> pixels;
    std::vector<double> sigmas;
    int iterations = 0;  // RANSAC iterations used
    bool alive = true;
};

} // namespace

template <typename TFeatureExtractor>
MatchedPose OptimizeMatchedPose(const MapReplica<TFeatureExtractor>& replica,
                                const features::TMeasurement<TFeatureExtractor>& measurement,
                                const geometry::RigidTransformation& initial_T_cw,
                                std::vector<int64_t> matches, const PoseRefinement& refinement) {
    const auto& left = LeftOf<TFeatureExtractor>(measurement);
    const auto* binocular = std::get_if<features::TBinocularMeasurement<TFeatureExtractor>>(&measurement);

    std::vector<optimization::PoseObservation> observations;
    std::vector<size_t> feature_of;  // per observation
    for (size_t i = 0; i < matches.size(); ++i) {
        if (matches[i] < 0) {
            continue;
        }
        const map::MapPointId id{static_cast<uint32_t>(matches[i])};
        if (!replica.IsAlive(id)) {
            matches[i] = -1;
            continue;
        }
        const auto feature = static_cast<Eigen::Index>(i);
        optimization::PoseObservation observation{
            replica.Position(id), left.distorted_points().col(feature), std::nullopt,
            refinement.sigma_px * std::pow(refinement.level_scale, left.levels()(feature))};
        if (binocular) {
            const int right = binocular->right_indices()(feature);
            if (right >= 0) {
                observation.right = binocular->right().distorted_points().col(right);
            }
        }
        observations.push_back(observation);
        feature_of.push_back(i);
    }

    MatchedPose pose;
    pose.num_matches = observations.size();
    std::optional<optimization::StereoCameras> stereo;
    if (binocular) {
        stereo = optimization::StereoCameras{binocular->right().camera(), binocular->rig().T_rl};
    }
    const optimization::PoseOptimizationResult result = optimization::OptimizePose(
        initial_T_cw, left.camera(), stereo, observations, refinement.optimization);
    if (!result.success || !IsFinite(result.T_cw)) {
        pose.T_cw = initial_T_cw;
        pose.matches.assign(matches.size(), -1);
        return pose;
    }
    for (size_t k = 0; k < observations.size(); ++k) {
        if (!result.inlier[k]) {
            matches[feature_of[k]] = -1;
        }
    }
    pose.T_cw = result.T_cw;
    pose.matches = std::move(matches);
    pose.num_inliers = result.num_inliers;
    return pose;
}

std::vector<map::FrameId> RelocalizationCandidates(const std::vector<bow::DatabaseHit>& hits,
                                                   const map::CovisibilityIndex& covisibility,
                                                   size_t group_neighbours, double min_group_score_ratio) {
    std::unordered_map<uint32_t, double> score_of;  // by FrameId::value
    for (const bow::DatabaseHit& hit : hits) {
        score_of.emplace(hit.frame.value, hit.score);
    }

    struct Group {
        double score;
        map::FrameId best;  // its best-scoring Frame
    };
    std::vector<Group> groups;
    groups.reserve(hits.size());
    double best_group_score = 0.0;
    for (const bow::DatabaseHit& hit : hits) {
        Group group{hit.score, hit.frame};
        double best_score = hit.score;
        for (const auto& [neighbour, shared] : covisibility.Covisible(hit.frame, group_neighbours)) {
            const auto it = score_of.find(neighbour.value);
            if (it == score_of.end()) {
                continue;
            }
            group.score += it->second;
            if (it->second > best_score) {
                best_score = it->second;
                group.best = neighbour;
            }
        }
        best_group_score = std::max(best_group_score, group.score);
        groups.push_back(group);
    }

    // Stable: equal group scores keep the hits' order.
    std::stable_sort(groups.begin(), groups.end(),
                     [](const Group& a, const Group& b) { return a.score > b.score; });
    const double min_score = min_group_score_ratio * best_group_score;
    std::vector<map::FrameId> candidates;
    for (const Group& group : groups) {
        if (group.score < min_score) {
            break;
        }
        if (std::find(candidates.begin(), candidates.end(), group.best) == candidates.end()) {
            candidates.push_back(group.best);
        }
    }
    return candidates;
}

template <typename TFeatureExtractor>
RelocalizationResult Relocalize(const MapReplica<TFeatureExtractor>& replica,
                                const features::TMeasurement<TFeatureExtractor>& measurement,
                                const bow::Vocabulary<TFeatureExtractor>& vocabulary,
                                const RelocalizationOptions& options, const PoseRefinement& refinement,
                                const features::ProjectionMatcherOptions& matcher) {
    RelocalizationResult result;
    const auto& current = LeftOf<TFeatureExtractor>(measurement);
    const auto num_features = static_cast<size_t>(current.points().cols());
    if (num_features == 0) {
        return result;
    }

    // 1. Database query, with the Mapper's FeatureVector levels.
    bow::BowVector current_bow;
    bow::FeatureVector current_features;
    vocabulary.Transform(current.descriptors(), mapping::Mapper<TFeatureExtractor>::kBowLevelsUp,
                         current_bow, current_features);
    std::vector<bow::DatabaseHit> hits = replica.database().Query(current_bow, options.min_common_words_ratio);
    // The replica drops a removed Frame from its database too; a hit without
    // its Frame is skipped all the same.
    std::erase_if(hits, [&](const bow::DatabaseHit& hit) { return replica.keyframe(hit.frame) == nullptr; });

    // 2.-3. Candidate groups, then SearchByBoW against each candidate.
    features::BowMatchOptions bow_options;
    bow_options.nn_ratio = options.nn_ratio;
    std::vector<Candidate> candidates;
    for (const map::FrameId frame :
         RelocalizationCandidates(hits, replica.covisibility(), options.group_neighbours,
                                  options.min_group_score_ratio)) {
        const ReplicaKeyFrame<TFeatureExtractor>* keyframe = replica.keyframe(frame);
        if (keyframe == nullptr) {
            continue;
        }
        const auto& frame_measurement = LeftOf<TFeatureExtractor>(keyframe->measurement);
        std::vector<std::optional<map::MapPointId>> frame_points(
            static_cast<size_t>(frame_measurement.points().cols()));
        for (const auto& [feature, point] : keyframe->observations) {
            if (feature < frame_points.size() && replica.IsAlive(point)) {
                frame_points[feature] = point;
            }
        }
        const std::vector<std::optional<map::MapPointId>> found = features::SearchByBoW<TFeatureExtractor>(
            frame_measurement, keyframe->features, frame_points, current, current_features, bow_options);

        Candidate candidate{frame, std::vector<int64_t>(num_features, -1)};
        for (size_t i = 0; i < found.size(); ++i) {
            if (!found[i]) {
                continue;
            }
            const auto feature = static_cast<Eigen::Index>(i);
            candidate.matches[i] = static_cast<int64_t>(found[i]->value);
            candidate.world.push_back(replica.Position(*found[i]));
            candidate.pixels.push_back(current.distorted_points().col(feature));
            candidate.sigmas.push_back(refinement.sigma_px *
                                       std::pow(refinement.level_scale, current.levels()(feature)));
        }
        if (candidate.world.size() >= options.min_bow_matches) {
            candidates.push_back(std::move(candidate));
        }
    }

    // 4.-6. Rounds of RANSAC, each verified at once.
    const double timestamp = std::visit([](const auto& m) { return m.timestamp(); }, measurement);
    features::ProjectionMatcherOptions first_pass = matcher;
    first_pass.max_distance_map = options.first_pass_max_distance;
    features::ProjectionMatcherOptions second_pass = matcher;
    second_pass.max_distance_map = options.second_pass_max_distance;
    size_t num_alive = candidates.size();
    for (int round = 0; num_alive > 0; ++round) {
        for (Candidate& candidate : candidates) {
            if (!candidate.alive) {
                continue;
            }
            geometry::PnpOptions pnp;
            pnp.max_iterations = std::min(options.ransac_iterations_per_round,
                                          options.ransac_max_iterations - candidate.iterations);
            pnp.confidence = options.ransac_confidence;
            pnp.min_inliers = options.ransac_min_inliers;
            pnp.seed = RansacSeed(timestamp, candidate.frame, round);
            const geometry::PnpResult ransac =
                geometry::SolvePnPRansac(current.camera(), candidate.world, candidate.pixels, candidate.sigmas, pnp);
            candidate.iterations += std::max(pnp.max_iterations, 1);
            const auto drop = [&] {
                candidate.alive = false;
                --num_alive;
            };
            if (!ransac.success) {
                if (candidate.iterations >= options.ransac_max_iterations) {
                    drop();
                }
                continue;
            }

            // Refined over all of the candidate's matches (Ruling F): the
            // robust rounds drop the outliers.
            MatchedPose pose =
                OptimizeMatchedPose(replica, measurement, ransac.T_cw, candidate.matches, refinement);
            if (pose.num_inliers < options.min_refined_inliers) {
                drop();
                continue;
            }
            if (pose.num_inliers < options.min_inliers) {
                // The candidate Frame's other points.
                LocalMap frame_points;
                frame_points.frames = {candidate.frame};
                frame_points.point_mask.assign(replica.num_points(), false);
                for (const map::MapPointId point : replica.covisibility().Points(candidate.frame)) {
                    frame_points.point_mask[point.value] = replica.IsAlive(point);
                }
                const LocalPointSource<TFeatureExtractor> source(replica, frame_points);
                const auto project = [&](double window, const features::ProjectionMatcherOptions& pass) {
                    features::SearchFromLocalMap<TFeatureExtractor>(current, pose.T_cw, source, pose.matches,
                                                                    window, pass);
                    pose = OptimizeMatchedPose(replica, measurement, pose.T_cw, std::move(pose.matches),
                                               refinement);
                };
                project(options.first_pass_window, first_pass);
                if (pose.num_inliers > options.min_second_pass_inliers &&
                    pose.num_inliers < options.min_inliers) {
                    project(options.second_pass_window, second_pass);
                }
            }
            if (pose.num_inliers >= options.min_inliers) {
                result.success = true;
                result.T_cw = pose.T_cw;
                result.frame = candidate.frame;
                result.matches = std::move(pose.matches);
                result.inliers = pose.num_inliers;
                return result;
            }
            // Not verified: RANSAC goes on in later rounds while it has
            // iterations left (ORB-SLAM3).
            if (candidate.iterations >= options.ransac_max_iterations) {
                drop();
            }
        }
    }
    return result;
}

#define INSTANTIATE_RELOCALIZER(FE)                                                                     \
    template MatchedPose OptimizeMatchedPose<FE>(const MapReplica<FE>&, const features::TMeasurement<FE>&, \
                                                 const geometry::RigidTransformation&, std::vector<int64_t>, \
                                                 const PoseRefinement&);                                \
    template RelocalizationResult Relocalize<FE>(const MapReplica<FE>&, const features::TMeasurement<FE>&, \
                                                 const bow::Vocabulary<FE>&, const RelocalizationOptions&, \
                                                 const PoseRefinement&, const features::ProjectionMatcherOptions&);

INSTANTIATE_RELOCALIZER(features::SIFTFeatureExtractor)
INSTANTIATE_RELOCALIZER(features::OrbFeatureExtractor)

#undef INSTANTIATE_RELOCALIZER

} // namespace voyahslam::tracking
