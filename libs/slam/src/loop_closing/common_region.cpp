#include <algorithm>
#include <cmath>
#include <limits>
#include <set>
#include <unordered_set>
#include <utility>
#include <voyahslam/features/bow_matcher.h>
#include <voyahslam/features/instantiation.h>
#include <voyahslam/geometry/sim3_solver.h>
#include <voyahslam/loop_closing/common_region.h>
#include <voyahslam/optimization/sim3_optimization.h>
#include <voyahslam/tracking/relocalizer.h>

#include "loop_closing/common_region_internal.h"

namespace voyahslam::loop_closing {
namespace {

using detail::FixesScale;
using detail::IsValid;
using detail::PointMatches;
using detail::PointsOf;
using detail::ToSim3;
using features::LeftOf;

template <typename TFeatureExtractor>
using TReplica = tracking::MapReplica<TFeatureExtractor>;
template <typename TFeatureExtractor>
using TKeyFrame = tracking::ReplicaKeyFrame<TFeatureExtractor>;
template <typename TFeatureExtractor>
using TMonocular = features::TMonocularMeasurement<TFeatureExtractor>;

double Sigma(const LoopCloserOptions& options, int level) {
    return options.sigma_px * std::pow(options.level_scale, level);
}

// SplitMix64 finalizer.
uint64_t Mix(uint64_t x) {
    x += 0x9E3779B97F4A7C15ULL;
    x = (x ^ (x >> 30U)) * 0xBF58476D1CE4E5B9ULL;
    x = (x ^ (x >> 27U)) * 0x94D049BB133111EBULL;
    return x ^ (x >> 31U);
}

// Per feature, the world position of the point in `points` (zero if none).
template <typename TFeatureExtractor>
std::vector<geometry::TPoint3DF> PositionsOf(const TReplica<TFeatureExtractor>& replica, const PointMatches& points) {
    std::vector<geometry::TPoint3DF> positions(points.size(), geometry::TPoint3DF::Zero());
    for (size_t i = 0; i < points.size(); ++i) {
        if (points[i]) {
            positions[i] = replica.Position(*points[i]);
        }
    }
    return positions;
}

// `frame` followed by its top `count` covisible Frames.
std::vector<map::FrameId> WindowOf(const map::CovisibilityIndex& covisibility, map::FrameId frame, size_t count) {
    std::vector<map::FrameId> window{frame};
    for (const auto& [neighbour, shared] : covisibility.Covisible(frame, count)) {
        window.push_back(neighbour);
    }
    return window;
}

// The alive points observed by `frames`, once each, in order.
template <typename TFeatureExtractor>
std::vector<map::MapPointId> PointsSeenBy(const TReplica<TFeatureExtractor>& replica,
                                          const std::vector<map::FrameId>& frames) {
    std::vector<map::MapPointId> points;
    std::unordered_set<uint32_t> seen;
    for (const map::FrameId frame : frames) {
        for (const map::MapPointId point : replica.covisibility().Points(frame)) {
            if (replica.IsAlive(point) && seen.insert(point.value).second) {
                points.push_back(point);
            }
        }
    }
    return points;
}

// The (left) feature of `frame` observing `point`, if any.
std::optional<uint32_t> FeatureOf(const map::CovisibilityIndex& covisibility, map::FrameId frame,
                                  map::MapPointId point) {
    for (const map::PointObservation& observation : covisibility.Observations(point)) {
        if (observation.frame == frame) {
            return observation.feature;
        }
    }
    return std::nullopt;
}

// features::SearchPointsBySim3 of `points` into `keyframe` with a fresh match
// list.
template <typename TFeatureExtractor>
PointMatches Project(const TReplica<TFeatureExtractor>& replica, const TKeyFrame<TFeatureExtractor>& keyframe,
                     const geometry::Sim3& S_cw, const std::vector<map::MapPointId>& points, double th,
                     double max_distance, const LoopCloserOptions& options) {
    const TMonocular<TFeatureExtractor>& left = LeftOf<TFeatureExtractor>(keyframe.measurement);
    std::vector<int64_t> found(static_cast<size_t>(left.points().cols()), -1);
    features::SearchPointsBySim3<TFeatureExtractor>(left, S_cw, points, replica, found, th, max_distance,
                                                    options.matcher);
    PointMatches matches(found.size());
    for (size_t i = 0; i < found.size(); ++i) {
        if (found[i] >= 0) {
            matches[i] = map::MapPointId{static_cast<uint32_t>(found[i])};
        }
    }
    return matches;
}

// The normal gate's angle: normals more than 60 deg apart disagree.
constexpr double kMaxNormalCosine = 0.5;

// Over the features with an own point in `own_points` and a match in
// `matches`, the pairs whose normals are both defined, and the share of them
// more than 60 deg apart.
template <typename TFeatureExtractor>
NormalAgreement CompareNormals(const TReplica<TFeatureExtractor>& replica, const PointMatches& own_points,
                               const PointMatches& matches) {
    size_t pairs = 0;
    size_t disagreeing = 0;
    for (size_t i = 0; i < matches.size() && i < own_points.size(); ++i) {
        if (!matches[i] || !own_points[i]) {
            continue;
        }
        const geometry::TPoint3DF& a = replica.Normal(*own_points[i]);
        const geometry::TPoint3DF& b = replica.Normal(*matches[i]);
        const double norms = a.norm() * b.norm();
        if (!(norms > 0.0) || !std::isfinite(norms)) {
            continue;  // undefined
        }
        ++pairs;
        disagreeing += a.dot(b) < kMaxNormalCosine * norms ? 1 : 0;
    }
    return {pairs, pairs > 0 ? static_cast<double>(disagreeing) / static_cast<double>(pairs) : 0.0};
}

bool Disagrees(const NormalAgreement& agreement, const LoopCloserOptions& options) {
    return agreement.disagreement > options.max_normal_disagreement;
}

size_t Count(const PointMatches& matches) {
    return static_cast<size_t>(std::count_if(matches.begin(), matches.end(), [](const auto& m) { return m.has_value(); }));
}

// One query feature matched to a match-side point, as OptimizeSim3 takes it:
// the query's own point and keypoint, the matched point in the match Frame's
// camera at its keypoint there, or else at its projection (ORB-SLAM3
// OptimizeSim3 with bAllPoints), with the observing feature's level.
template <typename TFeatureExtractor>
std::optional<optimization::Sim3Match> MakeSim3Match(const TReplica<TFeatureExtractor>& replica,
                                                     const TKeyFrame<TFeatureExtractor>& query,
                                                     map::MapPointId query_point, uint32_t query_feature,
                                                     const TKeyFrame<TFeatureExtractor>& match,
                                                     map::MapPointId match_point,
                                                     const LoopCloserOptions& options) {
    const TMonocular<TFeatureExtractor>& query_left = LeftOf<TFeatureExtractor>(query.measurement);
    const TMonocular<TFeatureExtractor>& match_left = LeftOf<TFeatureExtractor>(match.measurement);
    const auto feature1 = static_cast<Eigen::Index>(query_feature);
    optimization::Sim3Match pair{query.T_cw.Transform(replica.Position(query_point)),
                                 query_left.distorted_points().col(feature1),
                                 Sigma(options, query_left.levels()(feature1)),
                                 match.T_cw.Transform(replica.Position(match_point)),
                                 geometry::TPoint2DF::Zero(),
                                 0.0};
    if (pair.X1.z() <= 0.0 || pair.X2.z() <= 0.0) {
        return std::nullopt;
    }
    if (const std::optional<uint32_t> feature2 = FeatureOf(replica.covisibility(), match.id, match_point)) {
        const auto index = static_cast<Eigen::Index>(*feature2);
        pair.pixel2 = match_left.distorted_points().col(index);
        pair.sigma2 = Sigma(options, match_left.levels()(index));
    } else {
        const std::optional<map::PointReference>& reference = replica.Reference(match_point);
        pair.pixel2 = match_left.camera().Project(pair.X2);
        pair.sigma2 = Sigma(options, reference ? reference->level : 0);
    }
    return pair;
}

// OptimizeSim3 of S_query_match from `initial` over the query features in
// `matches` that observe a point themselves. None if it fails or the result
// is not a valid similarity.
template <typename TFeatureExtractor>
std::optional<optimization::Sim3OptimizationResult> Optimize(const TReplica<TFeatureExtractor>& replica,
                                                             const TKeyFrame<TFeatureExtractor>& query,
                                                             const TKeyFrame<TFeatureExtractor>& match,
                                                             const PointMatches& matches,
                                                             const geometry::Sim3& initial,
                                                             const LoopCloserOptions& options) {
    const PointMatches query_points = PointsOf(replica, query);
    std::vector<optimization::Sim3Match> pairs;
    for (size_t i = 0; i < matches.size(); ++i) {
        if (!matches[i] || !query_points[i]) {
            continue;
        }
        if (auto pair = MakeSim3Match(replica, query, *query_points[i], static_cast<uint32_t>(i), match, *matches[i],
                                      options)) {
            pairs.push_back(*pair);
        }
    }
    optimization::Sim3OptimizationResult result = optimization::OptimizeSim3(
        LeftOf<TFeatureExtractor>(query.measurement).camera(), LeftOf<TFeatureExtractor>(match.measurement).camera(),
        pairs, initial, FixesScale(query));
    if (!result.success || !IsValid(result.S_12)) {
        return std::nullopt;
    }
    if (FixesScale(query)) {
        result.S_12.scale = 1.0;
    }
    return result;
}

// Database hits that are neither the query nor covisible with it, sharing
// more than int(min_common_words_ratio x max) words, max over those hits
// (ORB-SLAM3 DetectNBestCandidates).
template <typename TFeatureExtractor>
std::vector<bow::DatabaseHit> NonCovisibleHits(const TReplica<TFeatureExtractor>& replica,
                                               const TKeyFrame<TFeatureExtractor>& query,
                                               const std::set<map::FrameId>& connected,
                                               const LoopCloserOptions& options) {
    std::vector<bow::DatabaseHit> hits = replica.database().Query(query.bow, 0.0);
    std::erase_if(hits, [&](const bow::DatabaseHit& hit) {
        return hit.frame == query.id || connected.contains(hit.frame) || replica.keyframe(hit.frame) == nullptr;
    });
    uint32_t max_common = 0;
    for (const bow::DatabaseHit& hit : hits) {
        max_common = std::max(max_common, hit.common_words);
    }
    const auto min_common =
        static_cast<uint32_t>(options.min_common_words_ratio * static_cast<double>(max_common));
    std::erase_if(hits, [&](const bow::DatabaseHit& hit) { return hit.common_words <= min_common; });
    return hits;
}

} // namespace

template <typename TFeatureExtractor>
CommonRegionResult DetectCommonRegion(const tracking::MapReplica<TFeatureExtractor>& replica,
                                      map::FrameId query_frame, const LoopCloserOptions& options) {
    CommonRegionResult result;
    const TKeyFrame<TFeatureExtractor>* query = replica.keyframe(query_frame);
    if (query == nullptr || query->bow.entries.empty()) {
        return result;
    }
    const map::CovisibilityIndex& covisibility = replica.covisibility();
    const TMonocular<TFeatureExtractor>& query_left = LeftOf<TFeatureExtractor>(query->measurement);
    const bool fix_scale = FixesScale(*query);
    const PointMatches query_points = PointsOf(replica, *query);
    std::set<map::FrameId> connected;
    for (const auto& [frame, shared] : covisibility.Covisible(query_frame, std::numeric_limits<size_t>::max())) {
        connected.insert(frame);
    }

    // 1. Candidates: the best groups' best Frames.
    const std::vector<bow::DatabaseHit> hits = NonCovisibleHits(replica, *query, connected, options);
    std::vector<map::FrameId> candidates =
        tracking::RelocalizationCandidates(hits, covisibility, options.num_covisibles, 0.0);
    if (candidates.size() > options.num_candidates) {
        candidates.resize(options.num_candidates);
    }
    result.stats.candidates = candidates;

    features::BowMatchOptions bow_options;
    bow_options.nn_ratio = options.bow_nn_ratio;
    for (const map::FrameId candidate : candidates) {
        // 2. The window, unless it touches the query's neighbourhood
        // (bAbortByNearKF); then SearchByBoW against each of its Frames.
        const std::vector<map::FrameId> window = WindowOf(covisibility, candidate, options.num_covisibles);
        if (std::any_of(window.begin(), window.end(), [&](map::FrameId f) { return connected.contains(f); })) {
            continue;
        }
        PointMatches bow_matches(query_points.size());
        std::unordered_set<uint32_t> matched_points;
        size_t num_bow_matches = 0;
        for (const map::FrameId frame : window) {
            const TKeyFrame<TFeatureExtractor>& keyframe = *replica.keyframe(frame);
            const PointMatches found = features::SearchByBoW<TFeatureExtractor>(
                LeftOf<TFeatureExtractor>(keyframe.measurement), keyframe.features, PointsOf(replica, keyframe),
                query_left, query->features, bow_options);
            for (size_t i = 0; i < found.size(); ++i) {
                if (!found[i] || !query_points[i]) {
                    continue;  // the query side keeps map points only
                }
                if (!bow_matches[i] && matched_points.insert(found[i]->value).second) {
                    bow_matches[i] = found[i];
                    ++num_bow_matches;
                }
            }
        }
        result.stats.bow_matches = std::max(result.stats.bow_matches, num_bow_matches);
        if (num_bow_matches < options.min_bow_matches) {
            continue;
        }

        // 3. RANSAC between the query's camera and the candidate's.
        const TKeyFrame<TFeatureExtractor>& match = *replica.keyframe(candidate);
        const TMonocular<TFeatureExtractor>& match_left = LeftOf<TFeatureExtractor>(match.measurement);
        std::vector<geometry::TPoint3DF> X1, X2;
        std::vector<geometry::TPoint2DF> pixels1, pixels2;
        std::vector<double> sigmas1, sigmas2;
        for (size_t i = 0; i < bow_matches.size(); ++i) {
            if (!bow_matches[i]) {
                continue;
            }
            const geometry::TPoint3DF x1 = query->T_cw.Transform(replica.Position(*query_points[i]));
            const geometry::TPoint3DF x2 = match.T_cw.Transform(replica.Position(*bow_matches[i]));
            const std::optional<map::PointReference>& reference = replica.Reference(*bow_matches[i]);
            if (x1.z() <= 0.0 || x2.z() <= 0.0 || !reference) {
                continue;
            }
            X1.push_back(x1);
            X2.push_back(x2);
            pixels1.push_back(query_left.camera().Project(x1));
            pixels2.push_back(match_left.camera().Project(x2));
            sigmas1.push_back(Sigma(options, query_left.levels()(static_cast<Eigen::Index>(i))));
            sigmas2.push_back(Sigma(options, reference->level));
        }
        geometry::Sim3RansacOptions ransac_options;
        ransac_options.max_iterations = options.ransac_iterations;
        ransac_options.confidence = options.ransac_confidence;
        ransac_options.min_inliers = options.min_sim3_inliers;
        ransac_options.chi2 = options.ransac_chi2;
        ransac_options.fix_scale = fix_scale;
        ransac_options.seed = Mix(Mix(query_frame.value) ^ candidate.value);
        const geometry::Sim3RansacResult ransac = geometry::SolveSim3Ransac(
            query_left.camera(), match_left.camera(), X1, pixels1, sigmas1, X2, pixels2, sigmas2, ransac_options);
        result.stats.ransac_inliers = std::max(result.stats.ransac_inliers, ransac.num_inliers);
        if (!ransac.success || !IsValid(ransac.S_12)) {
            continue;
        }

        // 4. The window's points by projection with the RANSAC Sim3, then
        // OptimizeSim3 over those matches.
        const std::vector<map::MapPointId> window_points = PointsSeenBy(replica, window);
        const PointMatches coarse =
            Project(replica, *query, geometry::Compose(ransac.S_12, ToSim3(match.T_cw)), window_points,
                    options.coarse_projection_radius, options.coarse_projection_max_distance, options);
        const size_t num_coarse = Count(coarse);
        result.stats.coarse_projection_matches = std::max(result.stats.coarse_projection_matches, num_coarse);
        if (num_coarse < options.min_coarse_projection) {
            continue;
        }
        const std::optional<optimization::Sim3OptimizationResult> optimized =
            Optimize(replica, *query, match, coarse, ransac.S_12, options);
        const size_t num_optimized = optimized ? optimized->num_inliers : 0;
        result.stats.optimized_inliers = std::max(result.stats.optimized_inliers, num_optimized);
        if (num_optimized < options.min_optimized) {
            continue;
        }

        // 5. The window's points by projection with the optimized Sim3.
        const geometry::Sim3 S_query_world = geometry::Compose(optimized->S_12, ToSim3(match.T_cw));
        PointMatches matches = Project(replica, *query, S_query_world, window_points, options.projection_radius,
                                       options.projection_max_distance, options);
        const size_t num_matches = Count(matches);
        result.stats.projection_matches = std::max(result.stats.projection_matches, num_matches);
        if (num_matches < options.min_projection) {
            continue;
        }
        // The normal gate (not in ORB-SLAM3).
        const NormalAgreement normals = CompareNormals(replica, query_points, matches);
        if (Disagrees(normals, options)) {
            result.stats.normal_rejections.push_back({query_frame, candidate, normals});
            continue;
        }

        // 6. Confirmations by the query's covisible Frames
        // (DetectCommonRegionsFromLastKF).
        size_t num_coincidences = 0;
        for (const auto& [frame, shared] : covisibility.Covisible(query_frame, options.num_covisibles)) {
            if (num_coincidences >= options.required_coincidences) {
                break;
            }
            const TKeyFrame<TFeatureExtractor>* neighbour = replica.keyframe(frame);
            if (neighbour == nullptr) {
                continue;
            }
            const geometry::Sim3 S_neighbour_world =
                geometry::Compose(ToSim3(neighbour->T_cw * query->T_cw.Inverse()), S_query_world);
            const PointMatches confirming =
                Project(replica, *neighbour, S_neighbour_world, window_points, options.confirmation_radius,
                        options.confirmation_max_distance, options);
            if (Count(confirming) < options.confirmation_projection) {
                continue;
            }
            const NormalAgreement confirming_normals =
                CompareNormals(replica, PointsOf(replica, *neighbour), confirming);
            if (Disagrees(confirming_normals, options)) {
                result.stats.normal_rejections.push_back({frame, candidate, confirming_normals});
                continue;
            }
            ++num_coincidences;
        }

        if (!result.region || num_matches > result.region->num_matches) {
            result.region = CommonRegion{candidate,   optimized->S_12,  S_query_world, std::move(matches),
                                         num_matches, num_coincidences, normals};
        }
    }
    return result;
}

template <typename TFeatureExtractor>
std::optional<CommonRegion> RefineCommonRegion(const tracking::MapReplica<TFeatureExtractor>& replica,
                                               map::FrameId query_frame, map::FrameId match_frame,
                                               const geometry::Sim3& S_query_world,
                                               const LoopCloserOptions& options, CommonRegionStats* out_stats) {
    const TKeyFrame<TFeatureExtractor>* query = replica.keyframe(query_frame);
    const TKeyFrame<TFeatureExtractor>* match = replica.keyframe(match_frame);
    if (query == nullptr || match == nullptr || !IsValid(S_query_world)) {
        return std::nullopt;
    }
    // FindMatchesByProjection: the match Frame's window.
    const std::vector<map::MapPointId> region_points =
        PointsSeenBy(replica, WindowOf(replica.covisibility(), match_frame, options.num_covisibles));
    const auto project = [&](const geometry::Sim3& S) {
        return Project(replica, *query, S, region_points, options.confirmation_radius,
                       options.confirmation_max_distance, options);
    };

    const PointMatches matches = project(S_query_world);
    if (Count(matches) < options.confirmation_projection) {
        return std::nullopt;
    }
    const geometry::Sim3 S_query_match = geometry::Compose(S_query_world, ToSim3(match->T_cw.Inverse()));
    const std::optional<optimization::Sim3OptimizationResult> optimized =
        Optimize(replica, *query, *match, matches, S_query_match, options);
    if (!optimized || optimized->num_inliers <= options.refine_min_optimized) {
        return std::nullopt;
    }
    const geometry::Sim3 refined = geometry::Compose(optimized->S_12, ToSim3(match->T_cw));
    PointMatches refined_matches = project(refined);
    const size_t num_matches = Count(refined_matches);
    if (num_matches < options.refine_min_projection) {
        return std::nullopt;
    }
    // The normal gate (not in ORB-SLAM3).
    const NormalAgreement normals = CompareNormals(replica, PointsOf(replica, *query), refined_matches);
    if (Disagrees(normals, options)) {
        if (out_stats != nullptr) {
            out_stats->normal_rejections.push_back({query_frame, match_frame, normals});
        }
        return std::nullopt;
    }
    return CommonRegion{match_frame, optimized->S_12, refined, std::move(refined_matches), num_matches, 0, normals};
}

#define INSTANTIATE_COMMON_REGION(FE)                                                                       \
    template CommonRegionResult DetectCommonRegion<FE>(const tracking::MapReplica<FE>&, map::FrameId,        \
                                                       const LoopCloserOptions&);                           \
    template std::optional<CommonRegion> RefineCommonRegion<FE>(const tracking::MapReplica<FE>&, map::FrameId, \
                                                                map::FrameId, const geometry::Sim3&,        \
                                                                const LoopCloserOptions&, CommonRegionStats*);

INSTANTIATE_COMMON_REGION(features::SIFTFeatureExtractor)
INSTANTIATE_COMMON_REGION(features::OrbFeatureExtractor)

#undef INSTANTIATE_COMMON_REGION

} // namespace voyahslam::loop_closing
