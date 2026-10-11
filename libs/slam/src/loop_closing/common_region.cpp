#include <algorithm>
#include <cmath>
#include <limits>
#include <set>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <voyahslam/features/bow_matcher.h>
#include <voyahslam/features/instantiation.h>
#include <voyahslam/geometry/sim3_solver.h>
#include <voyahslam/loop_closing/common_region.h>
#include <voyahslam/optimization/sim3_optimization.h>

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
    const TMonocular<TFeatureExtractor>& left = LeftOf<TFeatureExtractor>(keyframe.measurement());
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

// Over the features with an own point in `own_points` (of `replica`) and a
// match in `matches` (of `match_replica`), the pairs whose normals are both
// defined, and the share of them more than 60 deg apart. Each normal is in
// its own map's coordinates.
template <typename TFeatureExtractor>
NormalAgreement CompareNormals(const TReplica<TFeatureExtractor>& replica, const PointMatches& own_points,
                               const TReplica<TFeatureExtractor>& match_replica, const PointMatches& matches) {
    size_t pairs = 0;
    size_t disagreeing = 0;
    for (size_t i = 0; i < matches.size() && i < own_points.size(); ++i) {
        if (!matches[i] || !own_points[i]) {
            continue;
        }
        const geometry::TPoint3DF& a = replica.Normal(*own_points[i]);
        const geometry::TPoint3DF& b = match_replica.Normal(*matches[i]);
        const double norms = a.norm() * b.norm();
        if (!(norms > 0.0) || !std::isfinite(norms)) {
            continue;  // undefined
        }
        ++pairs;
        disagreeing += a.dot(b) < kMaxNormalCosine * norms ? 1 : 0;
    }
    return {pairs, pairs > 0 ? static_cast<double>(disagreeing) / static_cast<double>(pairs) : 0.0};
}

// The normal gate, for loops only: across two maps the normals are in
// unrelated world coordinates, and the only relation between those worlds is
// the merge hypothesis itself, so a merge relies on ORB-SLAM3's gates
// (controller ruling 2026-10-10). Its measurement is still reported.
template <typename TFeatureExtractor>
bool Disagrees(const NormalAgreement& agreement, const TReplica<TFeatureExtractor>& replica,
               const TReplica<TFeatureExtractor>& match_replica, const LoopCloserOptions& options) {
    return replica.map_id() == match_replica.map_id() && agreement.disagreement > options.max_normal_disagreement;
}

size_t Count(const PointMatches& matches) {
    return static_cast<size_t>(std::count_if(matches.begin(), matches.end(), [](const auto& m) { return m.has_value(); }));
}

// One query feature matched to a match-side point, as OptimizeSim3 takes it:
// the query's own point (of `replica`) and keypoint, the matched point (of
// `match_replica`) in the match Frame's camera at its keypoint there, or else
// at its projection (ORB-SLAM3 OptimizeSim3 with bAllPoints), with the
// observing feature's level.
template <typename TFeatureExtractor>
std::optional<optimization::Sim3Match> MakeSim3Match(const TReplica<TFeatureExtractor>& replica,
                                                     const TKeyFrame<TFeatureExtractor>& query,
                                                     map::MapPointId query_point, uint32_t query_feature,
                                                     const TReplica<TFeatureExtractor>& match_replica,
                                                     const TKeyFrame<TFeatureExtractor>& match,
                                                     map::MapPointId match_point,
                                                     const LoopCloserOptions& options) {
    const TMonocular<TFeatureExtractor>& query_left = LeftOf<TFeatureExtractor>(query.measurement());
    const TMonocular<TFeatureExtractor>& match_left = LeftOf<TFeatureExtractor>(match.measurement());
    const auto feature1 = static_cast<Eigen::Index>(query_feature);
    optimization::Sim3Match pair{query.T_cw.Transform(replica.Position(query_point)),
                                 query_left.distorted_points().col(feature1),
                                 Sigma(options, query_left.levels()(feature1)),
                                 match.T_cw.Transform(match_replica.Position(match_point)),
                                 geometry::TPoint2DF::Zero(),
                                 0.0};
    if (pair.X1.z() <= 0.0 || pair.X2.z() <= 0.0) {
        return std::nullopt;
    }
    if (const std::optional<uint32_t> feature2 = FeatureOf(match_replica.covisibility(), match.id, match_point)) {
        const auto index = static_cast<Eigen::Index>(*feature2);
        pair.pixel2 = match_left.distorted_points().col(index);
        pair.sigma2 = Sigma(options, match_left.levels()(index));
    } else {
        const std::optional<map::PointReference>& reference = match_replica.Reference(match_point);
        pair.pixel2 = match_left.camera().Project(pair.X2);
        pair.sigma2 = Sigma(options, reference ? reference->level : 0);
    }
    return pair;
}

// OptimizeSim3 of S_query_match from `initial` over the query features in
// `matches` (points of `match_replica`) that observe a point themselves (of
// `replica`). None if it fails or the result is not a valid similarity.
template <typename TFeatureExtractor>
std::optional<optimization::Sim3OptimizationResult> Optimize(const TReplica<TFeatureExtractor>& replica,
                                                             const TKeyFrame<TFeatureExtractor>& query,
                                                             const TReplica<TFeatureExtractor>& match_replica,
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
        if (auto pair = MakeSim3Match(replica, query, *query_points[i], static_cast<uint32_t>(i), match_replica, match,
                                      *matches[i], options)) {
            pairs.push_back(*pair);
        }
    }
    optimization::Sim3OptimizationResult result = optimization::OptimizeSim3(
        LeftOf<TFeatureExtractor>(query.measurement()).camera(), LeftOf<TFeatureExtractor>(match.measurement()).camera(),
        pairs, initial, FixesScale(query));
    if (!result.success || !IsValid(result.S_12)) {
        return std::nullopt;
    }
    if (FixesScale(query)) {
        result.S_12.scale = 1.0;
    }
    return result;
}

// ORB-SLAM3 DetectNBestCandidates over `maps`, the query's own map first:
// see the header (DetectNBestCandidates).
template <typename TFeatureExtractor>
RegionCandidates NBestCandidates(const std::vector<const TReplica<TFeatureExtractor>*>& maps,
                                 map::FrameId query_frame, const LoopCloserOptions& options) {
    RegionCandidates candidates;
    if (maps.empty()) {
        return candidates;
    }
    const TReplica<TFeatureExtractor>& own = *maps.front();
    const TKeyFrame<TFeatureExtractor>* query = own.keyframe(query_frame);
    if (query == nullptr || query->bow.entries.empty()) {
        return candidates;
    }
    std::set<map::FrameId> connected;
    for (const auto& [frame, shared] : own.covisibility().Covisible(query_frame, std::numeric_limits<size_t>::max())) {
        connected.insert(frame);
    }

    // The hits of every map, minus the query and its covisible Frames.
    struct Hit {
        size_t map;  // index into `maps`
        bow::DatabaseHit hit;
    };
    std::vector<Hit> hits;
    for (size_t m = 0; m < maps.size(); ++m) {
        for (const bow::DatabaseHit& hit : maps[m]->database().Query(query->bow, 0.0)) {
            if (maps[m]->keyframe(hit.frame) == nullptr ||
                (m == 0 && (hit.frame == query_frame || connected.contains(hit.frame)))) {
                continue;
            }
            hits.push_back({m, hit});
        }
    }
    uint32_t max_common = 0;
    for (const Hit& hit : hits) {
        max_common = std::max(max_common, hit.hit.common_words);
    }
    const auto min_common =
        static_cast<uint32_t>(options.min_common_words_ratio * static_cast<double>(max_common));
    std::erase_if(hits, [&](const Hit& hit) { return hit.hit.common_words <= min_common; });
    // FrameIds are unique across maps: one order over every map's hits.
    std::sort(hits.begin(), hits.end(), [](const Hit& a, const Hit& b) {
        return a.hit.score != b.hit.score ? a.hit.score > b.hit.score : a.hit.frame < b.hit.frame;
    });
    std::unordered_map<uint32_t, double> score_of;  // by FrameId::value
    for (const Hit& hit : hits) {
        score_of.emplace(hit.hit.frame.value, hit.hit.score);
    }

    // Each hit's group: itself and its top num_covisibles covisible Frames
    // that are hits (covisibility is within its map).
    struct Group {
        double score;
        size_t map;
        map::FrameId best;
    };
    std::vector<Group> groups;
    groups.reserve(hits.size());
    for (const Hit& hit : hits) {
        Group group{hit.hit.score, hit.map, hit.hit.frame};
        double best_score = hit.hit.score;
        for (const auto& [neighbour, shared] :
             maps[hit.map]->covisibility().Covisible(hit.hit.frame, options.num_covisibles)) {
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
        groups.push_back(group);
    }
    std::stable_sort(groups.begin(), groups.end(), [](const Group& a, const Group& b) { return a.score > b.score; });

    std::set<map::FrameId> added;
    for (const Group& group : groups) {
        if (candidates.loop.size() >= options.num_candidates && candidates.merge.size() >= options.num_candidates) {
            break;
        }
        if (!added.insert(group.best).second) {
            continue;
        }
        std::vector<RegionCandidate>& list = group.map == 0 ? candidates.loop : candidates.merge;
        if (list.size() < options.num_candidates) {
            list.push_back({maps[group.map]->map_id(), group.best});
        }
    }
    return candidates;
}

// DetectCommonRegion for `query_frame` of `replica`; `match_replica_of(map)`
// is a candidate's map's replica, or null if it is gone.
template <typename TFeatureExtractor, typename TMatchReplicaOf>
CommonRegionResult VerifyCandidates(const TReplica<TFeatureExtractor>& replica, map::FrameId query_frame,
                                    const std::vector<RegionCandidate>& candidates,
                                    const TMatchReplicaOf& match_replica_of, const LoopCloserOptions& options) {
    CommonRegionResult result;
    for (const RegionCandidate& candidate : candidates) {
        result.stats.candidates.push_back(candidate.frame);
    }
    const TKeyFrame<TFeatureExtractor>* query = replica.keyframe(query_frame);
    if (query == nullptr || query->bow.entries.empty()) {
        return result;
    }
    const map::CovisibilityIndex& covisibility = replica.covisibility();
    const TMonocular<TFeatureExtractor>& query_left = LeftOf<TFeatureExtractor>(query->measurement());
    const bool fix_scale = FixesScale(*query);
    const PointMatches query_points = PointsOf(replica, *query);
    std::set<map::FrameId> connected;
    for (const auto& [frame, shared] : covisibility.Covisible(query_frame, std::numeric_limits<size_t>::max())) {
        connected.insert(frame);
    }

    features::BowMatchOptions bow_options;
    bow_options.nn_ratio = options.bow_nn_ratio;
    for (const auto& [match_map, candidate] : candidates) {
        const TReplica<TFeatureExtractor>* match_side = match_replica_of(match_map);
        if (match_side == nullptr || match_side->keyframe(candidate) == nullptr) {
            continue;
        }
        const TReplica<TFeatureExtractor>& match_replica = *match_side;
        // 1. The window, unless it touches the query's neighbourhood
        // (bAbortByNearKF); then SearchByBoW against each of its Frames.
        const std::vector<map::FrameId> window =
            WindowOf(match_replica.covisibility(), candidate, options.num_covisibles);
        if (std::any_of(window.begin(), window.end(), [&](map::FrameId f) { return connected.contains(f); })) {
            continue;
        }
        PointMatches bow_matches(query_points.size());
        std::unordered_set<uint32_t> matched_points;
        size_t num_bow_matches = 0;
        for (const map::FrameId frame : window) {
            const TKeyFrame<TFeatureExtractor>& keyframe = *match_replica.keyframe(frame);
            const PointMatches found = features::SearchByBoW<TFeatureExtractor>(
                LeftOf<TFeatureExtractor>(keyframe.measurement()), keyframe.features,
                PointsOf(match_replica, keyframe), query_left, query->features, bow_options);
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

        // 2. RANSAC between the query's camera and the candidate's.
        const TKeyFrame<TFeatureExtractor>& match = *match_replica.keyframe(candidate);
        const TMonocular<TFeatureExtractor>& match_left = LeftOf<TFeatureExtractor>(match.measurement());
        std::vector<geometry::TPoint3DF> X1, X2;
        std::vector<geometry::TPoint2DF> pixels1, pixels2;
        std::vector<double> sigmas1, sigmas2;
        for (size_t i = 0; i < bow_matches.size(); ++i) {
            if (!bow_matches[i]) {
                continue;
            }
            const geometry::TPoint3DF x1 = query->T_cw.Transform(replica.Position(*query_points[i]));
            const geometry::TPoint3DF x2 = match.T_cw.Transform(match_replica.Position(*bow_matches[i]));
            const std::optional<map::PointReference>& reference = match_replica.Reference(*bow_matches[i]);
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

        // 3. The window's points by projection with the RANSAC Sim3, then
        // OptimizeSim3 over those matches.
        const std::vector<map::MapPointId> window_points = PointsSeenBy(match_replica, window);
        const PointMatches coarse =
            Project(match_replica, *query, geometry::Compose(ransac.S_12, ToSim3(match.T_cw)), window_points,
                    options.coarse_projection_radius, options.coarse_projection_max_distance, options);
        const size_t num_coarse = Count(coarse);
        result.stats.coarse_projection_matches = std::max(result.stats.coarse_projection_matches, num_coarse);
        if (num_coarse < options.min_coarse_projection) {
            continue;
        }
        const std::optional<optimization::Sim3OptimizationResult> optimized =
            Optimize(replica, *query, match_replica, match, coarse, ransac.S_12, options);
        const size_t num_optimized = optimized ? optimized->num_inliers : 0;
        result.stats.optimized_inliers = std::max(result.stats.optimized_inliers, num_optimized);
        if (num_optimized < options.min_optimized) {
            continue;
        }

        // 4. The window's points by projection with the optimized Sim3.
        const geometry::Sim3 S_query_world = geometry::Compose(optimized->S_12, ToSim3(match.T_cw));
        PointMatches matches = Project(match_replica, *query, S_query_world, window_points, options.projection_radius,
                                       options.projection_max_distance, options);
        const size_t num_matches = Count(matches);
        result.stats.projection_matches = std::max(result.stats.projection_matches, num_matches);
        if (num_matches < options.min_projection) {
            continue;
        }
        // The normal gate (not in ORB-SLAM3).
        const NormalAgreement normals = CompareNormals(replica, query_points, match_replica, matches);
        if (Disagrees(normals, replica, match_replica, options)) {
            result.stats.normal_rejections.push_back({query_frame, candidate, normals});
            continue;
        }

        // 5. Confirmations by the query's covisible Frames
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
                Project(match_replica, *neighbour, S_neighbour_world, window_points, options.confirmation_radius,
                        options.confirmation_max_distance, options);
            if (Count(confirming) < options.confirmation_projection) {
                continue;
            }
            const NormalAgreement confirming_normals =
                CompareNormals(replica, PointsOf(replica, *neighbour), match_replica, confirming);
            if (Disagrees(confirming_normals, replica, match_replica, options)) {
                result.stats.normal_rejections.push_back({frame, candidate, confirming_normals});
                continue;
            }
            ++num_coincidences;
        }

        if (!result.region || num_matches > result.region->num_matches) {
            result.region = CommonRegion{candidate,   match_map,        optimized->S_12, S_query_world,
                                         std::move(matches), num_matches, num_coincidences, normals};
        }
    }
    return result;
}

} // namespace

template <typename TFeatureExtractor>
RegionCandidates DetectNBestCandidates(const tracking::AtlasReplica<TFeatureExtractor>& atlas,
                                       map::FrameId query_frame, const LoopCloserOptions& options) {
    const TReplica<TFeatureExtractor>* active = atlas.active();
    if (active == nullptr) {
        return {};
    }
    std::vector<const TReplica<TFeatureExtractor>*> maps{active};
    for (const map::MapId id : atlas.map_ids()) {
        if (id != atlas.active_map()) {
            maps.push_back(atlas.map(id));
        }
    }
    return NBestCandidates(maps, query_frame, options);
}

template <typename TFeatureExtractor>
CommonRegionResult DetectCommonRegion(const tracking::AtlasReplica<TFeatureExtractor>& atlas,
                                      map::FrameId query_frame, const std::vector<RegionCandidate>& candidates,
                                      const LoopCloserOptions& options) {
    return VerifyCandidates(
        atlas.active_or_empty(), query_frame, candidates, [&](map::MapId id) { return atlas.map(id); }, options);
}

template <typename TFeatureExtractor>
CommonRegionResult DetectCommonRegion(const tracking::MapReplica<TFeatureExtractor>& replica,
                                      map::FrameId query_frame, const LoopCloserOptions& options) {
    const RegionCandidates candidates = NBestCandidates<TFeatureExtractor>({&replica}, query_frame, options);
    return VerifyCandidates(
        replica, query_frame, candidates.loop,
        [&](map::MapId id) { return id == replica.map_id() ? &replica : nullptr; }, options);
}

template <typename TFeatureExtractor>
std::optional<CommonRegion> RefineCommonRegion(const tracking::MapReplica<TFeatureExtractor>& replica,
                                               map::FrameId query_frame,
                                               const tracking::MapReplica<TFeatureExtractor>& match_replica,
                                               map::FrameId match_frame, const geometry::Sim3& S_query_world,
                                               const LoopCloserOptions& options, CommonRegionStats* out_stats) {
    const TKeyFrame<TFeatureExtractor>* query = replica.keyframe(query_frame);
    const TKeyFrame<TFeatureExtractor>* match = match_replica.keyframe(match_frame);
    if (query == nullptr || match == nullptr || !IsValid(S_query_world)) {
        return std::nullopt;
    }
    // FindMatchesByProjection: the match Frame's window.
    const std::vector<map::MapPointId> region_points =
        PointsSeenBy(match_replica, WindowOf(match_replica.covisibility(), match_frame, options.num_covisibles));
    const auto project = [&](const geometry::Sim3& S) {
        return Project(match_replica, *query, S, region_points, options.confirmation_radius,
                       options.confirmation_max_distance, options);
    };

    const PointMatches matches = project(S_query_world);
    if (Count(matches) < options.confirmation_projection) {
        return std::nullopt;
    }
    const geometry::Sim3 S_query_match = geometry::Compose(S_query_world, ToSim3(match->T_cw.Inverse()));
    const std::optional<optimization::Sim3OptimizationResult> optimized =
        Optimize(replica, *query, match_replica, *match, matches, S_query_match, options);
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
    const NormalAgreement normals =
        CompareNormals(replica, PointsOf(replica, *query), match_replica, refined_matches);
    if (Disagrees(normals, replica, match_replica, options)) {
        if (out_stats != nullptr) {
            out_stats->normal_rejections.push_back({query_frame, match_frame, normals});
        }
        return std::nullopt;
    }
    return CommonRegion{match_frame, match_replica.map_id(), optimized->S_12, refined, std::move(refined_matches),
                        num_matches, 0,                      normals};
}

#define INSTANTIATE_COMMON_REGION(FE)                                                                          \
    template RegionCandidates DetectNBestCandidates<FE>(const tracking::AtlasReplica<FE>&, map::FrameId,        \
                                                        const LoopCloserOptions&);                             \
    template CommonRegionResult DetectCommonRegion<FE>(const tracking::AtlasReplica<FE>&, map::FrameId,         \
                                                       const std::vector<RegionCandidate>&,                    \
                                                       const LoopCloserOptions&);                              \
    template CommonRegionResult DetectCommonRegion<FE>(const tracking::MapReplica<FE>&, map::FrameId,           \
                                                       const LoopCloserOptions&);                              \
    template std::optional<CommonRegion> RefineCommonRegion<FE>(const tracking::MapReplica<FE>&, map::FrameId,    \
                                                                const tracking::MapReplica<FE>&, map::FrameId,    \
                                                                const geometry::Sim3&, const LoopCloserOptions&, \
                                                                CommonRegionStats*);

INSTANTIATE_COMMON_REGION(features::OrbFeatureExtractor)

#undef INSTANTIATE_COMMON_REGION

} // namespace voyahslam::loop_closing
