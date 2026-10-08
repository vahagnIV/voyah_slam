#include <algorithm>
#include <cmath>
#include <limits>
#include <set>
#include <unordered_set>
#include <utility>
#include <variant>
#include <voyahslam/features/bow_matcher.h>
#include <voyahslam/features/feature_grid.h>
#include <voyahslam/features/instantiation.h>
#include <voyahslam/geometry/sim3_solver.h>
#include <voyahslam/loop_closing/loop_closer.h>
#include <voyahslam/optimization/sim3_optimization.h>
#include <voyahslam/tracking/relocalizer.h>

namespace voyahslam::loop_closing {
namespace {

template <typename TFeatureExtractor>
using TReplica = tracking::MapReplica<TFeatureExtractor>;
template <typename TFeatureExtractor>
using TKeyFrame = tracking::ReplicaKeyFrame<TFeatureExtractor>;
template <typename TFeatureExtractor>
using TMonocular = features::TMonocularMeasurement<TFeatureExtractor>;
using PointMatches = std::vector<std::optional<map::MapPointId>>;

// The monocular measurement matched against the map: the left image of a
// binocular one.
template <typename TFeatureExtractor>
const TMonocular<TFeatureExtractor>& LeftOf(const features::TMeasurement<TFeatureExtractor>& measurement) {
    if (const auto* binocular = std::get_if<features::TBinocularMeasurement<TFeatureExtractor>>(&measurement)) {
        return binocular->left();
    }
    return std::get<TMonocular<TFeatureExtractor>>(measurement);
}

// Stereo maps are metric: their similarities keep scale 1.
template <typename TFeatureExtractor>
bool FixesScale(const TKeyFrame<TFeatureExtractor>& keyframe) {
    return std::holds_alternative<features::TBinocularMeasurement<TFeatureExtractor>>(keyframe.measurement);
}

// Review Focus 4: a finite similarity with a positive scale.
bool IsValid(const geometry::Sim3& S) {
    return std::isfinite(S.scale) && S.scale > 0.0 && S.R.allFinite() && S.t.allFinite();
}

geometry::Sim3 ToSim3(const geometry::RigidTransformation& T) { return {1.0, T.rotation(), T.translation()}; }

// The rigid pose that projects like S_cw (ORB-SLAM3: Tcw = [R, t / s]): the
// camera in the world's units.
geometry::RigidTransformation FoldScale(const geometry::Sim3& S_cw) {
    return {S_cw.R, S_cw.t / S_cw.scale};
}

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

// Per feature of `keyframe`, the alive point it observes.
template <typename TFeatureExtractor>
PointMatches PointsOf(const TReplica<TFeatureExtractor>& replica, const TKeyFrame<TFeatureExtractor>& keyframe) {
    PointMatches points(static_cast<size_t>(LeftOf<TFeatureExtractor>(keyframe.measurement).points().cols()));
    for (const auto& [feature, point] : keyframe.observations) {
        if (feature < points.size() && replica.IsAlive(point)) {
            points[feature] = point;
        }
    }
    return points;
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

// ORB-SLAM3 ORBmatcher::SearchByProjection(KeyFrame*, Sim3 Scw, vpPoints,
// vpMatched, th, ratioHamming): projects each alive `point` with a reference
// that is not yet in out_matches into `keyframe` through S_cw (as the rigid
// [R, t / s], in the points' units) and matches it to the closest unmatched
// feature at levels [l - 1, l] within th * level_scale^l px, l the level
// predicted from its reference, accepted at <= max_distance. Skipped: behind
// the camera, beyond the ray-angle limit, outside the bounding box, a
// distance outside [(1 - slack) d_min, (1 + slack) d_max] (the reference's
// scale-invariance range), or a viewing angle over 60 deg from its normal.
// Returns the number added.
template <typename TFeatureExtractor>
size_t SearchBySim3(const TReplica<TFeatureExtractor>& replica, const TKeyFrame<TFeatureExtractor>& keyframe,
                    const geometry::Sim3& S_cw, const std::vector<map::MapPointId>& points, double th,
                    double max_distance, const features::ProjectionMatcherOptions& options,
                    PointMatches& out_matches) {
    const TMonocular<TFeatureExtractor>& current = LeftOf<TFeatureExtractor>(keyframe.measurement);
    std::unordered_set<uint32_t> already_found;
    for (const std::optional<map::MapPointId>& match : out_matches) {
        if (match) {
            already_found.insert(match->value);
        }
    }
    const geometry::RigidTransformation T_cw = FoldScale(S_cw);
    const geometry::TPoint3DF centre = T_cw.Inverse().translation();
    const features::FeatureGrid grid(current.distorted_points(), current.levels());
    const features::detail::ProjectionGate gate(current.camera(), current.distorted_points(),
                                                options.max_ray_angle_deg);
    const double max_level_scale = std::pow(options.level_scale, options.max_level);

    size_t added = 0;
    for (const map::MapPointId point : points) {
        if (!replica.IsAlive(point) || already_found.contains(point.value)) {
            continue;
        }
        const std::optional<map::PointReference>& reference = replica.Reference(point);
        if (!reference) {
            continue;
        }
        const geometry::TPoint3DF X = replica.Position(point);
        const std::optional<geometry::TPoint2DF> uv = gate.Project(T_cw.Transform(X));
        if (!uv) {
            continue;
        }
        const geometry::TPoint3DF view = X - centre;
        const double distance = view.norm();
        const double max_scale_distance = reference->distance * std::pow(options.level_scale, reference->level);
        const double min_scale_distance = max_scale_distance / max_level_scale;
        if (distance < (1.0 - options.scale_slack) * min_scale_distance ||
            distance > (1.0 + options.scale_slack) * max_scale_distance) {
            continue;
        }
        const geometry::TPoint3DF normal = replica.Normal(point);
        if (!(view.dot(normal) >= 0.5 * distance * normal.norm())) {
            continue;
        }
        const int level = features::detail::PredictLevel(*reference, distance, options.level_scale, options.max_level);
        const double radius = th * std::pow(options.level_scale, level);
        const typename TFeatureExtractor::TDescriptor descriptor = replica.Descriptor(point);
        double best = std::numeric_limits<double>::infinity();
        Eigen::Index best_index = -1;
        for (const Eigen::Index j : grid.FeaturesInArea(*uv, radius, level - 1, level)) {
            if (out_matches[static_cast<size_t>(j)]) {
                continue;
            }
            const double d =
                static_cast<double>(TFeatureExtractor::Distance(descriptor, current.descriptors().col(j)));
            if (d < best) {
                best = d;
                best_index = j;
            }
        }
        if (best_index < 0 || best > max_distance) {
            continue;
        }
        out_matches[static_cast<size_t>(best_index)] = point;
        already_found.insert(point.value);
        ++added;
    }
    return added;
}

// The match side's points of a region: the match Frame and its top `window`
// covisible Frames.
template <typename TFeatureExtractor>
std::vector<map::MapPointId> RegionPoints(const TReplica<TFeatureExtractor>& replica, map::FrameId match_frame,
                                          const LoopCloserOptions& options) {
    return PointsSeenBy(replica, WindowOf(replica.covisibility(), match_frame, options.window));
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
    const TMonocular<TFeatureExtractor>& query_left = LeftOf<TFeatureExtractor>(query.measurement);
    const TMonocular<TFeatureExtractor>& match_left = LeftOf<TFeatureExtractor>(match.measurement);
    optimization::Sim3OptimizationResult result =
        optimization::OptimizeSim3(query_left.camera(), match_left.camera(), pairs, initial, FixesScale(query));
    if (!result.success || !IsValid(result.S_12)) {
        return std::nullopt;
    }
    if (FixesScale(query)) {
        result.S_12.scale = 1.0;
    }
    return result;
}

// ORB-SLAM3 DetectCommonRegionsFromLastKF: `frame` confirms the region of
// `match_frame` at S_frame_world by projection.
template <typename TFeatureExtractor>
bool Confirms(const TReplica<TFeatureExtractor>& replica, const TKeyFrame<TFeatureExtractor>& frame,
              const geometry::Sim3& S_frame_world, const std::vector<map::MapPointId>& region_points,
              const LoopCloserOptions& options) {
    PointMatches matches(static_cast<size_t>(LeftOf<TFeatureExtractor>(frame.measurement).points().cols()));
    const size_t found = SearchBySim3(replica, frame, S_frame_world, region_points, options.confirmation_radius,
                                      options.confirmation_max_distance, options.matcher, matches);
    return found >= options.confirmation_projection;
}

// Database hits that are neither the query nor covisible with it (ORB-SLAM3
// DetectNBestCandidates): the common-words threshold is taken over them.
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
    const double min_common = options.min_common_words_ratio * static_cast<double>(max_common);
    std::erase_if(hits, [&](const bow::DatabaseHit& hit) { return static_cast<double>(hit.common_words) < min_common; });
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
    const std::vector<geometry::TPoint3DF> query_positions = PositionsOf(replica, query_points);
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
    result.stats.num_candidates = candidates.size();

    features::BowMatchOptions bow_options;
    bow_options.nn_ratio = options.bow_nn_ratio;
    for (const map::FrameId candidate : candidates) {
        // 2. The window, unless it touches the query's neighbourhood; then
        // SearchByBoW against each of its Frames.
        const std::vector<map::FrameId> window = WindowOf(covisibility, candidate, options.window);
        if (std::any_of(window.begin(), window.end(), [&](map::FrameId f) { return connected.contains(f); })) {
            continue;
        }
        PointMatches bow_matches(query_points.size());
        std::unordered_set<uint32_t> matched_points;
        const TKeyFrame<TFeatureExtractor>* match = nullptr;
        size_t most_matches = 0;
        size_t num_bow_matches = 0;
        for (const map::FrameId frame : window) {
            const TKeyFrame<TFeatureExtractor>& keyframe = *replica.keyframe(frame);
            const std::vector<std::optional<map::MapPointId>> found = features::SearchByBoW<TFeatureExtractor>(
                LeftOf<TFeatureExtractor>(keyframe.measurement), keyframe.features, PointsOf(replica, keyframe),
                query_left, query->features, bow_options);
            size_t num_found = 0;
            for (size_t i = 0; i < found.size(); ++i) {
                if (!found[i] || !query_points[i]) {
                    continue;  // the query side keeps map points only
                }
                ++num_found;
                if (!bow_matches[i] && matched_points.insert(found[i]->value).second) {
                    bow_matches[i] = found[i];
                    ++num_bow_matches;
                }
            }
            if (match == nullptr || num_found > most_matches) {
                most_matches = num_found;
                match = &keyframe;
            }
        }
        result.stats.bow_matches = std::max(result.stats.bow_matches, num_bow_matches);
        if (num_bow_matches < options.min_bow_matches) {
            continue;
        }

        // 3. RANSAC between the query's camera and the match Frame's.
        const TMonocular<TFeatureExtractor>& match_left = LeftOf<TFeatureExtractor>(match->measurement);
        std::vector<geometry::TPoint3DF> X1, X2;
        std::vector<geometry::TPoint2DF> pixels1, pixels2;
        std::vector<double> sigmas1, sigmas2;
        std::vector<size_t> feature_of;  // per pair
        for (size_t i = 0; i < bow_matches.size(); ++i) {
            if (!bow_matches[i]) {
                continue;
            }
            const geometry::TPoint3DF x1 = query->T_cw.Transform(query_positions[i]);
            const geometry::TPoint3DF x2 = match->T_cw.Transform(replica.Position(*bow_matches[i]));
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
            feature_of.push_back(i);
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

        // 4. The RANSAC inliers plus SearchBySim3Projection against each
        // window Frame, then OptimizeSim3.
        PointMatches sim3_matches(query_points.size());
        std::unordered_set<uint32_t> used;
        for (size_t k = 0; k < feature_of.size(); ++k) {
            if (ransac.inlier[k]) {
                sim3_matches[feature_of[k]] = bow_matches[feature_of[k]];
                used.insert(bow_matches[feature_of[k]]->value);
            }
        }
        for (const map::FrameId frame : window) {
            const TKeyFrame<TFeatureExtractor>& keyframe = *replica.keyframe(frame);
            const PointMatches frame_points = PointsOf(replica, keyframe);
            // x_query = S_query_match T_match_w T_frame_w⁻¹ x_frame.
            const geometry::Sim3 S_query_frame =
                geometry::Compose(ransac.S_12, ToSim3(match->T_cw * keyframe.T_cw.Inverse()));
            for (const auto& [i1, i2] : features::SearchBySim3Projection<TFeatureExtractor>(
                     query_left, query->T_cw, query_points, query_positions,
                     LeftOf<TFeatureExtractor>(keyframe.measurement), keyframe.T_cw, frame_points,
                     PositionsOf(replica, frame_points), S_query_frame, options.sim3_search_radius,
                     options.sim3_search_max_distance, options.matcher)) {
                const map::MapPointId point = *frame_points[i2];
                if (!sim3_matches[i1] && used.insert(point.value).second) {
                    sim3_matches[i1] = point;
                }
            }
        }
        const std::optional<optimization::Sim3OptimizationResult> optimized =
            Optimize(replica, *query, *match, sim3_matches, ransac.S_12, options);
        const size_t num_optimized = optimized ? optimized->num_inliers : 0;
        result.stats.optimized_inliers = std::max(result.stats.optimized_inliers, num_optimized);
        if (num_optimized < options.min_optimized) {
            continue;
        }

        // 5. The match side's points by projection with the optimized Sim3.
        const geometry::Sim3 S_query_world = geometry::Compose(optimized->S_12, ToSim3(match->T_cw));
        const std::vector<map::MapPointId> region_points = RegionPoints(replica, match->id, options);
        PointMatches matches(query_points.size());
        const size_t num_matches = SearchBySim3(replica, *query, S_query_world, region_points,
                                                options.projection_radius, options.projection_max_distance,
                                                options.matcher, matches);
        result.stats.projection_matches = std::max(result.stats.projection_matches, num_matches);
        if (num_matches < options.min_projection) {
            continue;
        }

        // 6. Confirmations by the query's covisible Frames.
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
            num_coincidences += Confirms(replica, *neighbour, S_neighbour_world, region_points, options) ? 1 : 0;
        }

        if (!result.region || num_matches > result.region->num_matches) {
            result.region = CommonRegion{match->id,         optimized->S_12,  S_query_world,
                                         std::move(matches), num_matches,      num_coincidences};
        }
    }
    return result;
}

template <typename TFeatureExtractor>
std::optional<CommonRegion> RefineCommonRegion(const tracking::MapReplica<TFeatureExtractor>& replica,
                                               map::FrameId query_frame, map::FrameId match_frame,
                                               const geometry::Sim3& S_query_world,
                                               const LoopCloserOptions& options) {
    const TKeyFrame<TFeatureExtractor>* query = replica.keyframe(query_frame);
    const TKeyFrame<TFeatureExtractor>* match = replica.keyframe(match_frame);
    if (query == nullptr || match == nullptr || !IsValid(S_query_world)) {
        return std::nullopt;
    }
    const size_t num_features = static_cast<size_t>(LeftOf<TFeatureExtractor>(query->measurement).points().cols());
    const std::vector<map::MapPointId> region_points = RegionPoints(replica, match_frame, options);
    const auto project = [&](const geometry::Sim3& S, PointMatches& out_matches) {
        out_matches.assign(num_features, std::nullopt);
        return SearchBySim3(replica, *query, S, region_points, options.confirmation_radius,
                            options.confirmation_max_distance, options.matcher, out_matches);
    };

    PointMatches matches;
    if (project(S_query_world, matches) < options.confirmation_projection) {
        return std::nullopt;
    }
    const geometry::Sim3 S_query_match = geometry::Compose(S_query_world, ToSim3(match->T_cw.Inverse()));
    const std::optional<optimization::Sim3OptimizationResult> optimized =
        Optimize(replica, *query, *match, matches, S_query_match, options);
    if (!optimized || optimized->num_inliers < options.min_optimized) {
        return std::nullopt;
    }
    const geometry::Sim3 refined = geometry::Compose(optimized->S_12, ToSim3(match->T_cw));
    const size_t num_matches = project(refined, matches);
    if (num_matches < options.confirmation_projection) {
        return std::nullopt;
    }
    return CommonRegion{match_frame, optimized->S_12, refined, std::move(matches), num_matches, 0};
}

template <typename TFeatureExtractor>
LoopCloser<TFeatureExtractor>::~LoopCloser() {
    worker_.Stop();
}

template <typename TFeatureExtractor>
void LoopCloser<TFeatureExtractor>::Start() {
    worker_.Start([this] { return ProcessPending(); });
}

template <typename TFeatureExtractor>
void LoopCloser<TFeatureExtractor>::Stop() {
    worker_.Stop();
}

template <typename TFeatureExtractor>
bool LoopCloser<TFeatureExtractor>::ProcessPending() {
    bool processed = false;
    while (const events::Event* event = inbox_listener_.GetNextEvent()) {
        processed = true;  // acknowledged by the next GetNextEvent()
        if (vocabulary_) {
            ApplyEvent(*event);
        }
    }
    return processed;
}

template <typename TFeatureExtractor>
void LoopCloser<TFeatureExtractor>::ApplyEvent(const events::Event& event) {
    if (event.map_id.value == 0 || event.map_id <= lost_map_) {
        return;  // map 0 is exempt; a lost map's late events are stale
    }
    if (event.type == events::EventType::TrackingLost) {
        if (static_cast<const events::TrackingLost&>(event).final && event.map_id == replica_.map_id()) {
            ClearMap(event.map_id);
        }
        return;
    }
    if (event.type != events::EventType::MapTransaction) {
        return;
    }
    const auto& transaction = static_cast<const events::MapTransaction<TFeatureExtractor>&>(event);
    if (event.map_id < replica_.map_id()) {
        return;  // an older map's
    }
    if (transaction.new_map || event.map_id > replica_.map_id()) {
        pending_.reset();  // the replica clears itself too
    }
    replica_.Apply(event);
    for (const events::NewFrame<TFeatureExtractor>& added : transaction.added_frames) {
        if (replica_.keyframe(added.id) != nullptr) {
            DetectLoop(added.id);
        }
    }
}

template <typename TFeatureExtractor>
void LoopCloser<TFeatureExtractor>::ClearMap(map::MapId lost_map) {
    replica_ = {};
    pending_.reset();
    lost_map_ = lost_map;
}

template <typename TFeatureExtractor>
void LoopCloser<TFeatureExtractor>::DetectLoop(map::FrameId query) {
    if (replica_.num_keyframes() < options_.min_map_frames) {
        return;
    }
    // A pending region is first checked against this query, from the last
    // confirming query's Sim3 moved by their relative pose.
    if (pending_ && pending_->num_coincidences > 0) {
        const auto* last = replica_.keyframe(pending_->last_query);
        const auto* current = replica_.keyframe(query);
        std::optional<CommonRegion> confirmed;
        if (last != nullptr && current != nullptr) {
            const geometry::Sim3 S_query_world =
                geometry::Compose(ToSim3(current->T_cw * last->T_cw.Inverse()), pending_->S_last_world);
            confirmed = RefineCommonRegion(replica_, query, pending_->match_frame, S_query_world, options_);
        }
        if (confirmed) {
            ++pending_->num_coincidences;
            pending_->last_query = query;
            pending_->S_last_world = confirmed->S_query_world;
            pending_->matches = std::move(confirmed->matches);
            pending_->num_not_found = 0;
            log_->debug("Loop region of Frame {} confirmed by Frame {} ({}/{})", pending_->match_frame.value,
                        query.value, pending_->num_coincidences, options_.required_coincidences);
            if (pending_->num_coincidences >= options_.required_coincidences) {
                PublishLoop(query, *pending_);
                pending_.reset();
            }
            return;
        }
        if (last == nullptr || ++pending_->num_not_found >= options_.max_unconfirmed_queries) {
            pending_.reset();
        }
    }

    const CommonRegionResult result = DetectCommonRegion(replica_, query, options_);
    last_stats_ = result.stats;
    if (!result.region) {
        return;
    }
    const CommonRegion& region = *result.region;
    log_->debug("Frame {} shares a region with Frame {}: {} matches, {} confirmations", query.value,
                region.match_frame.value, region.num_matches, region.num_coincidences);
    pending_ = PendingRegion{region.match_frame, query,           region.S_query_world, region.matches,
                             region.num_coincidences, 0};
    if (pending_->num_coincidences >= options_.required_coincidences) {
        PublishLoop(query, *pending_);
        pending_.reset();
    }
}

template <typename TFeatureExtractor>
void LoopCloser<TFeatureExtractor>::PublishLoop(map::FrameId query, const PendingRegion& region) {
    const auto* query_keyframe = replica_.keyframe(query);
    const auto* match_keyframe = replica_.keyframe(region.match_frame);
    if (query_keyframe == nullptr || match_keyframe == nullptr) {
        return;
    }
    geometry::Sim3 S_query_match = geometry::Compose(region.S_last_world, ToSim3(match_keyframe->T_cw.Inverse()));
    if (FixesScale(*query_keyframe)) {
        S_query_match.scale = 1.0;
    }
    if (!IsValid(S_query_match)) {
        return;
    }
    auto* event = loop_detected_pool_.Create();
    event->map_id = replica_.map_id();
    event->query_map_id = replica_.map_id();
    event->match_map_id = replica_.map_id();
    event->query_frame = query;
    event->match_frame = region.match_frame;
    event->S_query_to_match = S_query_match;
    const PointMatches query_points = PointsOf(replica_, *query_keyframe);
    for (size_t i = 0; i < region.matches.size() && i < query_points.size(); ++i) {
        if (region.matches[i] && query_points[i] && *region.matches[i] != *query_points[i]) {
            event->matched_points.emplace_back(*query_points[i], *region.matches[i]);
        }
    }
    log_->info("Loop detected: Frame {} with Frame {} (scale {:.3f}, {} matched points)", query.value,
               region.match_frame.value, S_query_match.scale, event->matched_points.size());
    observable_.Notify(event);
}

#define INSTANTIATE_LOOP_DETECTION(FE)                                                                      \
    template CommonRegionResult DetectCommonRegion<FE>(const tracking::MapReplica<FE>&, map::FrameId,        \
                                                       const LoopCloserOptions&);                           \
    template std::optional<CommonRegion> RefineCommonRegion<FE>(const tracking::MapReplica<FE>&, map::FrameId, \
                                                                map::FrameId, const geometry::Sim3&,        \
                                                                const LoopCloserOptions&);

INSTANTIATE_LOOP_DETECTION(features::SIFTFeatureExtractor)
INSTANTIATE_LOOP_DETECTION(features::OrbFeatureExtractor)

#undef INSTANTIATE_LOOP_DETECTION

INSTANTIATE_CLASS(LoopCloser)

} // namespace voyahslam::loop_closing
