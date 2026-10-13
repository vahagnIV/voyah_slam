#ifndef VOYAHSLAM_LOOP_CLOSING_COMMON_REGION_H
#define VOYAHSLAM_LOOP_CLOSING_COMMON_REGION_H

#include <cstddef>
#include <optional>
#include <vector>
#include <voyahslam/features/projection_matcher.h>
#include <voyahslam/geometry/sim3.h>
#include <voyahslam/map/frame_id.h>
#include <voyahslam/map/map_id.h>
#include <voyahslam/map/map_point_id.h>
#include <voyahslam/tracking/atlas_replica.h>
#include <voyahslam/tracking/map_replica.h>

namespace voyahslam::loop_closing {

// Place recognition (spec sections 2 and 4; ORB-SLAM3 NewDetectCommonRegions).
// Values from the ORB-SLAM3 source (LoopClosing.cc, KeyFrameDatabase.cc).
struct LoopCloserOptions {
    // Loop closing is opt-in (user decision 2026-10-10: on corridor1 it makes
    // the ATE worse through same-heading look-alikes; roadmap step 6). False:
    // no loop closing, as without a vocabulary; the Mapper and the Tracker
    // still use theirs (BoW triangulation, relocalization). The apps' option
    // is --loop_closing.
    bool enabled = false;
    // A map with fewer Frames is not searched.
    size_t min_map_frames = 12;
    // DetectNBestCandidates: the best groups' Frames tried per query.
    size_t num_candidates = 3;
    // nNumCovisibles: a candidate's window is the candidate plus its top
    // `num_covisibles` covisible Frames; a candidate group scores over that
    // many covisible Frames; that many of the query's covisible Frames are
    // tried as confirmations.
    size_t num_covisibles = 10;
    // DetectCommonRegionsFromBoW: SearchByBoW matches (nBoWMatches), RANSAC
    // inliers (nBoWInliers), matches by projection with the RANSAC Sim3
    // (nProjMatches), OptimizeSim3 inliers (nSim3Inliers), matches by
    // projection with the optimized Sim3 (nProjOptMatches); each at least.
    size_t min_bow_matches = 20;
    size_t min_sim3_inliers = 15;
    size_t min_coarse_projection = 50;
    size_t min_optimized = 20;
    size_t min_projection = 80;
    // DetectCommonRegionsFromLastKF: a covisible Frame of the query confirms
    // with at least this many matches by projection.
    size_t confirmation_projection = 30;
    // DetectAndReffineSim3FromLastKF (a later query): confirmation_projection
    // matches, then more than refine_min_optimized OptimizeSim3 inliers, then
    // at least refine_min_projection matches with the refined Sim3.
    size_t refine_min_optimized = 50;
    size_t refine_min_projection = 100;
    // Normal consistency (an addition to ORB-SLAM3, user decision
    // 2026-10-09; loops only, not merges, ruling 2026-10-10): a candidate, a
    // covisible confirmation or a later query's confirmation of a loop is
    // rejected when more than this share of its final matched
    // pairs (own map point, matched map point) have normals more than 60 deg
    // apart; pairs with an undefined normal do not count. Opposite-facing
    // false loops measured >= 0.89 on corridor1, true loops <= 0.05.
    double max_normal_disagreement = 0.5;
    // Confirmations before a region is a loop (mnLoopNumCoincidences); later
    // queries in a row that fail to confirm before it is dropped
    // (mnLoopNumNotFound).
    size_t required_coincidences = 3;
    size_t max_unconfirmed_queries = 2;
    // Queries (Frame ids) waiting for detection; beyond it the oldest are
    // dropped. Not ORB-SLAM3, whose keyframe queue is unbounded: the
    // LoopCloser applies each MapTransaction on receipt so that a slow
    // detector never holds the Mapper's transactions, and only the queries
    // wait. This is also the back-pressure that bounds the loops it publishes.
    size_t max_query_backlog = 8;
    // SolveSim3Ransac (Sim3Solver::SetRansacParameters, th2 9.21).
    int ransac_iterations = 300;
    double ransac_confidence = 0.99;
    double ransac_chi2 = 9.21;
    // Database hits share more than int(min_common_words_ratio x the best
    // non-covisible hit's) common words.
    double min_common_words_ratio = 0.8;
    // SearchByBoW against the window (ORBmatcher(0.9, true)).
    double bow_nn_ratio = 0.9;
    // features::SearchPointsBySim3 radii th * level_scale^level px and
    // descriptor distances TH_LOW (50) x ratioHamming: with the RANSAC Sim3
    // (th 8, 1.5), with the optimized one (th 5, 1.0), for confirmations
    // (FindMatchesByProjection: th 3, 1.5).
    double coarse_projection_radius = 8.0;
    double coarse_projection_max_distance = 75.0;
    double projection_radius = 5.0;
    double projection_max_distance = 50.0;
    double confirmation_radius = 3.0;
    double confirmation_max_distance = 75.0;
    // Residuals in raw pixels, sigma = sigma_px * level_scale^level.
    double sigma_px = 1.0;
    double level_scale = 1.2;
    // Level scale, number of levels and ray-angle limit of the projections.
    features::ProjectionMatcherOptions matcher;
};

// The normal gate's measurement over matched pairs: pairs with both normals
// defined, and the share of them more than 60 deg apart.
struct NormalAgreement {
    size_t pairs = 0;
    double disagreement = 0.0;  // 0 without pairs
};

// A region, or a confirmation of one, that the normal gate rejected:
// `frame`'s matches against `match_frame`'s window.
struct NormalRejection {
    map::FrameId frame;
    map::FrameId match_frame;
    NormalAgreement agreement;
};

// How far a detection got: the candidates, and per gate the most any of them
// reached.
struct CommonRegionStats {
    std::vector<map::FrameId> candidates;  // after DetectNBestCandidates
    size_t bow_matches = 0;
    size_t ransac_inliers = 0;
    size_t coarse_projection_matches = 0;  // with the RANSAC Sim3
    size_t optimized_inliers = 0;
    size_t projection_matches = 0;         // with the optimized Sim3
    // Candidates and covisible confirmations rejected by the normal gate.
    std::vector<NormalRejection> normal_rejections;
};

// A Frame of map `map` that DetectNBestCandidates chose.
struct RegionCandidate {
    map::MapId map{0};
    map::FrameId frame;
};

// ORB-SLAM3 DetectNBestCandidates' two lists, each by group score
// descending: hits in the query's own map (the active map) and hits in the
// other (inactive) maps.
struct RegionCandidates {
    std::vector<RegionCandidate> loop;
    std::vector<RegionCandidate> merge;
};

// A place the query Frame shares with earlier Frames that are not covisible
// with it: in its own map (a loop) or in another (a merge).
struct CommonRegion {
    // The candidate Frame the similarity refers to (ORB-SLAM3
    // pMostBoWMatchesKF, which is the candidate), and its map.
    map::FrameId match_frame;
    map::MapId match_map{0};
    // ORB-SLAM3 Scm: x_query = S_query_match * x_match in camera
    // coordinates; S_query_world = S_query_match composed with the match
    // Frame's T_cw is the query's corrected pose.
    geometry::Sim3 S_query_match;
    geometry::Sim3 S_query_world;
    // Per query (left) feature, the match side's point it matched by
    // projection with S_query_world, or none.
    std::vector<std::optional<map::MapPointId>> matches;
    size_t num_matches = 0;
    // Covisible Frames of the query that confirmed the region (at most
    // required_coincidences).
    size_t num_coincidences = 0;
    // The normal gate's measurement over `matches`.
    NormalAgreement normals;
};

struct CommonRegionResult {
    std::optional<CommonRegion> region;
    CommonRegionStats stats;
};

// ORB-SLAM3 DetectNBestCandidates for Frame `query_frame` of the atlas'
// active map: the hits of every live map's FrameDatabase that are neither the
// query nor covisible with it (covisibility is within the query's own map),
// sharing more than int(min_common_words_ratio x max) words, the max over
// all those hits; each hit scores as a group with its top num_covisibles
// covisible Frames that are hits too (in its own map), and gives the group's
// best-scoring Frame. By group score descending (ties by the hits' score,
// then FrameId), each Frame once: a Frame of the active map goes to `loop`,
// one of another map to `merge`, each list up to num_candidates. Removed maps
// are not in the atlas, so never searched. Empty without an active map or
// the query Frame.
template <typename TFeatureExtractor>
[[nodiscard]] RegionCandidates DetectNBestCandidates(const tracking::AtlasReplica<TFeatureExtractor>& atlas,
                                                     map::FrameId query_frame, const LoopCloserOptions& options);

// ORB-SLAM3 DetectCommonRegionsFromBoW, visual only, for Frame `query_frame`
// of the atlas' active map against `candidates` (one of
// DetectNBestCandidates' lists). Each candidate is verified against its own
// map's replica (the match side), which is the active map for a loop and an
// inactive map for a merge; the same gates for both:
//   1. per candidate, its window (skipped if any window Frame is covisible
//      with the query): SearchByBoW from the query's point-observing features
//      to each window Frame's points, each query feature and point once; at
//      least min_bow_matches;
//   2. SolveSim3Ransac between the query's points and the matched points in
//      the candidate's camera; pixels are each point's projection in its own
//      camera (ORB-SLAM3 mvP1im1 / mvP2im2); scale fixed for a binocular
//      query, free for a monocular one (two mono maps have unrelated scales);
//      at least min_sim3_inliers;
//   3. the window's points projected into the query with the RANSAC Sim3
//      (features::SearchPointsBySim3, a fresh match list); at least
//      min_coarse_projection; then OptimizeSim3 over those matches; at least
//      min_optimized inliers;
//   4. the window's points projected with the optimized Sim3; at least
//      min_projection matches;
//   5. the query's top num_covisibles covisible Frames, until
//      required_coincidences of them confirm, each by projection of the
//      window's points with its pose relative to the query
//      (confirmation_projection).
// For a loop, steps 4 and 5 also need the normal gate
// (max_normal_disagreement); its rejections are listed in the stats. A merge
// skips it (its normals are in two unrelated worlds) and relies on
// ORB-SLAM3's gates; CommonRegion::normals still measures it.
// A Sim3 that is not finite or has a scale <= 0 drops the candidate. The
// candidate with the most matches in step 4 wins (the first on a tie).
// Candidates of a map that is gone are skipped. Deterministic.
template <typename TFeatureExtractor>
[[nodiscard]] CommonRegionResult DetectCommonRegion(const tracking::AtlasReplica<TFeatureExtractor>& atlas,
                                                    map::FrameId query_frame,
                                                    const std::vector<RegionCandidate>& candidates,
                                                    const LoopCloserOptions& options);

// Loop detection within the single map `replica`: DetectNBestCandidates'
// loop list over that map alone, then DetectCommonRegion on it.
template <typename TFeatureExtractor>
[[nodiscard]] CommonRegionResult DetectCommonRegion(const tracking::MapReplica<TFeatureExtractor>& replica,
                                                    map::FrameId query_frame, const LoopCloserOptions& options);

// ORB-SLAM3 DetectAndReffineSim3FromLastKF: confirms the region of
// `match_frame` of `match_replica` (the query's own map for a loop, another
// for a merge) in a later query `query_frame` of `replica` from the guess
// S_query_world (match-side world to query camera): projection of the match
// Frame's window points (confirmation_projection), OptimizeSim3 over the
// matched query points (more than refine_min_optimized inliers), projection
// with the refined Sim3 (refine_min_projection), then, for a loop (the same
// map), the normal gate on those matches. None if a gate fails or a Frame is
// unknown; a normal rejection is added to `out_stats` if given.
template <typename TFeatureExtractor>
[[nodiscard]] std::optional<CommonRegion> RefineCommonRegion(
    const tracking::MapReplica<TFeatureExtractor>& replica, map::FrameId query_frame,
    const tracking::MapReplica<TFeatureExtractor>& match_replica, map::FrameId match_frame,
    const geometry::Sim3& S_query_world, const LoopCloserOptions& options, CommonRegionStats* out_stats = nullptr);

} // namespace voyahslam::loop_closing

#endif // VOYAHSLAM_LOOP_CLOSING_COMMON_REGION_H
