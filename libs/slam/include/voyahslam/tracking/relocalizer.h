#ifndef VOYAHSLAM_TRACKING_RELOCALIZER_H
#define VOYAHSLAM_TRACKING_RELOCALIZER_H

#include <cstddef>
#include <cstdint>
#include <vector>
#include <voyahslam/bow/frame_database.h>
#include <voyahslam/bow/vocabulary.h>
#include <voyahslam/features/measurement.h>
#include <voyahslam/features/projection_matcher.h>
#include <voyahslam/geometry/rigid_transformation.h>
#include <voyahslam/map/covisibility_index.h>
#include <voyahslam/map/frame_id.h>
#include <voyahslam/optimization/pose_optimization.h>
#include <voyahslam/tracking/map_replica.h>

namespace voyahslam::tracking {

// The loss protocol (ORB-SLAM3 RECENTLY_LOST) and relocalization in the
// current map (ORB-SLAM3 Relocalization), with a vocabulary only (spec
// sections 2 and 4).
struct RelocalizationOptions {
    // RecentlyLost for this long, in measurement timestamps, then Lost.
    double recently_lost_seconds = 5.0;
    // A map with at most this many Frames goes straight to Lost.
    size_t min_map_frames_to_relocalize = 10;

    // Database query: Frames sharing at least this fraction of the best
    // Frame's common words.
    double min_common_words_ratio = 0.8;
    // Candidate groups: a candidate plus its top group_neighbours covisible
    // Frames that are candidates too; groups scoring at least
    // min_group_score_ratio x the best group are kept.
    size_t group_neighbours = 10;
    double min_group_score_ratio = 0.75;
    // SearchByBoW against each candidate: ratio, and the minimum matches.
    double nn_ratio = 0.75;
    size_t min_bow_matches = 15;
    // P3P RANSAC: per candidate at most ransac_max_iterations iterations in
    // all, ransac_iterations_per_round of them per round.
    int ransac_max_iterations = 300;
    double ransac_confidence = 0.99;
    size_t ransac_min_inliers = 10;
    int ransac_iterations_per_round = 5;
    // Verification: fewer inliers after the first optimization drop the
    // candidate; success at min_inliers.
    size_t min_refined_inliers = 10;
    size_t min_inliers = 50;
    // Below min_inliers, the candidate's other points are projected
    // (first_pass_*), and again (second_pass_*) when the count is strictly
    // between min_second_pass_inliers and min_inliers: features::
    // SearchFromFrame with radius window * level_scale^level pixels and
    // descriptor distance <= max_distance (ORB-SLAM3's th and ORBdist).
    double first_pass_window = 10.0;
    double first_pass_max_distance = 100.0;
    size_t min_second_pass_inliers = 30;
    double second_pass_window = 3.0;
    double second_pass_max_distance = 64.0;
};

// The residual model of pose optimization against map points: raw pixels,
// sigma = sigma_px * level_scale^level.
struct PoseRefinement {
    optimization::PoseOptimizationOptions optimization;
    double sigma_px = 1.0;
    double level_scale = 1.2;
};

// A pose optimized against matches (per (left) feature, MapPointId value or
// -1), with the outliers dropped from the matches.
struct MatchedPose {
    geometry::RigidTransformation T_cw;
    std::vector<int64_t> matches;
    size_t num_matches = 0;  // alive matches optimized against
    size_t num_inliers = 0;
};

// Optimizes `initial_T_cw` against the alive matched points of `replica`
// (binocular measurements with their stereo matches) and drops the outliers
// and dead points from the matches. On an optimization failure the pose stays
// `initial_T_cw` and every match is dropped.
template <typename TFeatureExtractor>
[[nodiscard]] MatchedPose OptimizeMatchedPose(const MapReplica<TFeatureExtractor>& replica,
                                              const features::TMeasurement<TFeatureExtractor>& measurement,
                                              const geometry::RigidTransformation& initial_T_cw,
                                              std::vector<int64_t> matches,
                                              const PoseRefinement& refinement);

// Spec section 4.2 (ORB-SLAM3 DetectRelocalizationCandidates, after the
// database query): a hit's group score is its own score plus the scores of
// its top `group_neighbours` covisible Frames that are hits too. Each group
// with a score >= min_group_score_ratio x the best group gives its
// best-scoring Frame. Returns those Frames, once each, by group score
// descending (ties by the hits' order).
[[nodiscard]] std::vector<map::FrameId> RelocalizationCandidates(
    const std::vector<bow::DatabaseHit>& hits, const map::CovisibilityIndex& covisibility,
    size_t group_neighbours, double min_group_score_ratio);

struct RelocalizationResult {
    bool success = false;
    geometry::RigidTransformation T_cw;
    map::FrameId frame{};          // the candidate Frame relocalized against
    std::vector<int64_t> matches;  // per (left) feature: inlier MapPointId value or -1
    size_t inliers = 0;
    // How far the attempt got (also on failure).
    size_t num_candidates = 0;          // after grouping
    size_t num_matched_candidates = 0;  // with >= min_bow_matches SearchByBoW matches
    size_t num_hypotheses = 0;          // RANSAC poses that went to verification
    size_t best_verified_inliers = 0;   // most inliers a verification reached
};

// Relocalizes `measurement` in `replica` (spec sections 4.2-4.5):
//   1. the current frame's BoW (the Mapper's levels_up), queried in the
//      replica's Frame database; hits whose Frame is gone are skipped;
//   2. candidates grouped by covisibility (RelocalizationCandidates);
//   3. SearchByBoW against each candidate; fewer than min_bow_matches drop it;
//   4. rounds of P3P RANSAC on the left camera: each round gives every live
//      candidate up to ransac_iterations_per_round more iterations, as a
//      fresh SolvePnPRansac call with its own seed (from the measurement
//      timestamp, the candidate Frame and the round), until it has used
//      ransac_max_iterations;
//   5. a RANSAC pose is refined by OptimizePose over all of the candidate's
//      SearchByBoW matches; fewer than min_refined_inliers drop the candidate;
//   6. below min_inliers, the candidate Frame's other points are projected
//      (features::SearchFromFrame; `matcher` gives its level scale, ray-angle
//      limit and orientation check) and the pose optimized again, a second
//      time with the narrower window when the count is strictly between
//      min_second_pass_inliers and min_inliers.
// The first candidate reaching min_inliers wins. Deterministic. The
// measurement must be stereo-matched if binocular.
template <typename TFeatureExtractor>
[[nodiscard]] RelocalizationResult Relocalize(const MapReplica<TFeatureExtractor>& replica,
                                              const features::TMeasurement<TFeatureExtractor>& measurement,
                                              const bow::Vocabulary<TFeatureExtractor>& vocabulary,
                                              const RelocalizationOptions& options,
                                              const PoseRefinement& refinement,
                                              const features::ProjectionMatcherOptions& matcher);

} // namespace voyahslam::tracking

#endif // VOYAHSLAM_TRACKING_RELOCALIZER_H
