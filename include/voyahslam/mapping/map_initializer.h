#ifndef VOYAHSLAM_MAPPING_MAP_INITIALIZER_H
#define VOYAHSLAM_MAPPING_MAP_INITIALIZER_H

#include <cstddef>
#include <optional>
#include <utility>
#include <vector>
#include <voyahslam/defs.h>
#include <voyahslam/features/initialization_matcher.h>
#include <voyahslam/features/measurement.h>
#include <voyahslam/geometry/rigid_transformation.h>
#include <voyahslam/geometry/two_view_reconstruction.h>
#include <voyahslam/logging/logger.h>
#include <voyahslam/optimization/bundle_adjustment.h>

namespace voyahslam::mapping {

struct MapInitializerOptions {
    size_t min_features = 100;  // a monocular frame needs more features than this
    size_t min_matches = 100;   // matches needed to attempt a two-view reconstruction
    size_t stereo_min_features = 500;  // a binocular frame needs more left features than this
    size_t stereo_min_points = 100;    // left features with a stereo depth needed
    double sigma_px = 1.0;      // keypoint noise std-dev, px
    features::InitializationMatcherOptions matcher;
    geometry::TwoViewOptions reconstruction;
    // Monocular: bundle adjustment of the two-view reconstruction (before the
    // median-depth scaling); points with an outlier observation are dropped.
    bool bundle_adjust = true;
    // 5 + 40: the problem is unscaled (baseline 1, depths ~40), so the plain
    // stage needs ~40 iterations to converge.
    optimization::BundleAdjustmentOptions bundle_adjustment{.robust_iterations = 5,
                                                            .iterations = 40};
    double level_scale = 1.2;          // sigma_level = sigma_px * level_scale^level
    size_t min_points_after_ba = 50;   // survivors of the adjustment needed
};

template <typename TFeatureExtractor>
struct InitialKeyFrame {
    geometry::RigidTransformation T_cw;
    features::TMeasurement<TFeatureExtractor> measurement;
};

struct InitialMapPoint {
    geometry::TPoint3DF position;    // world (= keyframes[0]'s camera) frame
    Eigen::Index feature_in_first;   // feature index in keyframes[0]'s (left) image
    // Feature index in keyframes[1]'s measurement; -1 = no second keyframe
    // (stereo initialization).
    Eigen::Index feature_in_second;
};

template <typename TFeatureExtractor>
struct InitializationResult {
    std::vector<InitialKeyFrame<TFeatureExtractor>> keyframes;
    std::vector<InitialMapPoint> map_points;
};

// Builds the first keyframes and map points of a map.
//   - binocular (ORB-SLAM3 StereoInitialization): drops any monocular
//     reference. A stereo-matched frame with more than stereo_min_features
//     left features and at least stereo_min_points stereo depths becomes the
//     only keyframe (T_cw = identity); each depth gives a map point at
//     depth * (left normalized ray), metric scale, feature_in_second = -1;
//   - monocular (ORB-SLAM3 MonocularInitialization): the first frame with more
//     than min_features features becomes the reference; each later frame is
//     matched against it (MatchForInitialization) and, with at least
//     min_matches matches, reconstructed from two views. With bundle_adjust,
//     the reconstruction is refined by a two-keyframe bundle adjustment
//     (reference fixed), points with an outlier observation are dropped, and
//     fewer than min_points_after_ba survivors fail (the reference is kept).
//     On success the reference becomes the world frame (T_cw = identity), the
//     scene is scaled to median depth 1, and both frames become keyframes. A
//     frame with too few features drops the reference; one with too few
//     matches replaces it.
template <typename TFeatureExtractor>
class MapInitializer {
public:
    using TMeasurement = features::TMeasurement<TFeatureExtractor>;
    using TMonocularMeasurement = features::TMonocularMeasurement<TFeatureExtractor>;
    using TResult = InitializationResult<TFeatureExtractor>;

    explicit MapInitializer(MapInitializerOptions options = {}) : options_(std::move(options)) {}

    std::optional<TResult> TryInitialize(TMeasurement measurement);

    // Drops the reference measurement.
    void Reset();

private:
    using TBinocularMeasurement = features::TBinocularMeasurement<TFeatureExtractor>;

    std::optional<TResult> TryInitializeBinocular(TBinocularMeasurement measurement);
    std::optional<TResult> TryInitializeMonocular(TMonocularMeasurement measurement);
    void SetReference(TMonocularMeasurement measurement);

    MapInitializerOptions options_;
    std::optional<TMonocularMeasurement> reference_;
    // Per reference feature: where its match was last seen (seeds the search
    // windows); the reference's own points until a match moves it.
    std::vector<geometry::TPoint2DF> last_matched_;
    CLASS_LOGGER(MapInitializer);
};

} // namespace voyahslam::mapping

#endif // VOYAHSLAM_MAPPING_MAP_INITIALIZER_H
