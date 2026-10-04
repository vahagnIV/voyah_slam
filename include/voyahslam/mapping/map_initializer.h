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

namespace voyahslam::mapping {

struct MapInitializerOptions {
    size_t min_features = 100;  // a monocular frame needs more features than this
    size_t min_matches = 100;   // matches needed to attempt a two-view reconstruction
    double sigma_px = 1.0;      // keypoint noise std-dev, px
    features::InitializationMatcherOptions matcher;
    geometry::TwoViewOptions reconstruction;
};

template <typename TFeatureExtractor>
struct InitialKeyFrame {
    geometry::RigidTransformation T_cw;
    features::TMeasurement<TFeatureExtractor> measurement;
};

struct InitialMapPoint {
    geometry::TPoint3DF position;    // world (= camera 1) frame, scaled
    Eigen::Index feature_in_first;   // feature index in keyframes[0]'s measurement
    Eigen::Index feature_in_second;  // feature index in keyframes[1]'s measurement
};

template <typename TFeatureExtractor>
struct InitializationResult {
    std::vector<InitialKeyFrame<TFeatureExtractor>> keyframes;
    std::vector<InitialMapPoint> map_points;  // empty for the stereo stub
};

// Builds the first keyframes (and, for monocular input, map points) of a map.
//   - binocular: initializes at once with one identity-pose keyframe and no
//     map points (placeholder); also drops any monocular reference;
//   - monocular (ORB-SLAM3 MonocularInitialization): the first frame with more
//     than min_features features becomes the reference; each later frame is
//     matched against it (MatchForInitialization) and, with at least
//     min_matches matches, reconstructed from two views. On success the
//     reference becomes the world frame (T_cw = identity), the scene is scaled
//     to median depth 1, and both frames become keyframes. A frame with too
//     few features drops the reference; one with too few matches replaces it.
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
    std::optional<TResult> TryInitializeMonocular(TMonocularMeasurement measurement);
    void SetReference(TMonocularMeasurement measurement);

    MapInitializerOptions options_;
    std::optional<TMonocularMeasurement> reference_;
    // Per reference feature: where its match was last seen (seeds the search
    // windows); the reference's own points until a match moves it.
    std::vector<geometry::TPoint2DF> last_matched_;
};

} // namespace voyahslam::mapping

#endif // VOYAHSLAM_MAPPING_MAP_INITIALIZER_H
