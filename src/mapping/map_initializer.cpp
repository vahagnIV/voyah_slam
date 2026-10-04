#include <algorithm>
#include <cmath>
#include <cstddef>
#include <utility>
#include <variant>
#include <voyahslam/features/feature_grid.h>
#include <voyahslam/features/initialization_matcher.h>
#include <voyahslam/features/instantiation.h>
#include <voyahslam/geometry/two_view_correspondences.h>
#include <voyahslam/geometry/two_view_reconstruction.h>
#include <voyahslam/mapping/map_initializer.h>

namespace voyahslam::mapping {

namespace {

template <typename TMonocularMeasurement>
std::vector<geometry::TPoint2DF> Points(const TMonocularMeasurement& measurement) {
    const geometry::TPoints2DF& points = measurement.points();
    std::vector<geometry::TPoint2DF> result;
    result.reserve(points.cols());
    for (Eigen::Index i = 0; i < points.cols(); ++i) {
        result.emplace_back(points.col(i));
    }
    return result;
}

} // namespace

template <typename TFeatureExtractor>
std::optional<InitializationResult<TFeatureExtractor>>
MapInitializer<TFeatureExtractor>::TryInitialize(TMeasurement measurement) {
    if (auto* monocular = std::get_if<TMonocularMeasurement>(&measurement)) {
        return TryInitializeMonocular(std::move(*monocular));
    }

    // Binocular placeholder: one identity-pose keyframe, no map points.
    Reset();
    TResult result;
    result.keyframes.push_back({geometry::RigidTransformation::Identity(), std::move(measurement)});
    return result;
}

template <typename TFeatureExtractor>
std::optional<InitializationResult<TFeatureExtractor>>
MapInitializer<TFeatureExtractor>::TryInitializeMonocular(TMonocularMeasurement measurement) {
    if (static_cast<size_t>(measurement.points().cols()) <= options_.min_features) {
        Reset();
        return std::nullopt;
    }
    if (!reference_) {
        SetReference(std::move(measurement));
        return std::nullopt;
    }

    // Like ORB-SLAM3, last_matched_ keeps the matched positions even when the
    // reconstruction below fails.
    const features::FeatureGrid grid(measurement.points(), measurement.levels());
    const std::vector<Eigen::Index> matches = features::MatchForInitialization<TFeatureExtractor>(
        *reference_, measurement, grid, last_matched_, options_.matcher);
    std::vector<Eigen::Index> first;
    std::vector<Eigen::Index> second;
    for (Eigen::Index i = 0; i < static_cast<Eigen::Index>(matches.size()); ++i) {
        if (matches[i] >= 0) {
            first.push_back(i);
            second.push_back(matches[i]);
        }
    }
    if (first.size() < options_.min_matches) {
        SetReference(std::move(measurement));
        return std::nullopt;
    }

    geometry::TwoViewCorrespondences correspondences;
    correspondences.x1 = reference_->normalized_points().template topRows<2>()(Eigen::all, first);
    correspondences.x2 = measurement.normalized_points().template topRows<2>()(Eigen::all, second);
    correspondences.sigma = options_.sigma_px / reference_->K()(0, 0);
    const geometry::TwoViewReconstruction reconstruction =
        geometry::TwoViewReconstructor(options_.reconstruction).Reconstruct(correspondences);
    if (!reconstruction.success) {
        return std::nullopt;
    }

    std::vector<Eigen::Index> triangulated;
    std::vector<double> depths;
    for (Eigen::Index c = 0; c < correspondences.size(); ++c) {
        if (reconstruction.triangulated(c)) {
            triangulated.push_back(c);
            depths.push_back(reconstruction.points(2, c));
        }
    }
    if (depths.empty()) {
        return std::nullopt;
    }
    const auto median = depths.begin() + static_cast<std::ptrdiff_t>(depths.size() / 2);
    std::nth_element(depths.begin(), median, depths.end());
    const double median_depth = *median;
    if (!std::isfinite(median_depth) || median_depth <= 0.0) {
        return std::nullopt;
    }
    const double scale = 1.0 / median_depth;

    TResult result;
    result.map_points.reserve(triangulated.size());
    for (const Eigen::Index c : triangulated) {
        result.map_points.push_back({scale * reconstruction.points.col(c), first[c], second[c]});
    }
    const geometry::RigidTransformation T_21(reconstruction.T_21.rotation(),
                                             scale * reconstruction.T_21.translation());
    result.keyframes.push_back({geometry::RigidTransformation::Identity(), std::move(*reference_)});
    result.keyframes.push_back({T_21, std::move(measurement)});
    Reset();
    return result;
}

template <typename TFeatureExtractor>
void MapInitializer<TFeatureExtractor>::SetReference(TMonocularMeasurement measurement) {
    reference_ = std::move(measurement);
    last_matched_ = Points(*reference_);
}

template <typename TFeatureExtractor>
void MapInitializer<TFeatureExtractor>::Reset() {
    reference_.reset();
    last_matched_.clear();
}

INSTANTIATE_CLASS(MapInitializer)

} // namespace voyahslam::mapping
