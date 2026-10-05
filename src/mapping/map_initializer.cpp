#include <algorithm>
#include <cassert>
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
#include <voyahslam/optimization/bundle_adjustment.h>

#include "mapping/paired_inliers.h"

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

    return TryInitializeBinocular(std::get<TBinocularMeasurement>(std::move(measurement)));
}

template <typename TFeatureExtractor>
std::optional<InitializationResult<TFeatureExtractor>>
MapInitializer<TFeatureExtractor>::TryInitializeBinocular(TBinocularMeasurement measurement) {
    Reset();
    const Eigen::Index num_features = measurement.left().points().cols();
    if (static_cast<size_t>(num_features) <= options_.stereo_min_features) {
        log_->debug("Stereo initialization: {} features (need > {})", num_features,
                    options_.stereo_min_features);
        return std::nullopt;
    }
    // The Tracker stereo-matches before forwarding; unmatched means no depths.
    assert(measurement.stereo_matched());
    if (!measurement.stereo_matched()) {
        log_->debug("Stereo initialization: measurement is not stereo-matched");
        return std::nullopt;
    }

    TResult result;
    for (Eigen::Index i = 0; i < num_features; ++i) {
        const precision_t depth = measurement.depth(static_cast<int>(i));
        if (depth > 0.0) {
            result.map_points.push_back(
                {depth * measurement.left().normalized_point(static_cast<int>(i)), i, -1});
        }
    }
    if (result.map_points.size() < options_.stereo_min_points) {
        log_->debug("Stereo initialization: {} stereo points (need {})", result.map_points.size(),
                    options_.stereo_min_points);
        return std::nullopt;
    }
    result.keyframes.push_back({geometry::RigidTransformation::Identity(), std::move(measurement)});
    return result;
}

template <typename TFeatureExtractor>
std::optional<InitializationResult<TFeatureExtractor>>
MapInitializer<TFeatureExtractor>::TryInitializeMonocular(TMonocularMeasurement measurement) {
    if (static_cast<size_t>(measurement.points().cols()) <= options_.min_features) {
        log_->debug("Mono initialization: {} features (need > {}); reference dropped",
                    measurement.points().cols(), options_.min_features);
        Reset();
        return std::nullopt;
    }
    if (!reference_) {
        log_->debug("Mono initialization: new reference ({} features)", measurement.points().cols());
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
        log_->debug("Mono initialization: {} matches (need {}); current frame becomes the reference",
                    first.size(), options_.min_matches);
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
        log_->debug("Mono initialization: two-view reconstruction failed ({} matches); keeping the reference",
                    first.size());
        return std::nullopt;
    }

    std::vector<Eigen::Index> triangulated;
    for (Eigen::Index c = 0; c < correspondences.size(); ++c) {
        if (reconstruction.triangulated(c)) {
            triangulated.push_back(c);
        }
    }
    if (triangulated.empty()) {
        log_->debug("Mono initialization: no triangulated points");
        return std::nullopt;
    }
    // Unscaled; camera 1 (the reference) is the world frame.
    geometry::TPoints3DF points = reconstruction.points(Eigen::all, triangulated);
    geometry::RigidTransformation T_21 = reconstruction.T_21;

    if (options_.bundle_adjust) {
        // Raw pixels through each view's camera; sigma in pixels.
        const auto sigma = [this](const TMonocularMeasurement& view, Eigen::Index feature) {
            return options_.sigma_px *
                   std::pow(options_.level_scale, static_cast<double>(view.levels()(feature)));
        };
        optimization::BaProblem problem;
        problem.keyframe_poses = {geometry::RigidTransformation::Identity(), T_21};
        problem.keyframe_fixed = {true, false};
        problem.keyframe_cameras = {reference_->camera(), measurement.camera()};
        problem.points = points;
        problem.observations.reserve(2 * triangulated.size());
        for (size_t p = 0; p < triangulated.size(); ++p) {
            const Eigen::Index c = triangulated[p];
            problem.observations.push_back(
                {0, p, reference_->distorted_points().col(first[c]), sigma(*reference_, first[c])});
            problem.observations.push_back(
                {1, p, measurement.distorted_points().col(second[c]), sigma(measurement, second[c])});
        }
        const optimization::BaResult adjusted =
            optimization::BundleAdjust(problem, options_.bundle_adjustment);
        if (!adjusted.success) {
            log_->debug("Initialization BA failed; keeping the reference");
            return std::nullopt;
        }

        // Observations are pushed in pairs: point p owns 2p (view 1) and 2p + 1 (view 2).
        const std::vector<Eigen::Index> survivors = PairedInliers(adjusted.inlier, triangulated.size());
        std::vector<Eigen::Index> kept;
        kept.reserve(survivors.size());
        for (const Eigen::Index p : survivors) {
            kept.push_back(triangulated[static_cast<size_t>(p)]);
        }
        log_->debug("Initialization BA: cost {:.4g} -> {:.4g} in {} iterations, "
                    "{} outliers removed, {} points survive",
                    adjusted.initial_cost, adjusted.final_cost, adjusted.iterations,
                    triangulated.size() - kept.size(), kept.size());
        points = adjusted.points(Eigen::all, survivors);
        T_21 = adjusted.keyframe_poses[1];
        triangulated = std::move(kept);
        if (triangulated.empty()) {
            log_->debug("Initialization rejected: no points survive BA");
            return std::nullopt;
        }
    }

    std::vector<double> depths(points.row(2).begin(), points.row(2).end());
    const auto median = depths.begin() + static_cast<std::ptrdiff_t>(depths.size() / 2);
    std::nth_element(depths.begin(), median, depths.end());
    const double median_depth = *median;
    if (!std::isfinite(median_depth) || median_depth <= 0.0) {
        log_->debug("Initialization rejected: invalid median depth {}", median_depth);
        return std::nullopt;
    }
    if (options_.bundle_adjust && triangulated.size() < options_.min_points_after_ba) {
        log_->debug("Initialization rejected: {} points after BA, {} required", triangulated.size(),
                    options_.min_points_after_ba);
        return std::nullopt;
    }
    const double scale = 1.0 / median_depth;

    TResult result;
    result.map_points.reserve(triangulated.size());
    for (size_t p = 0; p < triangulated.size(); ++p) {
        const Eigen::Index c = triangulated[p];
        result.map_points.push_back(
            {scale * points.col(static_cast<Eigen::Index>(p)), first[c], second[c]});
    }
    T_21 = geometry::RigidTransformation(T_21.rotation(), scale * T_21.translation());
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
