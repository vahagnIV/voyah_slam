#ifndef VOYAHSLAM_FEATURES_INITIALIZATION_MATCHER_H
#define VOYAHSLAM_FEATURES_INITIALIZATION_MATCHER_H

#include <cassert>
#include <cstddef>
#include <limits>
#include <vector>
#include <voyahslam/defs.h>
#include <voyahslam/features/feature_grid.h>
#include <voyahslam/features/measurement.h>
#include <voyahslam/features/rotation_histogram.h>

namespace voyahslam::features {

struct InitializationMatcherOptions {
    double window_px = 100.0;     // square half-size around the last matched position (raw px)
    double max_distance = 50.0;   // descriptor distance threshold (double so a float-distance extractor fits)
    double nn_ratio = 0.9;        // best < nn_ratio * second best
    bool check_orientation = true;
    size_t histogram_bins = 12;   // 12 bins of 30° — ORB-SLAM3's effective resolution (HISTO_LENGTH = 30 applied as 1/30 per degree)
    int max_reference_level = 0;  // only reference features with level <= this
};

// ORB-SLAM3 ORBmatcher::SearchForInitialization, in raw (distorted) pixels.
// Returns, per reference feature, the matched current feature index or -1.
// current_grid is built on current.distorted_points(). out_last_matched (one
// entry per reference feature, raw px; initially the reference's
// distorted_points()) seeds the search windows and is overwritten with the
// matched current raw point for every surviving match.
template <typename TFeatureExtractor>
[[nodiscard]] std::vector<Eigen::Index> MatchForInitialization(
    const TMonocularMeasurement<TFeatureExtractor>& reference,
    const TMonocularMeasurement<TFeatureExtractor>& current,
    const FeatureGrid& current_grid,
    std::vector<geometry::TPoint2DF>& out_last_matched,
    const InitializationMatcherOptions& options) {
    const Eigen::Index num_reference = reference.points().cols();
    const Eigen::Index num_current = current.points().cols();
    assert(options.histogram_bins > 0 || !options.check_orientation);
    assert(static_cast<Eigen::Index>(out_last_matched.size()) == num_reference);
    std::vector<Eigen::Index> matches(num_reference, -1);
    constexpr double kInfinity = std::numeric_limits<double>::infinity();
    std::vector<double> matched_distance(num_current, kInfinity);
    std::vector<Eigen::Index> reverse_matches(num_current, -1);

    RotationHistogram histogram(options.check_orientation ? options.histogram_bins : 1);

    for (Eigen::Index i = 0; i < num_reference; ++i) {
        const int level = reference.levels()(i);
        if (level > options.max_reference_level) {
            continue;
        }
        const std::vector<Eigen::Index> candidates =
            current_grid.FeaturesInArea(out_last_matched[i], options.window_px, level, level);
        if (candidates.empty()) {
            continue;
        }

        const typename TMonocularMeasurement<TFeatureExtractor>::TDescriptor d_ref =
            reference.descriptors().col(i);
        double best = kInfinity;
        double second_best = kInfinity;
        Eigen::Index best_index = -1;
        for (const Eigen::Index j : candidates) {
            const typename TMonocularMeasurement<TFeatureExtractor>::TDescriptor d_cur =
                current.descriptors().col(j);
            const double distance = static_cast<double>(TFeatureExtractor::Distance(d_ref, d_cur));
            if (matched_distance[j] <= distance) {
                continue;
            }
            if (distance < best) {
                second_best = best;
                best = distance;
                best_index = j;
            } else if (distance < second_best) {
                second_best = distance;
            }
        }

        if (best_index < 0 || best > options.max_distance || !(best < second_best * options.nn_ratio)) {
            continue;
        }
        if (reverse_matches[best_index] >= 0) {
            matches[reverse_matches[best_index]] = -1;
        }
        matches[i] = best_index;
        reverse_matches[best_index] = i;
        matched_distance[best_index] = best;

        if (options.check_orientation) {
            histogram.Add(static_cast<double>(reference.angles()(i)) -
                              static_cast<double>(current.angles()(best_index)),
                          i);
        }
    }

    if (options.check_orientation) {
        for (const Eigen::Index i : histogram.Inconsistent()) {
            matches[i] = -1;
        }
    }

    for (Eigen::Index i = 0; i < num_reference; ++i) {
        if (matches[i] >= 0) {
            out_last_matched[i] = current.distorted_points().col(matches[i]);
        }
    }
    return matches;
}

}  // namespace voyahslam::features

#endif  // VOYAHSLAM_FEATURES_INITIALIZATION_MATCHER_H
