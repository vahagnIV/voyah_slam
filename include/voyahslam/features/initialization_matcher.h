#ifndef VOYAHSLAM_FEATURES_INITIALIZATION_MATCHER_H
#define VOYAHSLAM_FEATURES_INITIALIZATION_MATCHER_H

#include <array>
#include <cmath>
#include <cstddef>
#include <limits>
#include <numbers>
#include <vector>
#include <voyahslam/defs.h>
#include <voyahslam/features/feature_grid.h>
#include <voyahslam/features/measurement.h>

namespace voyahslam::features {

struct InitializationMatcherOptions {
    double window_px = 100.0;     // square half-size around the last matched position
    double max_distance = 50.0;   // descriptor distance threshold (double so SIFT's float L2 fits)
    double nn_ratio = 0.9;        // best < nn_ratio * second best
    bool check_orientation = true;
    size_t histogram_bins = 12;   // 12 bins of 30° — ORB-SLAM3's effective resolution (HISTO_LENGTH = 30 applied as 1/30 per degree)
    int max_reference_level = 0;  // only reference features with level <= this
};

namespace detail {

// ORB-SLAM3 ORBmatcher::ComputeThreeMaxima: indices of the three fullest bins;
// the 2nd/3rd are dropped (-1) when below 0.1 x the largest.
inline std::array<int, 3> ThreeLargestBins(const std::vector<std::vector<Eigen::Index>>& histogram) {
    std::array<int, 3> ind{-1, -1, -1};
    size_t max1 = 0, max2 = 0, max3 = 0;
    for (size_t i = 0; i < histogram.size(); ++i) {
        const size_t s = histogram[i].size();
        const int bin = static_cast<int>(i);
        if (s > max1) {
            max3 = max2;
            max2 = max1;
            max1 = s;
            ind[2] = ind[1];
            ind[1] = ind[0];
            ind[0] = bin;
        } else if (s > max2) {
            max3 = max2;
            max2 = s;
            ind[2] = ind[1];
            ind[1] = bin;
        } else if (s > max3) {
            max3 = s;
            ind[2] = bin;
        }
    }
    if (static_cast<double>(max2) < 0.1 * static_cast<double>(max1)) {
        ind[1] = -1;
        ind[2] = -1;
    } else if (static_cast<double>(max3) < 0.1 * static_cast<double>(max1)) {
        ind[2] = -1;
    }
    return ind;
}

}  // namespace detail

// ORB-SLAM3 ORBmatcher::SearchForInitialization. Returns, per reference
// feature, the matched current feature index or -1. out_last_matched (one entry
// per reference feature) seeds the search windows and is overwritten with the
// matched current point for every surviving match.
template <typename TFeatureExtractor>
[[nodiscard]] std::vector<Eigen::Index> MatchForInitialization(
    const TMonocularMeasurement<TFeatureExtractor>& reference,
    const TMonocularMeasurement<TFeatureExtractor>& current,
    const FeatureGrid& current_grid,
    std::vector<geometry::TPoint2DF>& out_last_matched,
    const InitializationMatcherOptions& options) {
    const Eigen::Index num_reference = reference.points().cols();
    const Eigen::Index num_current = current.points().cols();
    std::vector<Eigen::Index> matches(num_reference, -1);
    constexpr double kInfinity = std::numeric_limits<double>::infinity();
    std::vector<double> matched_distance(num_current, kInfinity);
    std::vector<Eigen::Index> reverse_matches(num_current, -1);

    const size_t bins = options.histogram_bins;
    std::vector<std::vector<Eigen::Index>> histogram(bins);
    constexpr double kTwoPi = 2.0 * std::numbers::pi;

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
            double rot = std::fmod(static_cast<double>(reference.angles()(i)) -
                                       static_cast<double>(current.angles()(best_index)),
                                   kTwoPi);
            if (rot < 0.0) {
                rot += kTwoPi;
            }
            const auto bin = static_cast<size_t>(std::llround(rot * static_cast<double>(bins) / kTwoPi)) % bins;
            histogram[bin].push_back(i);
        }
    }

    if (options.check_orientation) {
        const std::array<int, 3> kept = detail::ThreeLargestBins(histogram);
        for (size_t b = 0; b < bins; ++b) {
            if (static_cast<int>(b) == kept[0] || static_cast<int>(b) == kept[1] ||
                static_cast<int>(b) == kept[2]) {
                continue;
            }
            for (const Eigen::Index i : histogram[b]) {
                matches[i] = -1;
            }
        }
    }

    for (Eigen::Index i = 0; i < num_reference; ++i) {
        if (matches[i] >= 0) {
            out_last_matched[i] = current.points().col(matches[i]);
        }
    }
    return matches;
}

}  // namespace voyahslam::features

#endif  // VOYAHSLAM_FEATURES_INITIALIZATION_MATCHER_H
