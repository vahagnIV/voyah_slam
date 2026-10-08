#ifndef VOYAHSLAM_FEATURES_BOW_MATCHER_H
#define VOYAHSLAM_FEATURES_BOW_MATCHER_H

#include <cassert>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <optional>
#include <span>
#include <vector>
#include <voyahslam/bow/bow_vector.h>
#include <voyahslam/features/measurement.h>
#include <voyahslam/features/rotation_histogram.h>
#include <voyahslam/map/map_point_id.h>

namespace voyahslam::features {

struct BowMatchOptions {
    int th_low = 50;                // max descriptor distance (ORB-SLAM3 TH_LOW)
    double nn_ratio = 0.75;         // best < nn_ratio * second best (0.7 for reference-Frame tracking)
    bool check_orientation = true;  // rotation-histogram consistency
    size_t histogram_bins = 12;     // 12 bins of 30°
};

// ORB-SLAM3 ORBmatcher::SearchByBoW(KeyFrame*, Frame&): matches the current
// frame against one map Frame. frame_points[i] is the live point the Frame's
// feature i observes, or nullopt; features without one are skipped. Only
// features filed under a shared vocabulary node are compared (the two sorted
// node lists are walked side by side); a Frame feature's best / second best
// distance are taken over the current features under that node. Accepted when
// best <= th_low and best < nn_ratio * second best. A current feature already
// claimed keeps its first claimant (Frame features visited in node order).
// Then rotation-histogram consistency (top 3 bins), when enabled.
//
// Returns, per current feature, the matched MapPointId or nullopt.
template <typename TFeatureExtractor>
[[nodiscard]] std::vector<std::optional<map::MapPointId>> SearchByBoW(
    const TMonocularMeasurement<TFeatureExtractor>& frame_measurement, const bow::FeatureVector& frame_features,
    const std::vector<std::optional<map::MapPointId>>& frame_points,
    const TMonocularMeasurement<TFeatureExtractor>& current, const bow::FeatureVector& current_features,
    const BowMatchOptions& options) {
    using TDescriptor = typename TMonocularMeasurement<TFeatureExtractor>::TDescriptor;
    const Eigen::Index num_frame = frame_measurement.points().cols();
    const Eigen::Index num_current = current.points().cols();
    assert(frame_points.size() == static_cast<size_t>(num_frame));
    std::vector<std::optional<map::MapPointId>> result(static_cast<size_t>(num_current));
    std::vector<Eigen::Index> frame_of(static_cast<size_t>(num_current), -1);  // current → Frame feature

    constexpr double kInfinity = std::numeric_limits<double>::infinity();
    const auto match_node = [&](std::span<const uint32_t> frame_under, std::span<const uint32_t> current_under) {
        for (const uint32_t i : frame_under) {
            if (!frame_points[i].has_value()) {
                continue;
            }
            const TDescriptor d_frame = frame_measurement.descriptors().col(i);
            double best = kInfinity;
            double second_best = kInfinity;
            Eigen::Index best_index = -1;
            for (const uint32_t j : current_under) {
                if (frame_of[j] >= 0) {
                    continue;  // already matched
                }
                const double distance =
                    static_cast<double>(TFeatureExtractor::Distance(d_frame, current.descriptors().col(j)));
                if (distance < best) {
                    second_best = best;
                    best = distance;
                    best_index = j;
                } else if (distance < second_best) {
                    second_best = distance;
                }
            }
            if (best_index >= 0 && best <= static_cast<double>(options.th_low) &&
                best < options.nn_ratio * second_best) {
                frame_of[best_index] = i;
            }
        }
    };

    size_t ia = 0;
    size_t ib = 0;
    while (ia < frame_features.nodes.size() && ib < current_features.nodes.size()) {
        if (frame_features.nodes[ia] < current_features.nodes[ib]) {
            ++ia;
        } else if (current_features.nodes[ib] < frame_features.nodes[ia]) {
            ++ib;
        } else {
            match_node(frame_features.FeaturesOf(ia), current_features.FeaturesOf(ib));
            ++ia;
            ++ib;
        }
    }

    if (options.check_orientation) {
        RotationHistogram histogram(options.histogram_bins);
        for (Eigen::Index j = 0; j < num_current; ++j) {
            if (frame_of[j] >= 0) {
                histogram.Add(static_cast<double>(frame_measurement.angles()(frame_of[j])) -
                                  static_cast<double>(current.angles()(j)),
                              j);
            }
        }
        for (const Eigen::Index j : histogram.Inconsistent()) {
            frame_of[j] = -1;
        }
    }

    for (Eigen::Index j = 0; j < num_current; ++j) {
        if (frame_of[j] >= 0) {
            result[j] = frame_points[frame_of[j]];
        }
    }
    return result;
}

}  // namespace voyahslam::features

#endif  // VOYAHSLAM_FEATURES_BOW_MATCHER_H
