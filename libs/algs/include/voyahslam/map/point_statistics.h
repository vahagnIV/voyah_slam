#ifndef VOYAHSLAM_MAP_POINT_STATISTICS_H
#define VOYAHSLAM_MAP_POINT_STATISTICS_H

#include <algorithm>
#include <cassert>
#include <cstddef>
#include <vector>
#include <voyahslam/defs.h>

namespace voyahslam::map {

/// Unit mean of the unit vectors (point - C) over the camera centres C.
/// Returns the zero vector when `camera_centres` is empty or the mean cancels out.
[[nodiscard]] geometry::TPoint3DF MeanViewingDirection(
    const geometry::TPoint3DF& point, const std::vector<geometry::TPoint3DF>& camera_centres);

/// The descriptor with the least median Hamming distance to all the others
/// (the medoid). Ties go to the lowest index. Precondition: `descriptors` is non-empty.
template <typename TFeatureExtractor>
typename TFeatureExtractor::TDescriptor DistinctiveDescriptor(
    const std::vector<typename TFeatureExtractor::TDescriptor>& descriptors) {
    assert(!descriptors.empty());
    const size_t n = descriptors.size();
    size_t best = 0;
    double best_median = 0.0;
    std::vector<int> distances;
    distances.reserve(n);
    for (size_t i = 0; i < n; ++i) {
        distances.clear();
        for (size_t j = 0; j < n; ++j) {
            if (j != i) {
                distances.push_back(
                    static_cast<int>(TFeatureExtractor::Distance(descriptors[i], descriptors[j])));
            }
        }
        double median = 0.0;  // a single descriptor has no others
        if (!distances.empty()) {
            std::sort(distances.begin(), distances.end());
            const size_t m = distances.size();
            median = 0.5 * (distances[(m - 1) / 2] + distances[m / 2]);
        }
        if (i == 0 || median < best_median) {
            best = i;
            best_median = median;
        }
    }
    return descriptors[best];
}

}  // namespace voyahslam::map

#endif  // VOYAHSLAM_MAP_POINT_STATISTICS_H
