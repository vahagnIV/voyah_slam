#pragma once

#include <Eigen/Core>

#include <cassert>
#include <cstddef>
#include <vector>

namespace voyahslam::mapping {

/// Bundle adjustment of the initialization pushes two observations per point: point p owns
/// observation 2p (first view) and 2p + 1 (second view). Returns the indices p of the points
/// whose two observations are both inliers, in increasing order.
[[nodiscard]] inline std::vector<Eigen::Index> PairedInliers(const std::vector<bool>& inlier,
                                                             size_t num_points) {
    assert(inlier.size() == 2 * num_points);
    std::vector<Eigen::Index> survivors;
    survivors.reserve(num_points);
    for (size_t p = 0; p < num_points; ++p) {
        if (inlier[2 * p] && inlier[2 * p + 1]) {
            survivors.push_back(static_cast<Eigen::Index>(p));
        }
    }
    return survivors;
}

}  // namespace voyahslam::mapping
