#ifndef VOYAHSLAM_FEATURES_ROTATION_HISTOGRAM_H
#define VOYAHSLAM_FEATURES_ROTATION_HISTOGRAM_H

#include <array>
#include <cmath>
#include <cstddef>
#include <numbers>
#include <vector>
#include <Eigen/Core>

namespace voyahslam::features {

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

// Rotation-consistency check of a set of matches (ORB-SLAM3's rotHist): each
// match adds its keypoint angle difference; matches outside the three fullest
// bins are inconsistent.
class RotationHistogram {
public:
    explicit RotationHistogram(size_t bins) : histogram_(bins) {}

    // rotation: angle difference in radians, any range; index: caller's match id.
    void Add(double rotation, Eigen::Index index) {
        constexpr double kTwoPi = 2.0 * std::numbers::pi;
        const size_t bins = histogram_.size();
        rotation = std::fmod(rotation, kTwoPi);
        if (rotation < 0.0) {
            rotation += kTwoPi;
        }
        const auto bin = static_cast<size_t>(std::llround(rotation * static_cast<double>(bins) / kTwoPi)) % bins;
        histogram_[bin].push_back(index);
    }

    // Indices added to bins other than the three fullest.
    [[nodiscard]] std::vector<Eigen::Index> Inconsistent() const {
        const std::array<int, 3> kept = detail::ThreeLargestBins(histogram_);
        std::vector<Eigen::Index> inconsistent;
        for (size_t b = 0; b < histogram_.size(); ++b) {
            const int bin = static_cast<int>(b);
            if (bin == kept[0] || bin == kept[1] || bin == kept[2]) {
                continue;
            }
            inconsistent.insert(inconsistent.end(), histogram_[b].begin(), histogram_[b].end());
        }
        return inconsistent;
    }

private:
    std::vector<std::vector<Eigen::Index>> histogram_;
};

}  // namespace voyahslam::features

#endif  // VOYAHSLAM_FEATURES_ROTATION_HISTOGRAM_H
