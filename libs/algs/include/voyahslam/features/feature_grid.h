#ifndef VOYAHSLAM_FEATURES_FEATURE_GRID_H
#define VOYAHSLAM_FEATURES_FEATURE_GRID_H

#include <vector>
#include <voyahslam/defs.h>
#include <voyahslam/features/image_features.h>

namespace voyahslam::features {

// Uniform grid over the bounding box of a keypoint set, for fast square-window queries.
class FeatureGrid {
public:
    FeatureGrid(const geometry::TPoints2DF& points, const TLevels& levels, int cols = 64, int rows = 48);

    // Indices of points with |dx| <= radius, |dy| <= radius from center and
    // min_level <= level <= max_level. Order is unspecified.
    [[nodiscard]] std::vector<Eigen::Index> FeaturesInArea(const geometry::TPoint2DF& center, double radius,
                                                           int min_level, int max_level) const;

private:
    const geometry::TPoints2DF points_;
    const TLevels levels_;
    int cols_;
    int rows_;
    double min_x_ = 0.0;
    double min_y_ = 0.0;
    double cell_width_ = 1.0;
    double cell_height_ = 1.0;
    std::vector<std::vector<Eigen::Index>> cells_;
};

}  // namespace voyahslam::features

#endif  // VOYAHSLAM_FEATURES_FEATURE_GRID_H
