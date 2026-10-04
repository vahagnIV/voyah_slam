#include <voyahslam/features/feature_grid.h>

#include <algorithm>
#include <cmath>

namespace voyahslam::features {

namespace {
constexpr double kMinCellSize = 1e-6;
}

FeatureGrid::FeatureGrid(const geometry::TPoints2DF& points, const TLevels& levels, int cols, int rows)
    : points_(points), levels_(levels), cols_(std::max(cols, 1)), rows_(std::max(rows, 1)) {
    const Eigen::Index count = points_.cols();
    cells_.resize(static_cast<size_t>(cols_) * rows_);
    if (count == 0) return;

    min_x_ = points_.row(0).minCoeff();
    min_y_ = points_.row(1).minCoeff();
    cell_width_ = std::max((points_.row(0).maxCoeff() - min_x_) / cols_, kMinCellSize);
    cell_height_ = std::max((points_.row(1).maxCoeff() - min_y_) / rows_, kMinCellSize);

    for (Eigen::Index i = 0; i < count; ++i) {
        const int col = std::clamp(static_cast<int>((points_(0, i) - min_x_) / cell_width_), 0, cols_ - 1);
        const int row = std::clamp(static_cast<int>((points_(1, i) - min_y_) / cell_height_), 0, rows_ - 1);
        cells_[static_cast<size_t>(row) * cols_ + col].push_back(i);
    }
}

std::vector<Eigen::Index> FeatureGrid::FeaturesInArea(const geometry::TPoint2DF& center, double radius,
                                                      int min_level, int max_level) const {
    std::vector<Eigen::Index> result;
    if (points_.cols() == 0) return result;

    // Compute in double and clamp before casting, so far-away queries cannot overflow int.
    const auto cell_range = [](double lo, double hi, double origin, double size, int count) {
        const double first = std::floor((lo - origin) / size);
        const double last = std::floor((hi - origin) / size);
        return std::pair<int, int>(static_cast<int>(std::clamp(first, 0.0, count - 1.0)),
                                   static_cast<int>(std::clamp(last, -1.0, count - 1.0)));
    };
    const auto [col_begin, col_end] =
        cell_range(center.x() - radius, center.x() + radius, min_x_, cell_width_, cols_);
    const auto [row_begin, row_end] =
        cell_range(center.y() - radius, center.y() + radius, min_y_, cell_height_, rows_);

    for (int row = row_begin; row <= row_end; ++row) {
        for (int col = col_begin; col <= col_end; ++col) {
            for (const Eigen::Index i : cells_[static_cast<size_t>(row) * cols_ + col]) {
                const int level = levels_(i);
                if (level < min_level || level > max_level) continue;
                if (std::abs(points_(0, i) - center.x()) <= radius && std::abs(points_(1, i) - center.y()) <= radius)
                    result.push_back(i);
            }
        }
    }
    return result;
}

}  // namespace voyahslam::features
