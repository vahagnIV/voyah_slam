#include <voyahslam/map/point_statistics.h>

namespace voyahslam::map {

geometry::TPoint3DF MeanViewingDirection(const geometry::TPoint3DF& point,
                                         const std::vector<geometry::TPoint3DF>& camera_centres) {
    geometry::TPoint3DF sum = geometry::TPoint3DF::Zero();
    for (const auto& centre : camera_centres) {
        const geometry::TPoint3DF ray = point - centre;
        const auto norm = ray.norm();
        if (norm > 0) {
            sum += ray / norm;
        }
    }
    const auto norm = sum.norm();
    return norm > 0 ? geometry::TPoint3DF(sum / norm) : geometry::TPoint3DF(geometry::TPoint3DF::Zero());
}

}  // namespace voyahslam::map
