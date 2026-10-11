#ifndef VOYAHSLAM_SRC_FEATURES_ORB_EXTRACT_NODE_H
#define VOYAHSLAM_SRC_FEATURES_ORB_EXTRACT_NODE_H

#include "voyahslam/geometry/geometry.h"

#include <cstddef>
#include <list>
#include <vector>

namespace voyahslam::features::orb {

struct ExtractorNode {
    ExtractorNode() = default;
    ExtractorNode(size_t count);
    std::vector<size_t> keypoints;
    geometry::TPoint2DI ul, br;
    bool no_more = false;
    std::list<ExtractorNode>::iterator self_it{};

    void Divide(ExtractorNode& c1,
                ExtractorNode& c2,
                ExtractorNode& c3,
                ExtractorNode& c4,
                const geometry::TPoints2DF &positions) const;
};

} // namespace voyahslam::features::orb

#endif // VOYAHSLAM_SRC_FEATURES_ORB_EXTRACT_NODE_H
