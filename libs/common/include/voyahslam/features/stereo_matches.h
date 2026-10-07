#ifndef VOYAHSLAM_FEATURES_STEREO_MATCHES_H
#define VOYAHSLAM_FEATURES_STEREO_MATCHES_H

#include <Eigen/Dense>
#include <cstddef>
#include <voyahslam/defs.h>

namespace voyahslam::features {

// Result of matching a left image's features against the right image's.
struct StereoMatches {
    Eigen::VectorXi right_indices;  // per left feature; -1 = none
    geometry::TRowVectorF depths;   // z in the left camera; -1 = none
    size_t num_matches = 0;
};

} // namespace voyahslam::features

#endif // VOYAHSLAM_FEATURES_STEREO_MATCHES_H
