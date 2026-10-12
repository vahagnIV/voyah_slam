#include "voyahslam/voyahslam.h"

namespace voyahslam {

// From project(VERSION) in the top-level CMakeLists.txt (libs/slam/CMakeLists.txt).
int version_major() { return VOYAHSLAM_VERSION_MAJOR; }
int version_minor() { return VOYAHSLAM_VERSION_MINOR; }

} // namespace voyahslam
