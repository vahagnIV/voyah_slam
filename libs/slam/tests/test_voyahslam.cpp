#include <gtest/gtest.h>
#include "voyahslam/voyahslam.h"

// The library reports project(VERSION) from the top-level CMakeLists.txt.
TEST(VoyahSlam, VersionMajor) {
    EXPECT_EQ(voyahslam::version_major(), VOYAHSLAM_TEST_VERSION_MAJOR);
}

TEST(VoyahSlam, VersionMinor) {
    EXPECT_EQ(voyahslam::version_minor(), VOYAHSLAM_TEST_VERSION_MINOR);
}
