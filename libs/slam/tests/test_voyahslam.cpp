#include <gtest/gtest.h>
#include "voyahslam/voyahslam.h"

TEST(VoyahSlam, VersionMajor) {
    EXPECT_EQ(voyahslam::version_major(), 0);
}

TEST(VoyahSlam, VersionMinor) {
    EXPECT_EQ(voyahslam::version_minor(), 1);
}
