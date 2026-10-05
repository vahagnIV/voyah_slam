#include <gtest/gtest.h>
#include <mapping/paired_inliers.h>

#include <vector>

namespace voyahslam::mapping {
namespace {

using Indices = std::vector<Eigen::Index>;

TEST(PairedInliers, KeepsPointsWithBothObservationsInliers) {
    EXPECT_EQ(PairedInliers({true, true, true, true}, 2), (Indices{0, 1}));
}

TEST(PairedInliers, DropsOutlierInFirstViewOnly) {
    EXPECT_EQ(PairedInliers({true, true, false, true, true, true}, 3), (Indices{0, 2}));
}

TEST(PairedInliers, DropsOutlierInSecondViewOnly) {
    EXPECT_EQ(PairedInliers({true, true, true, false, true, true}, 3), (Indices{0, 2}));
}

TEST(PairedInliers, DropsOutlierInBothViews) {
    EXPECT_EQ(PairedInliers({true, true, false, false, true, true}, 3), (Indices{0, 2}));
}

TEST(PairedInliers, AllOutliersGiveEmpty) {
    EXPECT_TRUE(PairedInliers({false, false, false, true}, 2).empty());
}

TEST(PairedInliers, EmptyInput) {
    EXPECT_TRUE(PairedInliers({}, 0).empty());
}

}  // namespace
}  // namespace voyahslam::mapping
