#include <gtest/gtest.h>
#include <support/test_measurements.h>
#include <voyahslam/map/point_statistics.h>

namespace voyahslam::map {
namespace {

using Extractor = test::TestExtractor;
using Descriptor = Extractor::TDescriptor;

Descriptor FlippedBits(int first, int count) {
    Descriptor d = Descriptor::Zero();
    for (int b = first; b < first + count; ++b) {
        d[b / 8] ^= static_cast<uint8_t>(1U << (b % 8));
    }
    return d;
}

TEST(PointStatisticsTest, MeanViewingDirectionIsUnitMean) {
    const geometry::TPoint3DF point(0, 0, 5);
    const std::vector<geometry::TPoint3DF> centres{{1, 0, 0}, {-1, 0, 0}};
    const auto direction = MeanViewingDirection(point, centres);
    EXPECT_NEAR((direction - geometry::TPoint3DF(0, 0, 1)).norm(), 0.0, 1e-6);
}

TEST(PointStatisticsTest, MeanViewingDirectionOfNoCentresIsZero) {
    EXPECT_EQ(MeanViewingDirection({0, 0, 5}, {}), geometry::TPoint3DF::Zero());
}

TEST(PointStatisticsTest, DistinctiveDescriptorPicksMedoid) {
    // Distances: (0,1) = 1, (0,2) = 9, (1,2) = 10. Median distances: 5, 5.5, 9.5.
    const std::vector<Descriptor> descriptors{FlippedBits(100, 0), FlippedBits(0, 1),
                                              FlippedBits(1, 9)};
    EXPECT_EQ(Extractor::Distance(descriptors[1], descriptors[2]), 10);
    for (const auto order : {std::vector<int>{0, 1, 2}, {2, 1, 0}, {1, 2, 0}}) {
        std::vector<Descriptor> shuffled;
        for (const int i : order) shuffled.push_back(descriptors[i]);
        EXPECT_EQ(DistinctiveDescriptor<Extractor>(shuffled), descriptors[0]);
    }
}

TEST(PointStatisticsTest, DistinctiveDescriptorTiesGoToLowestIndex) {
    const Descriptor d = FlippedBits(3, 4);
    const std::vector<Descriptor> descriptors{d, d, d};
    EXPECT_EQ(DistinctiveDescriptor<Extractor>(descriptors), d);
    const std::vector<Descriptor> two{FlippedBits(0, 2), FlippedBits(10, 2)};
    EXPECT_EQ(DistinctiveDescriptor<Extractor>(two), two[0]);
}

}  // namespace
}  // namespace voyahslam::map
