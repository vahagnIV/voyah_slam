#include <gtest/gtest.h>
#include <algorithm>
#include <random>
#include <vector>
#include "voyahslam/features/feature_grid.h"

namespace voyahslam::features::test {

namespace {

struct GridData {
    geometry::TPoints2DF points;
    TLevels levels;
};

GridData RandomData(int count, double lo, double hi) {
    std::mt19937 rng(1234);
    std::uniform_real_distribution<double> coord(lo, hi);
    std::uniform_int_distribution<int> level(0, 7);
    GridData data;
    data.points.resize(2, count);
    data.levels.resize(count);
    for (int i = 0; i < count; ++i) {
        data.points(0, i) = coord(rng);
        data.points(1, i) = coord(rng);
        data.levels(i) = static_cast<uint8_t>(level(rng));
    }
    return data;
}

std::vector<Eigen::Index> BruteForce(const GridData& data, const geometry::TPoint2DF& center, double radius,
                                     int min_level, int max_level) {
    std::vector<Eigen::Index> result;
    for (Eigen::Index i = 0; i < data.points.cols(); ++i) {
        const int level = data.levels(i);
        if (level < min_level || level > max_level) continue;
        if (std::abs(data.points(0, i) - center.x()) <= radius && std::abs(data.points(1, i) - center.y()) <= radius)
            result.push_back(i);
    }
    return result;
}

std::vector<Eigen::Index> Sorted(std::vector<Eigen::Index> v) {
    std::sort(v.begin(), v.end());
    return v;
}

}  // namespace

TEST(FeatureGridTest, MatchesBruteForce) {
    const GridData data = RandomData(2000, -200, 700);
    const FeatureGrid grid(data.points, data.levels);
    std::mt19937 rng(99);
    std::uniform_real_distribution<double> coord(-200, 700);
    for (int q = 0; q < 50; ++q) {
        const geometry::TPoint2DF center(coord(rng), coord(rng));
        for (const double radius : {1.0, 10.0, 100.0}) {
            EXPECT_EQ(Sorted(grid.FeaturesInArea(center, radius, 0, 7)), BruteForce(data, center, radius, 0, 7));
        }
    }
}

TEST(FeatureGridTest, LevelFilter) {
    const GridData data = RandomData(2000, -200, 700);
    const FeatureGrid grid(data.points, data.levels);
    std::mt19937 rng(7);
    std::uniform_real_distribution<double> coord(-200, 700);
    for (int q = 0; q < 50; ++q) {
        const geometry::TPoint2DF center(coord(rng), coord(rng));
        for (const double radius : {1.0, 10.0, 100.0}) {
            const auto found = Sorted(grid.FeaturesInArea(center, radius, 2, 3));
            for (const Eigen::Index i : found) {
                EXPECT_GE(data.levels(i), 2);
                EXPECT_LE(data.levels(i), 3);
            }
            EXPECT_EQ(found, BruteForce(data, center, radius, 2, 3));
        }
    }
}

TEST(FeatureGridTest, ZeroExtentBoundingBox) {
    GridData data;
    data.points.resize(2, 10);
    data.points.setConstant(5.0);
    data.levels = TLevels::Zero(10);
    const FeatureGrid grid(data.points, data.levels);
    EXPECT_EQ(grid.FeaturesInArea({5.0, 5.0}, 0.0, 0, 7).size(), 10u);
    EXPECT_TRUE(grid.FeaturesInArea({7.0, 7.0}, 1.0, 0, 7).empty());
}

TEST(FeatureGridTest, EmptyInput) {
    const geometry::TPoints2DF points(2, 0);
    const TLevels levels(0);
    const FeatureGrid grid(points, levels);
    EXPECT_TRUE(grid.FeaturesInArea({0.0, 0.0}, 100.0, 0, 7).empty());
    EXPECT_TRUE(grid.FeaturesInArea({1e9, -1e9}, 1.0, 0, 7).empty());
}

}  // namespace voyahslam::features::test
