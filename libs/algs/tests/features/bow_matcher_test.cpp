#include <gtest/gtest.h>

#include <cstdint>
#include <optional>
#include <vector>

#include "support/test_measurements.h"
#include <voyahslam/features/bow_matcher.h>

namespace voyahslam::features {
namespace {

using voyahslam::test::TestExtractor;
using voyahslam::test::TestImageFeatures;
using Measurement = TMonocularMeasurement<TestExtractor>;
using Descriptor = TestImageFeatures::TDescriptor;
using Points = std::vector<std::optional<map::MapPointId>>;

// A feature with `bits` low descriptor bits set: the distance between two is
// the difference of their bits.
struct Spec {
    int bits = 0;
    double angle = 0.0;
};

Descriptor BitsDescriptor(int bits) {
    Descriptor d = Descriptor::Zero();
    for (int b = 0; b < bits; ++b) d[b / 8] |= static_cast<uint8_t>(1U << (b % 8));
    return d;
}

Measurement MakeMeasurement(const std::vector<Spec>& specs) {
    const auto n = static_cast<Eigen::Index>(specs.size());
    geometry::TPoints2DF points(2, n);
    geometry::TRowVectorF angles(n);
    TestImageFeatures::TDescriptors descriptors(TestExtractor::DescriptorSize, n);
    for (Eigen::Index i = 0; i < n; ++i) {
        points.col(i) = geometry::TPoint2DF(300.0 + 5.0 * static_cast<double>(i), 240.0);  // spread, inside the cut
        angles(i) = static_cast<float>(specs[static_cast<size_t>(i)].angle);
        descriptors.col(i) = BitsDescriptor(specs[static_cast<size_t>(i)].bits);
    }
    Measurement measurement(TestImageFeatures(std::move(points), geometry::TRowVectorF::Ones(n), std::move(angles),
                                              std::move(descriptors), TLevels::Zero(n)),
                            voyahslam::test::TestIntrinsics(), nullptr, 0.0);
    EXPECT_EQ(measurement.points().cols(), n);
    return measurement;
}

// groups[k] = (node, features under it); nodes ascending.
bow::FeatureVector MakeFeatureVector(const std::vector<std::pair<bow::NodeId, std::vector<uint32_t>>>& groups) {
    bow::FeatureVector fv;
    fv.offsets.push_back(0);
    for (const auto& [node, members] : groups) {
        fv.nodes.push_back(node);
        fv.features.insert(fv.features.end(), members.begin(), members.end());
        fv.offsets.push_back(static_cast<uint32_t>(fv.features.size()));
    }
    return fv;
}

Points AllPoints(size_t n) {
    Points points(n);
    for (size_t i = 0; i < n; ++i) points[i] = map::MapPointId{static_cast<uint32_t>(100 + i)};
    return points;
}

TEST(BowMatcherTest, MatchesOnlySharedNodes) {
    const Measurement frame = MakeMeasurement({{10}, {40}});
    const Measurement current = MakeMeasurement({{10}, {40}});
    // Identical descriptors, but frame feature 1 is under node 8 and current feature 1 under node 9.
    const auto result = SearchByBoW<TestExtractor>(frame, MakeFeatureVector({{7, {0}}, {8, {1}}}), AllPoints(2),
                                                   current, MakeFeatureVector({{7, {0}}, {9, {1}}}), {});
    ASSERT_EQ(result.size(), 2U);
    EXPECT_EQ(result[0], map::MapPointId{100});
    EXPECT_FALSE(result[1].has_value());
}

TEST(BowMatcherTest, RatioTestRejectsAmbiguous) {
    const Measurement frame = MakeMeasurement({{10}});
    // Distances 0 and 2 from the Frame feature: 0 < 0.75 * 2 accepted; 1 vs 2: 1 < 1.5 accepted; 2 vs 2: rejected.
    const auto fv = MakeFeatureVector({{1, {0, 1}}});
    const auto accepted = SearchByBoW<TestExtractor>(frame, MakeFeatureVector({{1, {0}}}), AllPoints(1),
                                                     MakeMeasurement({{10}, {12}}), fv, {});
    EXPECT_EQ(accepted[0], map::MapPointId{100});
    EXPECT_FALSE(accepted[1].has_value());
    const auto ambiguous = SearchByBoW<TestExtractor>(frame, MakeFeatureVector({{1, {0}}}), AllPoints(1),
                                                      MakeMeasurement({{8}, {12}}), fv, {});
    EXPECT_FALSE(ambiguous[0].has_value());
    EXPECT_FALSE(ambiguous[1].has_value());
}

TEST(BowMatcherTest, ThresholdRejectsFarDescriptors) {
    const auto result = SearchByBoW<TestExtractor>(MakeMeasurement({{0}}), MakeFeatureVector({{1, {0}}}), AllPoints(1),
                                                   MakeMeasurement({{60}}), MakeFeatureVector({{1, {0}}}), {});
    EXPECT_FALSE(result[0].has_value());
}

TEST(BowMatcherTest, OneToOne) {
    // Both Frame features prefer current feature 0; the first keeps it, the second takes the other.
    const Measurement frame = MakeMeasurement({{10}, {11}});
    const Measurement current = MakeMeasurement({{10}, {30}});
    const auto fv = MakeFeatureVector({{1, {0, 1}}});
    const auto result = SearchByBoW<TestExtractor>(frame, fv, AllPoints(2), current, fv, {});
    EXPECT_EQ(result[0], map::MapPointId{100});
    // Feature 1 (11) vs the remaining current feature (30): distance 19 <= 50, second best infinite.
    EXPECT_EQ(result[1], map::MapPointId{101});
    // No current feature is matched twice.
    EXPECT_NE(result[0], result[1]);
}

TEST(BowMatcherTest, SkipsFrameFeaturesWithoutPoints) {
    const Measurement frame = MakeMeasurement({{10}, {40}});
    const Measurement current = MakeMeasurement({{10}, {40}});
    const auto fv = MakeFeatureVector({{1, {0, 1}}});
    Points points = AllPoints(2);
    points[0] = std::nullopt;
    const auto result = SearchByBoW<TestExtractor>(frame, fv, points, current, fv, {});
    EXPECT_FALSE(result[0].has_value());
    EXPECT_EQ(result[1], map::MapPointId{101});
}

TEST(BowMatcherTest, RotationHistogramDropsInconsistent) {
    // Twenty matches rotated by 0, one by 180° (a bin under 0.1 x the largest is dropped).
    std::vector<Spec> frame_specs;
    std::vector<Spec> current_specs;
    std::vector<uint32_t> all;
    for (int i = 0; i < 21; ++i) {
        frame_specs.push_back({i * 10, 0.0});
        current_specs.push_back({i * 10, i == 20 ? 3.14159 : 0.0});
        all.push_back(static_cast<uint32_t>(i));
    }
    const auto fv = MakeFeatureVector({{1, all}});
    BowMatchOptions options;
    options.th_low = 5;  // descriptors are 10 apart
    const Measurement frame = MakeMeasurement(frame_specs);
    const Measurement current = MakeMeasurement(current_specs);
    const auto checked = SearchByBoW<TestExtractor>(frame, fv, AllPoints(21), current, fv, options);
    for (size_t i = 0; i < 20; ++i) EXPECT_EQ(checked[i], map::MapPointId{static_cast<uint32_t>(100 + i)});
    EXPECT_FALSE(checked[20].has_value());
    options.check_orientation = false;
    const auto unchecked = SearchByBoW<TestExtractor>(frame, fv, AllPoints(21), current, fv, options);
    EXPECT_TRUE(unchecked[20].has_value());
}

}  // namespace
}  // namespace voyahslam::features
