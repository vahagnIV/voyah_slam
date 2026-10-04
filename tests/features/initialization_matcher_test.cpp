#include <gtest/gtest.h>
#include <cmath>
#include <iostream>
#include <numbers>
#include <random>
#include <vector>
#include "support/synthetic_sequence.h"
#include "support/test_measurements.h"
#include "voyahslam/features/initialization_matcher.h"

namespace voyahslam::features::test {

namespace {

using voyahslam::test::TestExtractor;
using voyahslam::test::TestImageFeatures;
using Measurement = TMonocularMeasurement<TestExtractor>;
using Descriptor = TestImageFeatures::TDescriptor;

struct Spec {
    double x = 256.0;
    double y = 256.0;
    int level = 0;
    double angle = 0.0;
    int bits = 0;        // descriptor = `bits` low bits set; distance between two = |difference|
    int seed = -1;       // >= 0: random descriptor instead
};

Descriptor MakeDescriptor(const Spec& spec) {
    Descriptor d = Descriptor::Zero();
    if (spec.seed >= 0) {
        std::mt19937 rng(static_cast<uint32_t>(spec.seed));
        for (Eigen::Index i = 0; i < d.size(); ++i) {
            d[i] = static_cast<uint8_t>(rng() & 0xFFU);
        }
        return d;
    }
    for (int b = 0; b < spec.bits; ++b) {
        d[b / 8] |= static_cast<uint8_t>(1U << (b % 8));
    }
    return d;
}

Measurement Make(const std::vector<Spec>& specs) {
    const auto n = static_cast<Eigen::Index>(specs.size());
    geometry::TPoints2DF points(2, n);
    geometry::TRowVectorF angles(n);
    TestImageFeatures::TDescriptors descriptors(TestExtractor::DescriptorSize, n);
    TLevels levels(n);
    for (Eigen::Index i = 0; i < n; ++i) {
        points.col(i) = geometry::TPoint2DF(specs[i].x, specs[i].y);
        angles[i] = static_cast<float>(specs[i].angle);
        descriptors.col(i) = MakeDescriptor(specs[i]);
        levels[i] = static_cast<uint8_t>(specs[i].level);
    }
    return Measurement(TestImageFeatures(std::move(points), geometry::TRowVectorF::Ones(n),
                                         std::move(angles), std::move(descriptors),
                                         std::move(levels)),
                       voyahslam::test::SyntheticIntrinsics(), nullptr, 0.0);
}

std::vector<geometry::TPoint2DF> PointsOf(const Measurement& m) {
    std::vector<geometry::TPoint2DF> result;
    for (Eigen::Index i = 0; i < m.points().cols(); ++i) {
        result.emplace_back(m.points().col(i));
    }
    return result;
}

std::vector<Eigen::Index> RunMatcher(const Measurement& ref, const Measurement& cur,
                              std::vector<geometry::TPoint2DF>& last,
                              const InitializationMatcherOptions& options = {}) {
    const FeatureGrid grid(cur.points(), cur.levels());
    return MatchForInitialization<TestExtractor>(ref, cur, grid, last, options);
}

std::vector<Eigen::Index> RunMatcher(const Measurement& ref, const Measurement& cur,
                              const InitializationMatcherOptions& options = {}) {
    auto last = PointsOf(ref);
    return RunMatcher(ref, cur, last, options);
}

}  // namespace

TEST(InitializationMatcher, WindowExcludesFarCandidates) {
    const Measurement ref = Make({{.x = 100.0, .y = 256.0}});
    EXPECT_EQ(RunMatcher(ref, Make({{.x = 250.0, .y = 256.0}})), std::vector<Eigen::Index>{-1});
    EXPECT_EQ(RunMatcher(ref, Make({{.x = 150.0, .y = 256.0}})), std::vector<Eigen::Index>{0});
}

TEST(InitializationMatcher, RatioTestRejectsAmbiguous) {
    const Measurement ref = Make({{}});
    EXPECT_EQ(RunMatcher(ref, Make({{.bits = 10}, {.x = 260.0, .bits = 11}})), std::vector<Eigen::Index>{-1});
    EXPECT_EQ(RunMatcher(ref, Make({{.bits = 10}, {.x = 260.0, .bits = 20}})), std::vector<Eigen::Index>{0});
}

TEST(InitializationMatcher, RejectsAboveMaxDistance) {
    const Measurement ref = Make({{}});
    EXPECT_EQ(RunMatcher(ref, Make({{.bits = 51}})), std::vector<Eigen::Index>{-1});
    EXPECT_EQ(RunMatcher(ref, Make({{.bits = 50}})), std::vector<Eigen::Index>{0});
}

TEST(InitializationMatcher, LaterBetterMatchReplacesClaim) {
    // Reference A (20 bits off) first claims c; B (5 bits off) later replaces it.
    const Measurement ref = Make({{.bits = 20}, {.x = 260.0, .bits = 5}});
    const Measurement cur = Make({{.bits = 0}});
    // Distances to c (all-zero descriptor): A = 20, B = 5.
    auto last = PointsOf(ref);
    const auto matches = RunMatcher(ref, cur, last);
    ASSERT_EQ(matches.size(), 2U);
    EXPECT_EQ(matches[0], -1);
    EXPECT_EQ(matches[1], 0);
    EXPECT_EQ(last[0], PointsOf(ref)[0]);
    EXPECT_EQ(last[1], PointsOf(cur)[0]);
}

TEST(InitializationMatcher, EarlierBetterMatchBlocksLaterWorseOne) {
    const Measurement ref = Make({{.bits = 5}, {.x = 260.0, .bits = 20}});
    const auto matches = RunMatcher(ref, Make({{.bits = 0}}));
    EXPECT_EQ(matches[0], 0);
    EXPECT_EQ(matches[1], -1);
}

TEST(InitializationMatcher, OrientationFilterDropsInconsistent) {
    std::vector<Spec> ref_specs, cur_specs;
    for (int i = 0; i < 41; ++i) {
        const double x = 20.0 + 11.0 * i;
        const double y = 20.0 + 11.0 * (i % 7);
        const bool outlier = (i == 40);
        ref_specs.push_back({.x = x, .y = y, .angle = outlier ? std::numbers::pi : 0.1, .seed = i});
        cur_specs.push_back({.x = x, .y = y, .angle = 0.0, .seed = i});
    }
    const Measurement ref = Make(ref_specs);
    const Measurement cur = Make(cur_specs);

    const auto filtered = RunMatcher(ref, cur);
    ASSERT_EQ(filtered.size(), 41U);
    for (int i = 0; i < 40; ++i) {
        EXPECT_EQ(filtered[i], i);
    }
    EXPECT_EQ(filtered[40], -1);

    InitializationMatcherOptions options;
    options.check_orientation = false;
    EXPECT_EQ(RunMatcher(ref, cur, options)[40], 40);
}

TEST(InitializationMatcher, OrientationWrapsAroundZero) {
    // 12 bins of 30 degrees. Rotation = ref - cur, wrapped to [0, 2 pi):
    //   A: 11 x +0.01 rad          -> bin 0
    //   B:  3 x -0.3 rad (wraps)   -> bin 11
    //   C:  3 x 5.98 rad           -> bin 11 (same rotation as B)
    //   E:  3 x pi                 -> bin 6
    //   D:  1 x pi/2               -> bin 3, 4th bin: dropped
    // Without the wrap, B would land in another bin and push one group out of the top three.
    std::vector<Spec> ref_specs, cur_specs;
    std::vector<int> expected_kept;
    const auto add = [&](int count, double rotation, bool kept) {
        for (int k = 0; k < count; ++k) {
            const int i = static_cast<int>(ref_specs.size());
            const double x = 20.0 + 11.0 * (i % 40);
            const double y = 20.0 + 60.0 * (i / 40);
            ref_specs.push_back({.x = x, .y = y, .angle = rotation, .seed = i});
            cur_specs.push_back({.x = x, .y = y, .angle = 0.0, .seed = i});
            expected_kept.push_back(kept ? i : -1);
        }
    };
    add(11, 0.01, true);
    add(3, -0.3, true);
    add(3, 5.98, true);
    add(3, std::numbers::pi, true);
    add(1, std::numbers::pi / 2.0, false);
    EXPECT_EQ(RunMatcher(Make(ref_specs), Make(cur_specs)),
              std::vector<Eigen::Index>(expected_kept.begin(), expected_kept.end()));
}

TEST(InitializationMatcher, MaxReferenceLevel) {
    const Measurement ref = Make({{.level = 1}});
    const Measurement cur1 = Make({{.level = 1}});
    InitializationMatcherOptions options;
    EXPECT_EQ(RunMatcher(ref, cur1, options), std::vector<Eigen::Index>{-1});
    options.max_reference_level = 1;
    EXPECT_EQ(RunMatcher(ref, cur1, options), std::vector<Eigen::Index>{0});
    EXPECT_EQ(RunMatcher(ref, Make({{.level = 0}}), options), std::vector<Eigen::Index>{-1});
}

TEST(InitializationMatcher, NoReferenceFeatureAtAllowedLevel) {
    const Measurement ref = Make({{.x = 100.0, .level = 1}, {.x = 200.0, .level = 1}});
    const Measurement cur = Make({{.x = 100.0, .level = 1}, {.x = 200.0, .level = 1}});
    auto last = PointsOf(ref);
    const auto before = last;
    const auto matches = RunMatcher(ref, cur, last);
    EXPECT_EQ(matches, (std::vector<Eigen::Index>{-1, -1}));
    EXPECT_EQ(last, before);
}

TEST(InitializationMatcher, SyntheticPrecision) {
    const auto sequence = voyahslam::test::MakeSyntheticSequence();
    const auto& ref = sequence.frames[0];
    const auto& cur = sequence.frames[3];
    auto last = PointsOf(ref.measurement);
    const auto matches = RunMatcher(ref.measurement, cur.measurement, last);
    ASSERT_EQ(matches.size(), ref.point_of_feature.size());

    size_t found = 0, correct = 0, possible = 0;
    for (size_t i = 0; i < matches.size(); ++i) {
        const Eigen::Index p = ref.point_of_feature[i];
        if (p >= 0) {
            for (const Eigen::Index q : cur.point_of_feature) {
                if (q == p) {
                    ++possible;
                    break;
                }
            }
        }
        if (matches[i] < 0) {
            continue;
        }
        ++found;
        if (p >= 0 && cur.point_of_feature[matches[i]] == p) {
            ++correct;
        }
        EXPECT_EQ(last[i], cur.measurement.points().col(matches[i]));
    }
    ASSERT_GT(found, 0U);
    const double precision = static_cast<double>(correct) / static_cast<double>(found);
    const double recall = static_cast<double>(correct) / static_cast<double>(possible);
    std::cout << "precision " << precision << " recall " << recall << " (" << found << " matches)\n";
    RecordProperty("recall", std::to_string(recall));
    EXPECT_GE(precision, 0.95);
}

}  // namespace voyahslam::features::test
