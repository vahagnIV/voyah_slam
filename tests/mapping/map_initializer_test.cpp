#include <gtest/gtest.h>
#include <support/test_measurements.h>
#include <voyahslam/mapping/map_initializer.h>

namespace voyahslam::mapping {
namespace {

using Extractor = test::TestExtractor;
using Measurement = features::TMeasurement<Extractor>;
using Binocular = features::TBinocularMeasurement<Extractor>;

double Timestamp(const Measurement& m) {
    return std::visit([](const auto& v) { return v.timestamp(); }, m);
}

TEST(MapInitializerTest, StereoInitializesImmediately) {
    MapInitializer<Extractor> initializer(1.0);
    auto result = initializer.TryInitialize(test::MakeBinocular(200, 0.0));
    ASSERT_TRUE(result.has_value());
    ASSERT_EQ(result->size(), 1u);
    EXPECT_TRUE((*result)[0].T_cw.ToMatrix34().isApprox(geometry::RigidTransformation::Identity().ToMatrix34()));
    EXPECT_TRUE(std::holds_alternative<Binocular>((*result)[0].measurement));
}

TEST(MapInitializerTest, MonoWaitsForInterval) {
    MapInitializer<Extractor> initializer(1.0);
    EXPECT_FALSE(initializer.TryInitialize(test::MakeMonocular(200, 0.0)).has_value());
    EXPECT_FALSE(initializer.TryInitialize(test::MakeMonocular(200, 0.5)).has_value());
    auto result = initializer.TryInitialize(test::MakeMonocular(200, 1.0));
    ASSERT_TRUE(result.has_value());
    ASSERT_EQ(result->size(), 2u);
    EXPECT_DOUBLE_EQ(Timestamp((*result)[0].measurement), 0.0);
    EXPECT_DOUBLE_EQ(Timestamp((*result)[1].measurement), 1.0);
    EXPECT_TRUE((*result)[0].T_cw.ToMatrix34().isApprox(geometry::RigidTransformation::Identity().ToMatrix34()));
    EXPECT_TRUE((*result)[1].T_cw.ToMatrix34().isApprox(geometry::RigidTransformation::Identity().ToMatrix34()));
}

TEST(MapInitializerTest, FewFeaturesReplacesReference) {
    MapInitializer<Extractor> initializer(1.0);
    EXPECT_FALSE(initializer.TryInitialize(test::MakeMonocular(200, 0.0)).has_value());
    EXPECT_FALSE(initializer.TryInitialize(test::MakeMonocular(50, 0.6)).has_value());
    EXPECT_FALSE(initializer.TryInitialize(test::MakeMonocular(200, 1.2)).has_value());
    auto result = initializer.TryInitialize(test::MakeMonocular(200, 2.2));
    ASSERT_TRUE(result.has_value());
    ASSERT_EQ(result->size(), 2u);
    EXPECT_DOUBLE_EQ(Timestamp((*result)[0].measurement), 1.2);
    EXPECT_DOUBLE_EQ(Timestamp((*result)[1].measurement), 2.2);
}

TEST(MapInitializerTest, ResetDropsReference) {
    MapInitializer<Extractor> initializer(1.0);
    EXPECT_FALSE(initializer.TryInitialize(test::MakeMonocular(200, 0.0)).has_value());
    initializer.Reset();
    EXPECT_FALSE(initializer.TryInitialize(test::MakeMonocular(200, 1.0)).has_value());
}

TEST(MapInitializerTest, StereoInitializesEvenWithMonoReference) {
    MapInitializer<Extractor> initializer(1.0);
    EXPECT_FALSE(initializer.TryInitialize(test::MakeMonocular(200, 0.0)).has_value());
    auto result = initializer.TryInitialize(test::MakeBinocular(200, 0.1));
    ASSERT_TRUE(result.has_value());
    EXPECT_EQ(result->size(), 1u);
    EXPECT_FALSE(initializer.TryInitialize(test::MakeMonocular(200, 0.2)).has_value());
}

} // namespace
} // namespace voyahslam::mapping
