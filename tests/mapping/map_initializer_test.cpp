#include <gtest/gtest.h>
#include <support/synthetic_sequence.h>
#include <support/test_measurements.h>
#include <voyahslam/mapping/map_initializer.h>

#include <algorithm>
#include <cmath>
#include <numbers>
#include <optional>
#include <variant>
#include <vector>

namespace voyahslam::mapping {
namespace {

using Extractor = test::TestExtractor;
using Measurement = features::TMeasurement<Extractor>;
using Monocular = features::TMonocularMeasurement<Extractor>;
using Binocular = features::TBinocularMeasurement<Extractor>;
using Result = InitializationResult<Extractor>;

double Timestamp(const Measurement& m) {
    return std::visit([](const auto& v) { return v.timestamp(); }, m);
}

bool IsIdentity(const geometry::RigidTransformation& T) {
    return T.ToMatrix34().isApprox(geometry::RigidTransformation::Identity().ToMatrix34());
}

double RadToDeg(double rad) { return rad * 180.0 / std::numbers::pi; }

double RotationErrorDeg(const geometry::TMatrix33F& R, const geometry::TMatrix33F& R_true) {
    return RadToDeg(Eigen::AngleAxis<double>(R.transpose() * R_true).angle());
}

double AngleBetweenDeg(const geometry::TVector3DF& a, const geometry::TVector3DF& b) {
    const double cos = std::clamp(a.normalized().dot(b.normalized()), -1.0, 1.0);
    return RadToDeg(std::acos(cos));
}

// Feeds frames [first, last] in order; returns the index of the first frame
// that completes the initialization and the result.
struct Success {
    size_t index;
    Result result;
};
std::optional<Success> FeedUntilSuccess(MapInitializer<Extractor>& initializer,
                                        const test::SyntheticSequence& sequence, size_t first,
                                        size_t last) {
    for (size_t k = first; k <= last && k < sequence.frames.size(); ++k) {
        auto result = initializer.TryInitialize(sequence.frames[k].measurement);
        if (result) {
            return Success{k, std::move(*result)};
        }
    }
    return std::nullopt;
}

class MapInitializerTest : public ::testing::Test {
protected:
    test::SyntheticSequence sequence_ = test::MakeSyntheticSequence();
    MapInitializer<Extractor> initializer_;
};

TEST_F(MapInitializerTest, MonoInitializesOnceParallaxSuffices) {
    EXPECT_FALSE(initializer_.TryInitialize(sequence_.frames[0].measurement).has_value());
    EXPECT_FALSE(initializer_.TryInitialize(sequence_.frames[1].measurement).has_value());
    auto success = FeedUntilSuccess(initializer_, sequence_, 2, 20);
    ASSERT_TRUE(success.has_value());
    const size_t k = success->index;
    const Result& result = success->result;
    RecordProperty("first_success_index", static_cast<int>(k));

    ASSERT_EQ(result.keyframes.size(), 2u);
    EXPECT_TRUE(IsIdentity(result.keyframes[0].T_cw));
    EXPECT_DOUBLE_EQ(Timestamp(result.keyframes[0].measurement), sequence_.frames[0].measurement.timestamp());
    EXPECT_DOUBLE_EQ(Timestamp(result.keyframes[1].measurement), sequence_.frames[k].measurement.timestamp());

    // World = frame 0's camera, so frame k's T_cw is T_c_k,c_0.
    const geometry::RigidTransformation& T_true = sequence_.frames[k].T_cw;
    const geometry::RigidTransformation& T = result.keyframes[1].T_cw;
    const double rotation_error = RotationErrorDeg(T.rotation(), T_true.rotation());
    const double direction_error = AngleBetweenDeg(T.translation(), T_true.translation());
    RecordProperty("rotation_error_deg", std::to_string(rotation_error));
    RecordProperty("translation_direction_error_deg", std::to_string(direction_error));
    EXPECT_LT(rotation_error, 1.0);
    // Two-view inlier-set lock-in leaves a few degrees of direction error.
    EXPECT_LT(direction_error, 10.0);
}

TEST_F(MapInitializerTest, MedianDepthIsOne) {
    ASSERT_FALSE(initializer_.TryInitialize(sequence_.frames[0].measurement).has_value());
    auto success = FeedUntilSuccess(initializer_, sequence_, 1, 20);
    ASSERT_TRUE(success.has_value());
    const Result& result = success->result;
    const test::SyntheticFrame& first = sequence_.frames[0];
    const test::SyntheticFrame& second = sequence_.frames[success->index];

    ASSERT_GE(result.map_points.size(), 50u);
    std::vector<double> depths;
    for (const InitialMapPoint& point : result.map_points) {
        depths.push_back(point.position.z());
        ASSERT_GE(point.feature_in_first, 0);
        ASSERT_LT(point.feature_in_first, first.measurement.points().cols());
        ASSERT_GE(point.feature_in_second, 0);
        ASSERT_LT(point.feature_in_second, second.measurement.points().cols());
        ASSERT_GE(first.point_of_feature[point.feature_in_first], 0);
        EXPECT_EQ(first.point_of_feature[point.feature_in_first],
                  second.point_of_feature[point.feature_in_second]);
    }
    std::nth_element(depths.begin(), depths.begin() + depths.size() / 2, depths.end());
    EXPECT_NEAR(depths[depths.size() / 2], 1.0, 1e-9);

    // The scaled points and the scaled t_21 agree with the measurements: a
    // missing or doubled 1/median_depth scale on t breaks the second view.
    const geometry::TMatrix33F K = test::SyntheticIntrinsics();
    const geometry::RigidTransformation& T_second = result.keyframes[1].T_cw;
    double max_error_first = 0.0;
    double max_error_second = 0.0;
    for (const InitialMapPoint& point : result.map_points) {
        const geometry::TVector3DF p1 = result.keyframes[0].T_cw.Transform(point.position);
        const geometry::TVector3DF p2 = T_second.Transform(point.position);
        const Eigen::Vector2d uv1 = (K * p1).hnormalized();
        const Eigen::Vector2d uv2 = (K * p2).hnormalized();
        max_error_first = std::max(
            max_error_first, (uv1 - first.measurement.points().col(point.feature_in_first)).norm());
        max_error_second = std::max(
            max_error_second,
            (uv2 - second.measurement.points().col(point.feature_in_second)).norm());
    }
    RecordProperty("max_reprojection_error_first_px", std::to_string(max_error_first));
    RecordProperty("max_reprojection_error_second_px", std::to_string(max_error_second));
    EXPECT_LT(max_error_first, 5.0);
    EXPECT_LT(max_error_second, 5.0);
}

TEST_F(MapInitializerTest, FewFeaturesResetsReference) {
    EXPECT_FALSE(initializer_.TryInitialize(sequence_.frames[0].measurement).has_value());
    EXPECT_FALSE(initializer_.TryInitialize(test::MakeMonocular(50, 0.01)).has_value());
    EXPECT_FALSE(initializer_.TryInitialize(sequence_.frames[3].measurement).has_value());

    auto success = FeedUntilSuccess(initializer_, sequence_, 4, 25);
    ASSERT_TRUE(success.has_value());
    ASSERT_EQ(success->result.keyframes.size(), 2u);
    EXPECT_DOUBLE_EQ(Timestamp(success->result.keyframes[0].measurement),
                     sequence_.frames[3].measurement.timestamp());
}

TEST_F(MapInitializerTest, FewMatchesReplacesReference) {
    test::SyntheticSequenceOptions unrelated_options;
    unrelated_options.seed = 2;
    const test::SyntheticSequence unrelated = test::MakeSyntheticSequence(unrelated_options);

    EXPECT_FALSE(initializer_.TryInitialize(sequence_.frames[0].measurement).has_value());
    EXPECT_FALSE(initializer_.TryInitialize(unrelated.frames[3].measurement).has_value());
    // The reference is now the unrelated frame: nothing matches.
    EXPECT_FALSE(initializer_.TryInitialize(sequence_.frames[3].measurement).has_value());
}

TEST_F(MapInitializerTest, FewMatchesReplacesReferenceEvenWithEnoughParallax) {
    // The frame that initializes against frame 0 ...
    MapInitializer<Extractor> probe;
    ASSERT_FALSE(probe.TryInitialize(sequence_.frames[0].measurement).has_value());
    auto success = FeedUntilSuccess(probe, sequence_, 1, 20);
    ASSERT_TRUE(success.has_value());

    test::SyntheticSequenceOptions unrelated_options;
    unrelated_options.seed = 2;
    const test::SyntheticSequence unrelated = test::MakeSyntheticSequence(unrelated_options);

    // ... does not once an unrelated frame has replaced frame 0.
    EXPECT_FALSE(initializer_.TryInitialize(sequence_.frames[0].measurement).has_value());
    EXPECT_FALSE(initializer_.TryInitialize(unrelated.frames[3].measurement).has_value());
    EXPECT_FALSE(initializer_.TryInitialize(sequence_.frames[success->index].measurement).has_value());
}

TEST_F(MapInitializerTest, PureRotationNeverInitializes) {
    test::SyntheticSequenceOptions options;
    options.step = 0.0;
    options.yaw_step_deg = 0.5;
    options.num_frames = 30;
    const test::SyntheticSequence rotation = test::MakeSyntheticSequence(options);
    EXPECT_FALSE(FeedUntilSuccess(initializer_, rotation, 0, 29).has_value());
}

TEST_F(MapInitializerTest, ResetDropsReference) {
    MapInitializer<Extractor> probe;
    ASSERT_FALSE(probe.TryInitialize(sequence_.frames[0].measurement).has_value());
    auto success = FeedUntilSuccess(probe, sequence_, 1, 20);
    ASSERT_TRUE(success.has_value());

    EXPECT_FALSE(initializer_.TryInitialize(sequence_.frames[0].measurement).has_value());
    initializer_.Reset();
    EXPECT_FALSE(initializer_.TryInitialize(sequence_.frames[success->index].measurement).has_value());
}

TEST_F(MapInitializerTest, StereoInitializesFromOneFrame) {
    // Noise-free, so every point is within a few % of the truth (with 0.5 px
    // noise the far points' depth error reaches ~10 %).
    test::SyntheticStereoOptions options;
    options.pixel_noise = 0.0;
    const test::SyntheticStereoFrame frame = test::MakeMatchedSyntheticStereoFrame(options);
    const Binocular& measurement = frame.measurement;
    auto result = initializer_.TryInitialize(measurement);
    ASSERT_TRUE(result.has_value());
    ASSERT_EQ(result->keyframes.size(), 1u);
    EXPECT_TRUE(IsIdentity(result->keyframes[0].T_cw));
    EXPECT_TRUE(std::holds_alternative<Binocular>(result->keyframes[0].measurement));

    const auto num_valid = static_cast<size_t>((measurement.depths().array() > 0.0).count());
    ASSERT_GE(num_valid, 100u);
    EXPECT_EQ(result->map_points.size(), num_valid);
    for (const InitialMapPoint& point : result->map_points) {
        const Eigen::Index i = point.feature_in_first;
        ASSERT_GE(i, 0);
        ASSERT_LT(i, measurement.left().points().cols());
        EXPECT_EQ(point.feature_in_second, -1);
        ASSERT_GT(measurement.depth(static_cast<int>(i)), 0.0);
        const geometry::TPoint3DF expected =
            measurement.depth(static_cast<int>(i)) * measurement.left().normalized_point(static_cast<int>(i));
        EXPECT_LT((point.position - expected).norm(), 1e-12) << "feature " << i;
        const geometry::TPoint3DF truth = frame.points_left.col(i);
        ASSERT_GT(frame.depth_of_left[i], 0.0) << "feature " << i << " is an extra";
        EXPECT_LT((point.position - truth).norm(), 0.05 * truth.norm()) << "feature " << i;
    }
}

TEST_F(MapInitializerTest, StereoTooFewFeatures) {
    // Exactly stereo_min_features (500) left features, nearly all with a depth.
    test::SyntheticStereoOptions options;
    options.num_points = 500;
    options.num_extra = 0;
    const test::SyntheticStereoFrame frame = test::MakeMatchedSyntheticStereoFrame(options);
    ASSERT_EQ(frame.measurement.left().points().cols(), 500);
    ASSERT_GE((frame.measurement.depths().array() > 0.0).count(), 100);
    EXPECT_FALSE(initializer_.TryInitialize(frame.measurement).has_value());
}

TEST_F(MapInitializerTest, StereoTooFewPoints) {
    // Enough left features, but at most 50 can have a depth.
    test::SyntheticStereoOptions options;
    options.num_points = 50;
    options.num_extra = 600;
    const test::SyntheticStereoFrame frame = test::MakeMatchedSyntheticStereoFrame(options);
    ASSERT_GT(frame.measurement.left().points().cols(), 500);
    ASSERT_LT((frame.measurement.depths().array() > 0.0).count(), 100);
    EXPECT_FALSE(initializer_.TryInitialize(frame.measurement).has_value());
}

TEST_F(MapInitializerTest, BinocularDropsMonoReference) {
    EXPECT_FALSE(initializer_.TryInitialize(sequence_.frames[0].measurement).has_value());
    test::SyntheticStereoOptions options;
    options.timestamp = 0.1;
    auto result = initializer_.TryInitialize(test::MakeMatchedSyntheticStereoFrame(options).measurement);
    ASSERT_TRUE(result.has_value());
    ASSERT_EQ(result->keyframes.size(), 1u);
    EXPECT_TRUE(IsIdentity(result->keyframes[0].T_cw));
    EXPECT_GE(result->map_points.size(), 100u);
    EXPECT_FALSE(initializer_.TryInitialize(sequence_.frames[3].measurement).has_value());
}

} // namespace
} // namespace voyahslam::mapping
