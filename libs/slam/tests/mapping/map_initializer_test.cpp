#include <gtest/gtest.h>
#include <support/synthetic_sequence.h>
#include <support/test_measurements.h>
#include <voyahslam/mapping/map_initializer.h>

#include <algorithm>
#include <cmath>
#include <numbers>
#include <optional>
#include <sstream>
#include <string>
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

// Translation direction error of the second keyframe against the truth.
double DirectionErrorDeg(const test::SyntheticSequence& sequence, const Success& success) {
    return AngleBetweenDeg(success.result.keyframes[1].T_cw.translation(),
                           sequence.frames[success.index].T_cw.translation());
}

double MedianDepth(const Result& result) {
    std::vector<double> depths;
    for (const InitialMapPoint& point : result.map_points) {
        depths.push_back(point.position.z());
    }
    std::nth_element(depths.begin(), depths.begin() + depths.size() / 2, depths.end());
    return depths[depths.size() / 2];
}

// The initializer's bundle adjustment problem for the correspondences of an
// initialization (its two keyframes and map points): raw-pixel measurements,
// each view's camera and level-scaled pixel sigmas, starting from poses and
// points supplied later.
optimization::BaProblem InitializationProblem(const Result& result, const MapInitializerOptions& options) {
    const auto& view1 = std::get<Monocular>(result.keyframes[0].measurement);
    const auto& view2 = std::get<Monocular>(result.keyframes[1].measurement);
    const auto sigma = [&options](const Monocular& view, Eigen::Index feature) {
        return options.sigma_px * std::pow(options.level_scale, static_cast<double>(view.levels()(feature)));
    };
    optimization::BaProblem problem;
    problem.keyframe_fixed = {true, false};
    problem.keyframe_cameras = {view1.camera(), view2.camera()};
    problem.points.resize(3, static_cast<Eigen::Index>(result.map_points.size()));
    for (size_t p = 0; p < result.map_points.size(); ++p) {
        const InitialMapPoint& point = result.map_points[p];
        problem.observations.push_back({0, p, view1.distorted_points().col(point.feature_in_first),
                                        sigma(view1, point.feature_in_first)});
        problem.observations.push_back({1, p, view2.distorted_points().col(point.feature_in_second),
                                        sigma(view2, point.feature_in_second)});
    }
    return problem;
}

// Plain chi2 of the initialization's own estimate on its observations.
double Chi2(const Result& result, const MapInitializerOptions& options) {
    const optimization::BaProblem problem = InitializationProblem(result, options);
    const geometry::RigidTransformation T[2] = {result.keyframes[0].T_cw, result.keyframes[1].T_cw};
    double chi2 = 0.0;
    for (const optimization::BaObservation& observation : problem.observations) {
        const geometry::TVector3DF p_c =
            T[observation.keyframe].Transform(result.map_points[observation.point].position);
        const geometry::TPoint2DF projected = problem.keyframe_cameras[observation.keyframe].Project(p_c);
        chi2 += ((projected - observation.measurement) / observation.sigma).squaredNorm();
    }
    return chi2;
}

TEST_F(MapInitializerTest, BundleAdjustmentReachesOptimum) {
    // At the initializing frame's low parallax the direction error is limited by
    // the noise, not by the solver: the BA must reach the ML optimum, found by
    // the same adjustment started from the ground truth (the oracle).
    const MapInitializerOptions ba_options;
    double sum_plain = 0.0;
    double sum_ba = 0.0;
    for (unsigned seed = 1; seed <= 5; ++seed) {
        SCOPED_TRACE("seed " + std::to_string(seed));
        test::SyntheticSequenceOptions sequence_options;
        sequence_options.seed = seed;
        const test::SyntheticSequence sequence = test::MakeSyntheticSequence(sequence_options);

        MapInitializerOptions plain_options;
        plain_options.bundle_adjust = false;
        MapInitializer<Extractor> plain(plain_options);
        MapInitializer<Extractor> adjusted(ba_options);
        const auto plain_success = FeedUntilSuccess(plain, sequence, 0, 20);
        const auto ba_success = FeedUntilSuccess(adjusted, sequence, 0, 20);
        ASSERT_TRUE(plain_success.has_value());
        ASSERT_TRUE(ba_success.has_value());
        ASSERT_EQ(plain_success->index, ba_success->index);
        const Result& plain_result = plain_success->result;
        const Result& ba_result = ba_success->result;

        // Oracle: the plain initialization's problem (all triangulated points),
        // started from the truth with the unscaled two-view baseline (|t| = 1).
        const geometry::RigidTransformation& T_true = sequence.frames[plain_success->index].T_cw;
        const double to_unit_baseline = 1.0 / T_true.translation().norm();
        optimization::BaProblem oracle_problem = InitializationProblem(plain_result, ba_options);
        oracle_problem.keyframe_poses = {
            geometry::RigidTransformation::Identity(),
            geometry::RigidTransformation(T_true.rotation(), to_unit_baseline * T_true.translation())};
        const std::vector<Eigen::Index>& point_of_feature = sequence.frames[0].point_of_feature;
        for (size_t p = 0; p < plain_result.map_points.size(); ++p) {
            const Eigen::Index point = point_of_feature[plain_result.map_points[p].feature_in_first];
            ASSERT_GE(point, 0);
            oracle_problem.points.col(static_cast<Eigen::Index>(p)) =
                to_unit_baseline * sequence.points_w.col(point);
        }
        const optimization::BaResult oracle =
            optimization::BundleAdjust(oracle_problem, ba_options.bundle_adjustment);
        ASSERT_TRUE(oracle.success);

        const double dir_err_plain = DirectionErrorDeg(sequence, *plain_success);
        const double dir_err_ba = DirectionErrorDeg(sequence, *ba_success);
        const double dir_err_oracle = AngleBetweenDeg(oracle.keyframe_poses[1].translation(), T_true.translation());
        // Both costs over the observations of the BA's survivors.
        Result plain_survivors = plain_result;
        std::erase_if(plain_survivors.map_points, [&ba_result](const InitialMapPoint& point) {
            return std::none_of(ba_result.map_points.begin(), ba_result.map_points.end(),
                                [&point](const InitialMapPoint& survivor) {
                                    return survivor.feature_in_first == point.feature_in_first;
                                });
        });
        ASSERT_EQ(plain_survivors.map_points.size(), ba_result.map_points.size());
        const double cost_plain = Chi2(plain_survivors, ba_options);
        const double cost_ba = Chi2(ba_result, ba_options);
        std::ostringstream summary;
        summary << "frame " << ba_success->index << ": dir_err plain " << dir_err_plain << " deg, BA " << dir_err_ba
                << " deg, oracle " << dir_err_oracle << " deg; chi2 plain " << cost_plain << ", BA " << cost_ba
                << "; points " << plain_result.map_points.size() << " -> " << ba_result.map_points.size();
        RecordProperty("seed_" + std::to_string(seed), summary.str());

        EXPECT_NEAR(dir_err_ba, dir_err_oracle, 0.1);
        EXPECT_LT(cost_ba, cost_plain);
        EXPECT_NEAR(MedianDepth(ba_result), 1.0, 1e-9);
        EXPECT_GE(ba_result.map_points.size(), 50u);
        sum_plain += dir_err_plain;
        sum_ba += dir_err_ba;
    }
    EXPECT_LE(sum_ba / 5.0, sum_plain / 5.0);
}

TEST_F(MapInitializerTest, TooFewPointsAfterBaFails) {
    MapInitializerOptions options;
    options.min_points_after_ba = 100000;
    MapInitializer<Extractor> initializer(options);
    EXPECT_FALSE(FeedUntilSuccess(initializer, sequence_, 0, 10).has_value());
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
