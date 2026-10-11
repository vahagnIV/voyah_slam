#include <gtest/gtest.h>

#include <cmath>
#include <cstddef>
#include <limits>
#include <optional>
#include <utility>
#include <vector>

#include <Eigen/Dense>

#include "voyahslam/geometry/rigid_transformation.h"
#include "voyahslam/geometry/sim3.h"
#include "voyahslam/optimization/essential_graph.h"

namespace voyahslam::optimization {
namespace {

using geometry::RigidTransformation;
using geometry::Sim3;
using geometry::TMatrix33F;
using geometry::TPoint3DF;
using geometry::TVector3DF;

constexpr size_t kRingFrames = 20;
constexpr size_t kQuery = kRingFrames - 1;  // closes the loop onto Frame 0
constexpr size_t kWindow = 3;               // the query and the two Frames before it

Sim3 ToSim3(const RigidTransformation& T) { return Sim3{1.0, T.rotation(), T.translation()}; }

/// Camera centre of S_cw once the scale is folded into the SE3 pose (R, t / scale).
TPoint3DF Centre(const Sim3& S_cw) { return -(S_cw.R.transpose() * S_cw.t) / S_cw.scale; }

TPoint3DF Centre(const RigidTransformation& T_cw) { return Centre(ToSim3(T_cw)); }

double AngleBetween(const TMatrix33F& R1, const TMatrix33F& R2) {
    return Eigen::AngleAxisd(R1.transpose() * R2).angle();
}

/// Frames on a 5 m circle, each looking along the tangent (true T_cw).
std::vector<RigidTransformation> Ring() {
    std::vector<RigidTransformation> poses;
    for (size_t k = 0; k < kRingFrames; ++k) {
        const double theta = 2.0 * M_PI * static_cast<double>(k) / kRingFrames;
        const TPoint3DF centre(5.0 * std::cos(theta), 0.0, 5.0 * std::sin(theta));
        const TMatrix33F R_wc(Eigen::AngleAxisd(-theta, TVector3DF::UnitY()));
        poses.push_back(RigidTransformation(R_wc.transpose(), -(R_wc.transpose() * centre)));
    }
    return poses;
}

/// Deterministic odometry drift: every step's relative motion gains a rotation of rotation_rad
/// (about a fixed tilted axis) and a translation bias of translation_m along x, and (mono) its
/// translation grows by (1 + scale_growth) per step. Frame 0 stays true.
struct Drift {
    double rotation_rad = 0.0;
    double translation_m = 0.0;
    double scale_growth = 0.0;
};

std::vector<RigidTransformation> Drifted(const std::vector<RigidTransformation>& truth, const Drift& drift) {
    const TMatrix33F R_drift(Eigen::AngleAxisd(drift.rotation_rad, TVector3DF(0.3, 1.0, 0.2).normalized()));
    std::vector<RigidTransformation> drifted{truth.front()};
    for (size_t k = 0; k + 1 < truth.size(); ++k) {
        const RigidTransformation relative = truth[k + 1] * truth[k].Inverse();
        const double growth = std::pow(1.0 + drift.scale_growth, static_cast<double>(k + 1));
        const RigidTransformation drifted_relative(R_drift * relative.rotation(),
                                                   growth * relative.translation() +
                                                       TVector3DF(drift.translation_m, 0.0, 0.0));
        drifted.push_back(drifted_relative * drifted.back());
    }
    return drifted;
}

/// A loop correction like ORB-SLAM3's CorrectLoop: the query's corrected Sim3 maps the true world
/// into its camera at the drifted map scale; the window Frames keep their relative poses to it.
/// Loop connections join each window Frame to Frames 0..2 across the loop. Edges are the chain
/// (spanning tree) plus covisibility to the Frame two back.
EssentialGraphProblem RingProblem(const std::vector<RigidTransformation>& truth,
                                  const std::vector<RigidTransformation>& drifted, double query_scale,
                                  bool fix_scale) {
    EssentialGraphProblem problem;
    problem.T_cw = drifted;
    problem.corrected_S_cw.assign(kRingFrames, std::nullopt);
    for (size_t k = 0; k < kRingFrames; ++k) {
        problem.frames.push_back(map::FrameId{static_cast<uint32_t>(10 * k)});
    }
    const Sim3 true_query{query_scale, truth[kQuery].rotation(), query_scale * truth[kQuery].translation()};
    const Sim3 S_drifted_true = Compose(Inverse(ToSim3(drifted[kQuery])), true_query);
    for (size_t k = kQuery + 1 - kWindow; k <= kQuery; ++k) {
        problem.corrected_S_cw[k] = Compose(ToSim3(drifted[k]), S_drifted_true);
        for (size_t m = 0; m < 3; ++m) {
            problem.loop_connections.emplace_back(k, m);
        }
    }
    for (size_t k = 0; k + 1 < kRingFrames; ++k) {
        problem.edges.emplace_back(k, k + 1);
        if (k + 2 < kRingFrames) {
            problem.edges.emplace_back(k, k + 2);
        }
    }
    problem.fixed = 0;
    problem.fix_scale = fix_scale;
    return problem;
}

double RmsCentreError(const std::vector<TPoint3DF>& centres, const std::vector<RigidTransformation>& truth) {
    double sum = 0.0;
    for (size_t k = 0; k < truth.size(); ++k) {
        sum += (centres[k] - Centre(truth[k])).squaredNorm();
    }
    return std::sqrt(sum / static_cast<double>(truth.size()));
}

std::vector<TPoint3DF> Centres(const std::vector<Sim3>& S_cw) {
    std::vector<TPoint3DF> centres;
    for (const Sim3& S : S_cw) {
        centres.push_back(Centre(S));
    }
    return centres;
}

std::vector<TPoint3DF> Centres(const std::vector<RigidTransformation>& T_cw) {
    std::vector<TPoint3DF> centres;
    for (const RigidTransformation& T : T_cw) {
        centres.push_back(Centre(T));
    }
    return centres;
}

// Stereo-like drift (rotation and translation, no scale) of three magnitudes: the end-to-start
// error (the query's pose relative to the fixed Frame 0) drops to <= 10 % of the injected drift,
// and the whole ring improves.
TEST(EssentialGraphTest, RingWithDriftIsCorrected) {
    const std::vector<RigidTransformation> truth = Ring();
    for (int magnitude = 1; magnitude <= 3; ++magnitude) {
        SCOPED_TRACE(magnitude);
        const Drift drift{magnitude * 0.2 * M_PI / 180.0, magnitude * 0.01, 0.0};
        const std::vector<RigidTransformation> drifted = Drifted(truth, drift);
        const EssentialGraphResult result = OptimizeEssentialGraph(RingProblem(truth, drifted, 1.0, true));
        ASSERT_TRUE(result.success);
        ASSERT_EQ(result.S_cw.size(), kRingFrames);

        const double drift_position = (Centre(drifted[kQuery]) - Centre(truth[kQuery])).norm();
        const double drift_rotation = AngleBetween(drifted[kQuery].rotation(), truth[kQuery].rotation());
        ASSERT_GT(drift_position, 0.1);
        EXPECT_LE((Centre(result.S_cw[kQuery]) - Centre(truth[kQuery])).norm(), 0.1 * drift_position);
        EXPECT_LE(AngleBetween(result.S_cw[kQuery].R, truth[kQuery].rotation()), 0.1 * drift_rotation);
        EXPECT_LE(RmsCentreError(Centres(result.S_cw), truth), 0.5 * RmsCentreError(Centres(drifted), truth));
    }
}

// Mono scale drift (the map grows 1-3 % per step, plus rotation drift): the vertices' scales
// absorb it, the end-to-start error drops to <= 10 % of the injected drift, and the step lengths
// of the SE3 trajectory (scale folded) return towards the true ones.
TEST(EssentialGraphTest, MonoScaleDriftIsCorrected) {
    const std::vector<RigidTransformation> truth = Ring();
    for (int magnitude = 1; magnitude <= 3; ++magnitude) {
        SCOPED_TRACE(magnitude);
        const Drift drift{magnitude * 0.1 * M_PI / 180.0, 0.0, magnitude * 0.01};
        const std::vector<RigidTransformation> drifted = Drifted(truth, drift);
        const double query_scale = std::pow(1.0 + drift.scale_growth, static_cast<double>(kQuery));
        const EssentialGraphResult result =
            OptimizeEssentialGraph(RingProblem(truth, drifted, query_scale, false));
        ASSERT_TRUE(result.success);
        ASSERT_EQ(result.S_cw.size(), kRingFrames);

        const double drift_position = (Centre(drifted[kQuery]) - Centre(truth[kQuery])).norm();
        ASSERT_GT(drift_position, 0.1);
        EXPECT_LE((Centre(result.S_cw[kQuery]) - Centre(truth[kQuery])).norm(), 0.1 * drift_position);
        EXPECT_LE(RmsCentreError(Centres(result.S_cw), truth), 0.5 * RmsCentreError(Centres(drifted), truth));

        // Mid-ring step length relative to the true one: drifted by (1 + g)^10, then mostly undone.
        const std::vector<TPoint3DF> optimized = Centres(result.S_cw);
        const std::vector<TPoint3DF> drifted_centres = Centres(drifted);
        const std::vector<TPoint3DF> true_centres = Centres(truth);
        const double true_step = (true_centres[10] - true_centres[9]).norm();
        const double drifted_ratio = (drifted_centres[10] - drifted_centres[9]).norm() / true_step;
        const double optimized_ratio = (optimized[10] - optimized[9]).norm() / true_step;
        EXPECT_LE(std::abs(optimized_ratio - 1.0), 0.5 * std::abs(drifted_ratio - 1.0));
        EXPECT_NE(result.S_cw[10].scale, 1.0);
    }
}

TEST(EssentialGraphTest, FixedVertexStays) {
    const std::vector<RigidTransformation> truth = Ring();
    const std::vector<RigidTransformation> drifted = Drifted(truth, Drift{0.5 * M_PI / 180.0, 0.02, 0.01});
    for (const size_t fixed : {size_t{0}, size_t{7}, kQuery}) {
        SCOPED_TRACE(fixed);
        EssentialGraphProblem problem = RingProblem(truth, drifted, 1.2, false);
        problem.fixed = fixed;
        const Sim3 start = problem.corrected_S_cw[fixed].value_or(ToSim3(problem.T_cw[fixed]));
        const EssentialGraphResult result = OptimizeEssentialGraph(problem);
        ASSERT_TRUE(result.success);
        EXPECT_EQ(result.S_cw[fixed].scale, start.scale);
        EXPECT_TRUE(result.S_cw[fixed].R.isApprox(start.R, 1e-12));
        EXPECT_TRUE(result.S_cw[fixed].t.isApprox(start.t, 1e-12));
        // Something else moved.
        const size_t other = fixed == 0 ? 10 : 0;
        const Sim3 other_start = problem.corrected_S_cw[other].value_or(ToSim3(problem.T_cw[other]));
        EXPECT_GT((result.S_cw[other].t - other_start.t).norm(), 1e-3);
    }
}

// also_fixed (map merging): every listed vertex keeps its start value along with `fixed`; the
// others still move.
TEST(EssentialGraphTest, AlsoFixedVerticesStay) {
    const std::vector<RigidTransformation> truth = Ring();
    const std::vector<RigidTransformation> drifted = Drifted(truth, Drift{0.5 * M_PI / 180.0, 0.02, 0.01});
    EssentialGraphProblem problem = RingProblem(truth, drifted, 1.2, false);
    problem.fixed = 0;
    problem.also_fixed = {kQuery, 7};
    const EssentialGraphResult result = OptimizeEssentialGraph(problem);
    ASSERT_TRUE(result.success);
    for (const size_t fixed : {size_t{0}, size_t{7}, kQuery}) {
        SCOPED_TRACE(fixed);
        const Sim3 start = problem.corrected_S_cw[fixed].value_or(ToSim3(problem.T_cw[fixed]));
        EXPECT_EQ(result.S_cw[fixed].scale, start.scale);
        EXPECT_TRUE(result.S_cw[fixed].R.isApprox(start.R, 1e-12));
        EXPECT_TRUE(result.S_cw[fixed].t.isApprox(start.t, 1e-12));
    }
    EXPECT_GT((result.S_cw[10].t - drifted[10].translation()).norm(), 1e-3);
}

// fix_scale: every vertex keeps its start scale (1 outside the window, the corrected one inside),
// though the scale drift pulls on it.
TEST(EssentialGraphTest, StereoKeepsScale) {
    const std::vector<RigidTransformation> truth = Ring();
    const std::vector<RigidTransformation> drifted = Drifted(truth, Drift{0.5 * M_PI / 180.0, 0.02, 0.02});
    const EssentialGraphProblem problem = RingProblem(truth, drifted, 1.0, true);
    const EssentialGraphResult result = OptimizeEssentialGraph(problem);
    ASSERT_TRUE(result.success);
    ASSERT_EQ(result.S_cw.size(), kRingFrames);
    for (size_t k = 0; k < kRingFrames; ++k) {
        EXPECT_EQ(result.S_cw[k].scale, 1.0) << k;
    }
    EXPECT_GT((result.S_cw[10].t - drifted[10].translation()).norm(), 1e-3);
}

// Two Frames, Frame 1 corrected away from its current pose. As a loop connection the edge is
// measured from the corrected value, so nothing moves; as an ordinary edge it is measured from
// the current poses, so Frame 1 returns to its current pose relative to the fixed Frame 0.
TEST(EssentialGraphTest, LoopConnectionsUseCorrectedValues) {
    EssentialGraphProblem problem;
    problem.frames = {map::FrameId{1}, map::FrameId{2}};
    const RigidTransformation T1(TMatrix33F(Eigen::AngleAxisd(0.2, TVector3DF::UnitY())), TVector3DF(1.0, 0.0, 0.0));
    problem.T_cw = {RigidTransformation::Identity(), T1};
    const Sim3 corrected{1.3, TMatrix33F(Eigen::AngleAxisd(0.3, TVector3DF::UnitY())), TVector3DF(1.5, 0.2, 0.0)};
    problem.corrected_S_cw = {std::nullopt, corrected};
    problem.fixed = 0;

    problem.loop_connections = {{1, 0}};
    EssentialGraphResult result = OptimizeEssentialGraph(problem);
    ASSERT_TRUE(result.success);
    EXPECT_NEAR(result.S_cw[1].scale, corrected.scale, 1e-9);
    EXPECT_TRUE(result.S_cw[1].R.isApprox(corrected.R, 1e-9));
    EXPECT_TRUE(result.S_cw[1].t.isApprox(corrected.t, 1e-9));

    problem.loop_connections.clear();
    problem.edges = {{1, 0}};
    result = OptimizeEssentialGraph(problem);
    ASSERT_TRUE(result.success);
    EXPECT_NEAR(result.S_cw[1].scale, 1.0, 1e-6);
    EXPECT_TRUE(result.S_cw[1].R.isApprox(T1.rotation(), 1e-6));
    EXPECT_TRUE(result.S_cw[1].t.isApprox(T1.translation(), 1e-6));
}

// A pair in both lists is one edge, measured as the loop connection: adding it (either order) to
// edges changes nothing.
TEST(EssentialGraphTest, DuplicatePairIsAddedOnce) {
    const std::vector<RigidTransformation> truth = Ring();
    const std::vector<RigidTransformation> drifted = Drifted(truth, Drift{0.5 * M_PI / 180.0, 0.02, 0.0});
    const EssentialGraphProblem problem = RingProblem(truth, drifted, 1.0, true);
    EssentialGraphProblem duplicated = problem;
    duplicated.edges.emplace_back(0, kQuery);
    duplicated.edges.emplace_back(kQuery, 1);
    duplicated.loop_connections.emplace_back(kQuery, 0);

    const EssentialGraphResult expected = OptimizeEssentialGraph(problem);
    const EssentialGraphResult result = OptimizeEssentialGraph(duplicated);
    ASSERT_TRUE(expected.success);
    ASSERT_TRUE(result.success);
    for (size_t k = 0; k < kRingFrames; ++k) {
        EXPECT_EQ(result.S_cw[k].scale, expected.S_cw[k].scale) << k;
        EXPECT_TRUE(result.S_cw[k].R.isApprox(expected.S_cw[k].R, 1e-12)) << k;
        EXPECT_TRUE(result.S_cw[k].t.isApprox(expected.S_cw[k].t, 1e-12)) << k;
    }
}

TEST(EssentialGraphTest, NoEdgesReturnsStartValues) {
    EssentialGraphProblem problem;
    problem.frames = {map::FrameId{1}, map::FrameId{2}};
    problem.T_cw = {RigidTransformation::Identity(), RigidTransformation(TMatrix33F::Identity(), TVector3DF(1, 2, 3))};
    const Sim3 corrected{2.0, TMatrix33F::Identity(), TVector3DF(0.5, 0.0, 0.0)};
    problem.corrected_S_cw = {corrected, std::nullopt};
    const EssentialGraphResult result = OptimizeEssentialGraph(problem);
    ASSERT_TRUE(result.success);
    ASSERT_EQ(result.S_cw.size(), 2u);
    EXPECT_EQ(result.S_cw[0].scale, 2.0);
    EXPECT_EQ(result.S_cw[0].t, corrected.t);
    EXPECT_EQ(result.S_cw[1].scale, 1.0);
    EXPECT_EQ(result.S_cw[1].t, TVector3DF(1, 2, 3));
}

TEST(EssentialGraphTest, InvalidInputFails) {
    // A small chain whose last Frame is corrected and connected back to Frame 0.
    constexpr size_t kFrames = 4;
    EssentialGraphProblem valid;
    for (size_t k = 0; k < kFrames; ++k) {
        valid.frames.push_back(map::FrameId{static_cast<uint32_t>(k)});
        valid.T_cw.emplace_back(TMatrix33F::Identity(), TVector3DF(static_cast<double>(k), 0.0, 0.0));
        valid.corrected_S_cw.emplace_back(std::nullopt);
        if (k > 0) {
            valid.edges.emplace_back(k - 1, k);
        }
    }
    valid.corrected_S_cw.back() = Sim3{1.1, TMatrix33F::Identity(), TVector3DF(2.5, 0.0, 0.0)};
    valid.loop_connections = {{kFrames - 1, 0}};
    ASSERT_TRUE(OptimizeEssentialGraph(valid).success);

    const double nan = std::numeric_limits<double>::quiet_NaN();
    std::vector<EssentialGraphProblem> invalid;
    invalid.emplace_back();  // empty
    {
        EssentialGraphProblem p = valid;
        p.frames.pop_back();
        invalid.push_back(p);
    }
    {
        EssentialGraphProblem p = valid;
        p.corrected_S_cw.pop_back();
        invalid.push_back(p);
    }
    {
        EssentialGraphProblem p = valid;
        p.fixed = kFrames;
        invalid.push_back(p);
    }
    {
        EssentialGraphProblem p = valid;
        p.also_fixed = {1, kFrames};
        invalid.push_back(p);
    }
    {
        EssentialGraphProblem p = valid;
        p.edges.emplace_back(0, kFrames);
        invalid.push_back(p);
    }
    {
        EssentialGraphProblem p = valid;
        p.loop_connections.emplace_back(kFrames, 0);
        invalid.push_back(p);
    }
    {
        EssentialGraphProblem p = valid;
        p.edges.emplace_back(2, 2);
        invalid.push_back(p);
    }
    {
        EssentialGraphProblem p = valid;
        p.T_cw[1] = RigidTransformation(TMatrix33F::Identity(), TVector3DF(nan, 0.0, 0.0));
        invalid.push_back(p);
    }
    {
        EssentialGraphProblem p = valid;
        p.corrected_S_cw[kFrames - 1]->scale = 0.0;
        invalid.push_back(p);
    }
    {
        EssentialGraphProblem p = valid;
        p.corrected_S_cw[kFrames - 1]->R(0, 0) = nan;
        invalid.push_back(p);
    }
    {
        EssentialGraphProblem p = valid;
        p.iterations = 0;
        invalid.push_back(p);
    }
    for (size_t i = 0; i < invalid.size(); ++i) {
        const EssentialGraphResult result = OptimizeEssentialGraph(invalid[i]);
        EXPECT_FALSE(result.success) << i;
        EXPECT_TRUE(result.S_cw.empty()) << i;
    }
}

}  // namespace
}  // namespace voyahslam::optimization
