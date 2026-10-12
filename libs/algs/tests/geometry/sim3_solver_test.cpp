#include <gtest/gtest.h>

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <random>
#include <utility>
#include <vector>

#include <Eigen/Dense>

#include "voyahslam/geometry/camera.h"
#include "voyahslam/geometry/distortion_models/kannala_brandt_distortion_model.h"
#include "voyahslam/geometry/sim3.h"
#include "voyahslam/geometry/sim3_solver.h"

namespace voyahslam::geometry {
namespace {

constexpr double kDeg = M_PI / 180.0;
// Per-axis noise, in pixels, on each pixel and (as pixel-equivalents at its depth) on each 3-D
// point. A pair's residual in one image combines one 3-D point and one pixel, so its per-axis
// σ is √2 · kNoisePx = 1 px: the σ the χ² 9.21 threshold assumes (sigmas are 1).
constexpr double kNoisePx = M_SQRT1_2;

// TUM-VI cam0 (equidistant fisheye, 1024x1024).
distortion_models::TDistortionModelPtr TumViModel() {
    return std::make_shared<const distortion_models::KannalaBrandtDistortionModel>(0.010171079892421483, -0.010816440029919381,
                                                           0.005942781769412756, -0.001662284667857643);
}

Camera TumViCamera(distortion_models::TDistortionModelPtr model) {
    Camera camera;
    camera.K << 380.81042871360756, 0.0, 510.29465304840727,
                0.0, 380.81194179427075, 514.3304630538506,
                0.0, 0.0, 1.0;
    camera.distortion = std::move(model);
    return camera;
}

TVector3DF RandomUnit(std::mt19937& rng) {
    std::normal_distribution<double> normal(0.0, 1.0);
    return TVector3DF(normal(rng), normal(rng), normal(rng)).normalized();
}

// Unit ray at angle theta from the optical axis, uniform in [0, max_theta], random azimuth.
TVector3DF RandomRay(std::mt19937& rng, double max_theta) {
    std::uniform_real_distribution<double> theta_dist(0.0, max_theta);
    std::uniform_real_distribution<double> phi_dist(-M_PI, M_PI);
    const double theta = theta_dist(rng);
    const double phi = phi_dist(rng);
    return TVector3DF(std::sin(theta) * std::cos(phi), std::sin(theta) * std::sin(phi), std::cos(theta));
}

Sim3 RandomSim3(std::mt19937& rng, double min_scale, double max_scale) {
    std::uniform_real_distribution<double> angle(-M_PI, M_PI);
    std::uniform_real_distribution<double> offset(-2.0, 2.0);
    std::uniform_real_distribution<double> log_scale(std::log(min_scale), std::log(max_scale));
    return Sim3{std::exp(log_scale(rng)), Eigen::AngleAxisd(angle(rng), RandomUnit(rng)).toRotationMatrix(),
                TVector3DF(offset(rng), offset(rng), offset(rng))};
}

bool IsFinite(const Sim3& S) {
    return std::isfinite(S.scale) && S.R.allFinite() && S.t.allFinite();
}

double RotationErrorDeg(const Sim3& a, const Sim3& b) {
    return Eigen::AngleAxisd(a.R.transpose() * b.R).angle() / kDeg;
}

// Correspondences for SolveSim3Ransac: pair i is X1[i] (camera-1 frame) seen at pixels1[i], and
// X2[i] (camera-2 frame) seen at pixels2[i]; inliers satisfy X1 = S_12 · X2.
struct Sim3Scene {
    Sim3 S_12;
    std::vector<TPoint3DF> X1;
    std::vector<TPoint2DF> pixels1;
    std::vector<double> sigmas1;
    std::vector<TPoint3DF> X2;
    std::vector<TPoint2DF> pixels2;
    std::vector<double> sigmas2;
};

// A point in front of a camera: depth 1..5 m along a ray up to 60°.
TPoint3DF RandomVisiblePoint(std::mt19937& rng) {
    std::uniform_real_distribution<double> depth(1.0, 5.0);
    return depth(rng) * RandomRay(rng, 60.0 * kDeg);
}

// S_21 of a loop-closure-like pair of views: camera 2 is camera 1 moved by up to 20° and
// 0.3 m per axis, in a map whose scale is 1/scale times camera 1's (mono drift; S_12.scale = scale).
Sim3 RandomLoopS21(std::mt19937& rng, double scale) {
    std::uniform_real_distribution<double> small_angle(-20.0 * kDeg, 20.0 * kDeg);
    std::uniform_real_distribution<double> offset(-0.3, 0.3);
    const TMatrix33F R_21 = Eigen::AngleAxisd(small_angle(rng), RandomUnit(rng)).toRotationMatrix();
    const TVector3DF t_21(offset(rng), offset(rng), offset(rng));
    return Sim3{1.0 / scale, R_21, t_21 / scale};
}

// n pairs under RandomLoopS21; every true point is in front of both cameras (within 80° of
// camera 2's axis). Each 3-D point carries isotropic noise of noise_px pixel-equivalents at its
// depth (map-point error), and each pixel is the projection of the true point plus noise_px noise.
Sim3Scene RandomSim3Scene(std::mt19937& rng, const Camera& camera, size_t n, double scale, double noise_px) {
    std::normal_distribution<double> noise(0.0, 1.0);
    const Sim3 S_21 = RandomLoopS21(rng, scale);
    Sim3Scene scene;
    scene.S_12 = Inverse(S_21);
    const double f = camera.K(0, 0);
    const auto noisy_point = [&](const TPoint3DF& X) {
        return TPoint3DF(X + (noise_px * X.z() / f) * TVector3DF(noise(rng), noise(rng), noise(rng)));
    };
    const auto noisy_pixel = [&](const TPoint3DF& X) {
        return TPoint2DF(camera.Project(X) + noise_px * TPoint2DF(noise(rng), noise(rng)));
    };
    while (scene.X1.size() < n) {
        const TPoint3DF X1 = RandomVisiblePoint(rng);
        const TPoint3DF X2 = Apply(S_21, X1);
        if (!(X2.z() > 0.2 / scale)) continue;
        if (std::acos(X2.normalized().z()) > 80.0 * kDeg) continue;
        scene.X1.push_back(noisy_point(X1));
        scene.pixels1.push_back(noisy_pixel(X1));
        scene.sigmas1.push_back(1.0);
        scene.X2.push_back(noisy_point(X2));
        scene.pixels2.push_back(noisy_pixel(X2));
        scene.sigmas2.push_back(1.0);
    }
    return scene;
}

// Pair i becomes a wrong match: X2 is another point in front of camera 2, seen at its own pixel.
void MakeOutlier(std::mt19937& rng, const Camera& camera, size_t i, double scale, Sim3Scene& out_scene) {
    const TPoint3DF X2 = RandomVisiblePoint(rng) / scale;
    out_scene.X2[i] = X2;
    out_scene.pixels2[i] = camera.Project(X2);
}

Sim3RansacResult Solve(const Camera& camera, const Sim3Scene& scene, const Sim3RansacOptions& options) {
    return SolveSim3Ransac(camera, camera, scene.X1, scene.pixels1, scene.sigmas1, scene.X2, scene.pixels2,
                           scene.sigmas2, options);
}

TEST(Sim3Test, HornRecoversNoiseFree) {
    std::mt19937 rng(1);
    for (int trial = 0; trial < 50; ++trial) {
        for (const bool fix_scale : {false, true}) {
            const Sim3 truth = RandomSim3(rng, fix_scale ? 1.0 : 0.2, fix_scale ? 1.0 : 5.0);
            std::vector<TPoint3DF> from;
            std::vector<TPoint3DF> to;
            const size_t n = trial % 2 == 0 ? 3 : 10;  // the minimal sample and an overdetermined set
            for (size_t i = 0; i < n; ++i) {
                from.push_back(TPoint3DF::Random() * 3.0);
                to.push_back(Apply(truth, from.back()));
            }
            const std::optional<Sim3> S = AlignHorn(from, to, fix_scale);
            ASSERT_TRUE(S.has_value()) << "trial " << trial;
            EXPECT_NEAR(S->scale, truth.scale, 1e-9) << "trial " << trial;
            EXPECT_LT((S->R - truth.R).norm(), 1e-9) << "trial " << trial;
            EXPECT_LT((S->t - truth.t).norm(), 1e-9) << "trial " << trial;
            EXPECT_NEAR(S->R.determinant(), 1.0, 1e-12);
        }
    }
}

// Contract (Task 5 verifies a loop candidate with it, then OptimizeSim3 refines): over 200
// random scenes (seeds 0..199, not hand-picked) with 100 pairs, half of them wrong matches, and
// a residual σ of 1 px (kNoisePx), in at least 95 % of the scenes RANSAC succeeds with (a) an
// inlier mask at least 90 % correct and (b) the raw S_12 within 1° and 2 % scale. Measured:
// 197/200 and 200/200 (rotation p95 0.38°, scale p95 0.4 %). At a residual σ of √2 px (1 px on
// pixels and on points) even the true S_12 flags >= 40 of the 50 inliers in only 181/200.
TEST(Sim3Test, RansacHalfOutliers) {
    const auto model = TumViModel();
    const Camera camera = TumViCamera(model);
    constexpr uint64_t kScenes = 200;
    size_t mask_ok = 0;
    size_t accurate = 0;
    for (uint64_t seed = 0; seed < kScenes; ++seed) {
        std::mt19937 rng(static_cast<std::mt19937::result_type>(seed));
        std::uniform_real_distribution<double> log_scale(std::log(0.5), std::log(2.0));
        const double scale = std::exp(log_scale(rng));
        Sim3Scene scene = RandomSim3Scene(rng, camera, 100, scale, kNoisePx);
        for (size_t i = 50; i < 100; ++i) MakeOutlier(rng, camera, i, scale, scene);

        Sim3RansacOptions options;
        options.seed = seed;
        const Sim3RansacResult result = Solve(camera, scene, options);
        EXPECT_GE(result.iterations, 1);
        EXPECT_LE(result.iterations, options.max_iterations);
        EXPECT_TRUE(IsFinite(result.S_12));
        if (!result.success) continue;
        ASSERT_EQ(result.inlier.size(), 100u);
        size_t correct = 0;
        size_t flagged = 0;
        for (size_t i = 0; i < 100; ++i) {
            correct += result.inlier[i] == (i < 50) ? 1 : 0;
            flagged += result.inlier[i] ? 1 : 0;
        }
        EXPECT_EQ(flagged, result.num_inliers);
        mask_ok += correct >= 90 ? 1 : 0;
        accurate += RotationErrorDeg(result.S_12, scene.S_12) < 1.0 &&
                            std::abs(result.S_12.scale / scene.S_12.scale - 1.0) < 0.02
                        ? 1
                        : 0;
    }
    EXPECT_GE(mask_ok, 190u) << "RANSAC success and mask >= 90 % correct in " << mask_ok << "/" << kScenes;
    EXPECT_GE(accurate, 190u) << "RANSAC success within 1° / 2 % scale in " << accurate << "/" << kScenes;
}

// Stereo / RGB-D maps are metric: with fix_scale the scale stays exactly 1, and the pose is
// still recovered.
TEST(Sim3Test, FixScaleKeepsUnitScale) {
    {  // AlignHorn on data whose best similarity has scale 2: the scale stays 1.
        std::mt19937 rng(3);
        const Sim3 truth = RandomSim3(rng, 2.0, 2.0);
        std::vector<TPoint3DF> from;
        std::vector<TPoint3DF> to;
        for (int i = 0; i < 10; ++i) {
            from.push_back(TPoint3DF::Random());
            to.push_back(Apply(truth, from.back()));
        }
        const std::optional<Sim3> S = AlignHorn(from, to, /*fix_scale=*/true);
        ASSERT_TRUE(S.has_value());
        EXPECT_EQ(S->scale, 1.0);
        EXPECT_LT((S->R - truth.R).norm(), 1e-9);  // the rotation does not depend on the scale
    }
    const auto model = TumViModel();
    const Camera camera = TumViCamera(model);
    for (uint64_t seed = 0; seed < 50; ++seed) {
        std::mt19937 rng(static_cast<std::mt19937::result_type>(seed));
        Sim3Scene scene = RandomSim3Scene(rng, camera, 100, 1.0, kNoisePx);
        for (size_t i = 70; i < 100; ++i) MakeOutlier(rng, camera, i, 1.0, scene);
        Sim3RansacOptions options;
        options.fix_scale = true;
        options.seed = seed;
        const Sim3RansacResult result = Solve(camera, scene, options);
        ASSERT_TRUE(result.success) << "seed " << seed;
        EXPECT_EQ(result.S_12.scale, 1.0) << "seed " << seed;
        EXPECT_LT(RotationErrorDeg(result.S_12, scene.S_12), 1.0) << "seed " << seed;
    }
}

void ExpectCleanFailure(const Sim3RansacResult& result) {
    EXPECT_FALSE(result.success);
    EXPECT_TRUE(IsFinite(result.S_12));
    EXPECT_EQ(result.num_inliers, 0u);
}

// Every data-dependent case runs over seeds 0..49 (not hand-picked).
TEST(Sim3Test, DegenerateFailsCleanly) {
    const auto model = TumViModel();
    const Camera camera = TumViCamera(model);

    {  // Fewer than 3 pairs, and mismatched sizes.
        std::mt19937 rng(0);
        Sim3Scene scene = RandomSim3Scene(rng, camera, 2, 1.0, 0.0);
        Sim3RansacOptions options;
        options.min_inliers = 1;
        ExpectCleanFailure(Solve(camera, scene, options));
        EXPECT_FALSE(AlignHorn(scene.X2, scene.X1, false).has_value());

        scene = RandomSim3Scene(rng, camera, 30, 1.0, 0.0);
        scene.sigmas2.pop_back();
        ExpectCleanFailure(Solve(camera, scene, options));
        std::vector<TPoint3DF> shorter = scene.X1;
        shorter.pop_back();
        EXPECT_FALSE(AlignHorn(scene.X2, shorter, false).has_value());
    }
    {  // Zero cross-covariance (M = 0) with both sets non-collinear: the best scale is 0 and the
       // rotation is undetermined.
        const std::vector<TPoint3DF> from{{1, 0, 0}, {-1, 0, 0}, {0, 1, 0}, {0, -1, 0}, {0, 0, 0}, {0, 0, 0}};
        const TPoint3DF p(0, 0, 1);
        const TPoint3DF q(0, 1, 0);
        const std::vector<TPoint3DF> to{p, p, -p, -p, q, -q};
        EXPECT_FALSE(AlignHorn(from, to, false).has_value());
        EXPECT_FALSE(AlignHorn(from, to, true).has_value());
    }
    for (uint64_t seed = 0; seed < 50; ++seed) {
        std::mt19937 rng(static_cast<std::mt19937::result_type>(seed));
        Sim3RansacOptions options;
        options.seed = seed;

        {  // 40 noise-free pairs on one 3-D line: the rotation about the line is unobservable.
            const Sim3 S_21 = RandomLoopS21(rng, 1.0);
            Sim3Scene scene;
            const TPoint3DF origin(-0.5, 0.3, 3.0);
            const TVector3DF direction = RandomUnit(rng);
            for (int i = 0; i < 40; ++i) {
                const TPoint3DF X1 = origin + 0.02 * i * direction;
                const TPoint3DF X2 = Apply(S_21, X1);
                if (!(X1.z() > 0.5 && X2.z() > 0.5)) continue;
                scene.X1.push_back(X1);
                scene.pixels1.push_back(camera.Project(X1));
                scene.sigmas1.push_back(1.0);
                scene.X2.push_back(X2);
                scene.pixels2.push_back(camera.Project(X2));
                scene.sigmas2.push_back(1.0);
            }
            ASSERT_GE(scene.X1.size(), 20u) << "seed " << seed;
            ExpectCleanFailure(Solve(camera, scene, options));
            EXPECT_FALSE(AlignHorn(scene.X2, scene.X1, false).has_value()) << "seed " << seed;
            EXPECT_FALSE(AlignHorn(scene.X2, scene.X1, true).has_value()) << "seed " << seed;
        }
        {  // All 100 pairs are wrong matches.
            Sim3Scene scene = RandomSim3Scene(rng, camera, 100, 1.0, kNoisePx);
            for (size_t i = 0; i < 100; ++i) MakeOutlier(rng, camera, i, 1.0, scene);
            ExpectCleanFailure(Solve(camera, scene, options));
        }
        {  // The only consistent map is a mirror, X2 = diag(-1, 1, 1) X1 = -1 · R_x(π) X1: scale -1.
            Sim3Scene scene = RandomSim3Scene(rng, camera, 100, 1.0, 0.0);
            for (size_t i = 0; i < 100; ++i) {
                scene.X2[i] = TPoint3DF(-scene.X1[i].x(), scene.X1[i].y(), scene.X1[i].z());
                scene.pixels2[i] = camera.Project(scene.X2[i]);
            }
            ExpectCleanFailure(Solve(camera, scene, options));
        }
    }
}

}  // namespace
}  // namespace voyahslam::geometry
