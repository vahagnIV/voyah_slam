#include <gtest/gtest.h>
#include <support/test_measurements.h>
#include <voyahslam/events/events.h>
#include <voyahslam/global_ba/global_ba.h>
#include <voyahslam/memory/object_pool.h>
#include <voyahslam/tracking/map_replica.h>

#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <optional>
#include <random>
#include <utility>
#include <vector>

namespace voyahslam::global_ba {
namespace {

using Extractor = test::TestExtractor;
using TReplica = tracking::MapReplica<Extractor>;
using TTransaction = events::MapTransaction<Extractor>;
using geometry::RigidTransformation;
using geometry::TPoint3DF;

constexpr uint32_t kMap = 1;
constexpr size_t kNumFrames = 6;

// A deterministic scene: a 7 x 5 grid of points at depths 5 and 7 m in front
// of a camera moving 0.2 m to the right and turning 0.02 rad per Frame.
struct Scene {
    std::vector<TPoint3DF> points;
    std::vector<RigidTransformation> poses;  // T_cw
};

Scene MakeScene() {
    Scene scene;
    for (const double z : {5.0, 7.0}) {
        for (int ix = -3; ix <= 3; ++ix) {
            for (int iy = -2; iy <= 2; ++iy) {
                scene.points.emplace_back(0.5 * ix, 0.5 * iy, z);
            }
        }
    }
    for (size_t k = 0; k < kNumFrames; ++k) {
        const double step = static_cast<double>(k);
        const Eigen::Matrix3d R_wc = Eigen::AngleAxisd(0.02 * step, Eigen::Vector3d::UnitY()).toRotationMatrix();
        const RigidTransformation T_wc(R_wc.cast<precision_t>(),
                                       Eigen::Vector3d(0.2 * step, 0.0, 0.0).cast<precision_t>());
        scene.poses.push_back(T_wc.Inverse());
    }
    return scene;
}

const features::StereoRig& Rig() {
    static const features::StereoRig rig{
        .K_left = test::TestIntrinsics(),
        .K_right = test::TestIntrinsics(),
        .T_rl = RigidTransformation(geometry::TMatrix33F::Identity(), geometry::TVector3DF(-0.1, 0.0, 0.0)),
        .baseline = 0.1};
    return rig;
}

test::TestImageFeatures MakeFeatures(const geometry::TPoints2DF& pixels) {
    const auto n = pixels.cols();
    return test::TestImageFeatures(geometry::TPoints2DF(pixels), geometry::TRowVectorF::Ones(n),
                                   geometry::TRowVectorF::Zero(n),
                                   test::TestImageFeatures::TDescriptors::Zero(Extractor::DescriptorSize, n),
                                   features::TLevels::Zero(n));
}

// The exact measurement of every scene point by a camera at true pose T_cw:
// feature i is point i (stereo: right feature i too, matched).
features::TMeasurement<Extractor> Measure(const Scene& scene, const RigidTransformation& T_cw, bool stereo) {
    const auto n = static_cast<Eigen::Index>(scene.points.size());
    const geometry::TMatrix33F K = test::TestIntrinsics();
    geometry::TPoints2DF left(2, n);
    geometry::TPoints2DF right(2, n);
    geometry::TRowVectorF depths(n);
    for (Eigen::Index i = 0; i < n; ++i) {
        const TPoint3DF Xc = T_cw.Transform(scene.points[static_cast<size_t>(i)]);
        left.col(i) = (K * Xc).hnormalized();
        right.col(i) = (K * Rig().T_rl.Transform(Xc)).hnormalized();
        depths(i) = Xc.z();
    }
    if (!stereo) {
        return features::TMonocularMeasurement<Extractor>(MakeFeatures(left), K, nullptr, 0.0);
    }
    features::TBinocularMeasurement<Extractor> measurement(MakeFeatures(left), MakeFeatures(right), Rig(), 0.0);
    measurement.SetStereoMatches({Eigen::VectorXi::LinSpaced(n, 0, static_cast<int>(n) - 1), depths,
                                  static_cast<size_t>(n)});
    return measurement;
}

// Applies one transaction of map kMap, filled by `fill`.
template <typename TFill>
void ApplyTransaction(TReplica& out_replica, TFill fill) {
    memory::ObjectPool<TTransaction, memory::ThreadSafePolicy> pool(1);
    TTransaction* event = pool.Create();
    event->map_id = {kMap};
    fill(*event);
    out_replica.Apply(*event);
    event->Recycle();
}

// Builds the replica of `scene` in one transaction: Frame k at map pose
// map_poses[k], observing every point; point i at map position map_points[i].
void BuildMap(TReplica& out_replica, const Scene& scene, const std::vector<RigidTransformation>& map_poses,
              const std::vector<TPoint3DF>& map_points, bool stereo) {
    ApplyTransaction(out_replica, [&](TTransaction& event) {
        event.new_map = true;
        for (size_t k = 0; k < map_poses.size(); ++k) {
            events::NewFrame<Extractor>& added = event.added_frames.emplace_back();
            added.id = {static_cast<uint32_t>(k)};
            added.T_cw = map_poses[k];
            added.measurement = Measure(scene, scene.poses[k], stereo);
            if (k > 0) added.parent = map::FrameId{static_cast<uint32_t>(k - 1)};
            for (size_t i = 0; i < map_points.size(); ++i) {
                added.observed_features.push_back(static_cast<uint32_t>(i));
                added.observed_points.push_back({static_cast<uint32_t>(i)});
            }
        }
        event.new_points_start_id = {0};
        event.ResizeNewPoints(static_cast<uint32_t>(map_points.size()));
        for (size_t i = 0; i < map_points.size(); ++i) {
            const auto col = static_cast<Eigen::Index>(i);
            event.new_positions.col(col) = map_points[i];
            event.new_descriptors.col(col).setZero();
            event.new_normals.col(col) = TPoint3DF(0, 0, -1);
        }
    });
}

void BuildExactMap(TReplica& out_replica, const Scene& scene, bool stereo) {
    BuildMap(out_replica, scene, scene.poses, scene.points, stereo);
}

const std::function<bool()> kNeverStop;

bool SamePose(const RigidTransformation& a, const RigidTransformation& b) {
    return a.rotation() == b.rotation() && a.translation() == b.translation();
}

TEST(GlobalBaProblemTest, IncludesEveryFrameAndObservedPoint) {
    const Scene scene = MakeScene();
    TReplica replica;
    BuildExactMap(replica, scene, /*stereo=*/true);
    const auto num_scene = static_cast<uint32_t>(scene.points.size());
    // Point 3 removed; two more alive points that no Frame observes, added
    // with Frame 100 (no features).
    ApplyTransaction(replica, [&](TTransaction& event) { event.removed_points = {{3}}; });
    ApplyTransaction(replica, [&](TTransaction& event) {
        events::NewFrame<Extractor>& added = event.added_frames.emplace_back();
        added.id = {100};
        added.T_cw = scene.poses[2];
        added.measurement = features::TMonocularMeasurement<Extractor>(
            MakeFeatures(geometry::TPoints2DF(2, 0)), test::TestIntrinsics(), nullptr, 0.0);
        event.new_points_start_id = {num_scene};
        event.ResizeNewPoints(2);
        for (Eigen::Index i = 0; i < 2; ++i) {
            event.new_positions.col(i) = TPoint3DF(1, 1, 1);
            event.new_descriptors.col(i).setZero();
            event.new_normals.col(i) = TPoint3DF(0, 0, 1);
        }
    });
    // Alive without an observation: off the replica's invariant (as
    // MapReplicaTest's AddPoints), but RunGlobalBa must leave such a point out.
    ASSERT_TRUE(replica.IsAlive({num_scene}));
    ASSERT_EQ(replica.covisibility().NumObservations({num_scene}), 0u);

    const std::optional<GlobalBaSolution> solution = RunGlobalBa(replica, GlobalBaOptions{}, kNeverStop);
    ASSERT_TRUE(solution.has_value());

    std::vector<uint32_t> frames;
    for (const auto& [id, pose] : solution->frame_poses) frames.push_back(id.value);
    std::sort(frames.begin(), frames.end());
    EXPECT_EQ(frames, (std::vector<uint32_t>{0, 1, 2, 3, 4, 5, 100}));

    std::vector<uint32_t> points;
    for (const auto& [id, position] : solution->point_positions) points.push_back(id.value);
    std::sort(points.begin(), points.end());
    std::vector<uint32_t> expected;
    for (uint32_t i = 0; i < num_scene; ++i) {
        if (i != 3) expected.push_back(i);
    }
    EXPECT_EQ(points, expected);
}

TEST(GlobalBaProblemTest, FixesFirstFrame) {
    const Scene scene = MakeScene();
    std::vector<RigidTransformation> poses = scene.poses;
    // A first Frame off the identity, so that "unchanged" is not trivial.
    poses[0] = RigidTransformation(
        Eigen::AngleAxisd(0.05, Eigen::Vector3d::UnitX()).toRotationMatrix().cast<precision_t>(),
        geometry::TVector3DF(0.01, -0.02, 0.03));
    TReplica replica;
    BuildMap(replica, scene, poses, scene.points, /*stereo=*/true);

    const std::optional<GlobalBaSolution> solution = RunGlobalBa(replica, GlobalBaOptions{}, kNeverStop);
    ASSERT_TRUE(solution.has_value());
    EXPECT_EQ(solution->fixed_frame, map::FrameId{0});
    const auto fixed = std::find_if(solution->frame_poses.begin(), solution->frame_poses.end(),
                                    [](const auto& entry) { return entry.first == map::FrameId{0}; });
    ASSERT_NE(fixed, solution->frame_poses.end());
    EXPECT_TRUE(SamePose(fixed->second, replica.keyframe({0})->T_cw));
}

TEST(GlobalBaProblemTest, FixesOldestSurvivingFrame) {
    const Scene scene = MakeScene();
    TReplica replica;
    BuildExactMap(replica, scene, /*stereo=*/true);
    ApplyTransaction(replica, [](TTransaction& event) { event.removed_frames = {{0}}; });
    ASSERT_EQ(replica.keyframe({0}), nullptr);

    const std::optional<GlobalBaSolution> solution = RunGlobalBa(replica, GlobalBaOptions{}, kNeverStop);
    ASSERT_TRUE(solution.has_value());
    EXPECT_EQ(solution->fixed_frame, map::FrameId{1});
    EXPECT_EQ(solution->frame_poses.size(), kNumFrames - 1);
    for (const auto& [id, pose] : solution->frame_poses) {
        EXPECT_NE(id, map::FrameId{0});
        if (id == map::FrameId{1}) EXPECT_TRUE(SamePose(pose, replica.keyframe({1})->T_cw));
    }
}

// The RMS distance of the map's points to the true ones. Mono: after the
// best scale about the fixed first camera (the world origin), the gauge a
// mono BA leaves free.
double PointRms(const std::vector<TPoint3DF>& estimated, const std::vector<TPoint3DF>& truth, bool mono) {
    double scale = 1.0;
    if (mono) {
        double dot = 0.0;
        double norm = 0.0;
        for (size_t i = 0; i < truth.size(); ++i) {
            dot += estimated[i].dot(truth[i]);
            norm += estimated[i].squaredNorm();
        }
        scale = dot / norm;
    }
    double sum = 0.0;
    for (size_t i = 0; i < truth.size(); ++i) sum += (scale * estimated[i] - truth[i]).squaredNorm();
    return std::sqrt(sum / static_cast<double>(truth.size()));
}

TEST(GlobalBaProblemTest, RecoversPerturbedMap) {
    const Scene scene = MakeScene();
    for (const bool stereo : {true, false}) {
        for (uint32_t seed = 1; seed <= 5; ++seed) {
            SCOPED_TRACE(testing::Message() << (stereo ? "stereo" : "mono") << " seed " << seed);
            std::mt19937 rng(seed);
            std::normal_distribution<double> noise(0.0, 1.0);
            const auto vector = [&](double sigma) {
                return Eigen::Vector3d(sigma * noise(rng), sigma * noise(rng), sigma * noise(rng));
            };
            // Frame 0 is the gauge (fixed at its true pose); the others are
            // off by about 0.01 rad and 5 cm, the points by about 10 cm.
            std::vector<RigidTransformation> poses = scene.poses;
            for (size_t k = 1; k < poses.size(); ++k) {
                const Eigen::Vector3d rotation = vector(0.01);
                const RigidTransformation delta(
                    Eigen::AngleAxisd(rotation.norm(), rotation.normalized()).toRotationMatrix().cast<precision_t>(),
                    vector(0.05).cast<precision_t>());
                poses[k] = delta * poses[k];
            }
            std::vector<TPoint3DF> points = scene.points;
            for (TPoint3DF& point : points) point += vector(0.1).cast<precision_t>();

            TReplica replica;
            BuildMap(replica, scene, poses, points, stereo);
            const std::optional<GlobalBaSolution> solution =
                RunGlobalBa(replica, GlobalBaOptions{}, kNeverStop);
            ASSERT_TRUE(solution.has_value());
            ASSERT_EQ(solution->point_positions.size(), scene.points.size());
            std::vector<TPoint3DF> optimized(scene.points.size());
            for (const auto& [id, position] : solution->point_positions) optimized[id.value] = position;

            const double before = PointRms(points, scene.points, !stereo);
            const double after = PointRms(optimized, scene.points, !stereo);
            EXPECT_LE(after, 0.3 * before) << "before " << before << " after " << after;
        }
    }
}

TEST(GlobalBaProblemTest, InterruptedReturnsNothing) {
    const Scene scene = MakeScene();
    TReplica replica;
    BuildExactMap(replica, scene, /*stereo=*/true);
    int calls = 0;
    const std::optional<GlobalBaSolution> solution = RunGlobalBa(replica, GlobalBaOptions{}, [&calls] {
        ++calls;
        return true;
    });
    EXPECT_FALSE(solution.has_value());
    EXPECT_GE(calls, 1);
}

TEST(GlobalBaProblemTest, EmptyMapReturnsNothing) {
    const TReplica replica;
    EXPECT_FALSE(RunGlobalBa(replica, GlobalBaOptions{}, kNeverStop).has_value());
}

} // namespace
} // namespace voyahslam::global_ba
