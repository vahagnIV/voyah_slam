#include <gtest/gtest.h>
#include <support/synthetic_sequence.h>
#include <support/test_measurements.h>
#include <voyahslam/bow/vocabulary.h>
#include <voyahslam/events/async_observer.h>
#include <voyahslam/events/events.h>
#include <voyahslam/events/observable.h>
#include <voyahslam/geometry/sim3.h>
#include <voyahslam/loop_closing/loop_closer.h>
#include <voyahslam/mapping/mapper.h>
#include <voyahslam/memory/object_pool.h>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <numbers>
#include <optional>
#include <random>
#include <string>
#include <utility>
#include <variant>
#include <vector>

namespace voyahslam::loop_closing {
namespace {

using Extractor = test::TestExtractor;
using TTransaction = events::MapTransaction<Extractor>;
using TVocabulary = bow::Vocabulary<Extractor>;
using TMeasurement = features::TMeasurement<Extractor>;
using TDescriptor = test::TestImageFeatures::TDescriptor;
using geometry::RigidTransformation;
using geometry::Sim3;
using geometry::TPoint3DF;

constexpr double kImageSize = 512.0;  // test::SyntheticIntrinsics()
constexpr int kBitFlips = 3;
constexpr double kPixelNoise = 0.5;
constexpr double kAngleNoise = 0.02;
constexpr size_t kExtras = 20;
constexpr size_t kRegionPoints = 400;
constexpr double kMapNoise = 0.03;  // m

// Byte 1 of a scene point's descriptor is one of these codes (pairwise >= 4
// bits apart), byte 0 its region's byte: 0x00 or 0xFF.
constexpr uint8_t kWordCodes[] = {0x00, 0xFF, 0x0F, 0xF0, 0x33, 0xCC, 0x55, 0xAA};

// A flat vocabulary of one word per (region byte, code): a descriptor goes to
// the word of its bytes 0 and 1 (the other bytes are zero in every word), so
// regions differ in their BoW vectors. Declared with L = 5, so levels_up 4
// files every feature under its word and SearchByBoW compares only features
// of one word.
std::shared_ptr<const TVocabulary> MakeRegionVocabulary() {
    using TDescriptors = typename Extractor::TDescriptors;
    constexpr size_t kNumCodes = std::size(kWordCodes);
    const size_t num_nodes = 1 + 2 * kNumCodes;
    TDescriptors descriptors = TDescriptors::Zero(Extractor::DescriptorSize, static_cast<Eigen::Index>(num_nodes));
    std::vector<double> weights(num_nodes, 1.0);
    weights[0] = 0.0;
    for (size_t region = 0; region < 2; ++region) {
        for (size_t code = 0; code < kNumCodes; ++code) {
            const auto node = static_cast<Eigen::Index>(1 + region * kNumCodes + code);
            descriptors(0, node) = region == 0 ? 0x00 : 0xFF;
            descriptors(1, node) = kWordCodes[code];
        }
    }
    return std::make_shared<const TVocabulary>(TVocabulary::FromTree(
        static_cast<int>(2 * kNumCodes), 5, std::vector<bow::NodeId>(num_nodes, 0), std::move(descriptors),
        std::move(weights)));
}

RigidTransformation Pose(double x, double y, double z, double yaw_deg) {
    const geometry::TMatrix33F R =
        Eigen::AngleAxis<double>(yaw_deg * std::numbers::pi / 180.0, geometry::TVector3DF::UnitY())
            .toRotationMatrix();
    const geometry::TVector3DF centre(x, y, z);
    return {R, -R * centre};
}

geometry::TVector3DF Centre(const RigidTransformation& T_cw) { return T_cw.Inverse().translation(); }

struct ScenePoint {
    TPoint3DF X;  // true world position
    TDescriptor descriptor;
    double angle;
};

// `count` points 4..8 m in front of camera `T_cw`, spread over its image;
// byte 0 of every descriptor is `word_byte`.
std::vector<ScenePoint> MakeRegion(size_t count, const RigidTransformation& T_cw, uint8_t word_byte,
                                   std::mt19937& rng) {
    std::uniform_real_distribution<double> pixel(30.0, kImageSize - 30.0);
    std::uniform_real_distribution<double> depth(4.0, 8.0);
    std::uniform_real_distribution<double> angle(0.0, 2.0 * std::numbers::pi);
    std::uniform_int_distribution<size_t> code(0, std::size(kWordCodes) - 1);
    const geometry::TMatrix33F K_inverse = test::SyntheticIntrinsics().inverse();
    const RigidTransformation T_wc = T_cw.Inverse();
    std::vector<ScenePoint> region;
    for (size_t i = 0; i < count; ++i) {
        const geometry::TVector3DF ray(pixel(rng), pixel(rng), 1.0);
        TDescriptor descriptor = test::detail::RandomDescriptor(rng);
        descriptor[0] = word_byte;
        descriptor[1] = kWordCodes[code(rng)];
        region.push_back({T_wc.Transform(TPoint3DF(depth(rng) * (K_inverse * ray))), descriptor, angle(rng)});
    }
    return region;
}

// The same points with each one's descriptor and angle taken by another: a
// place that looks like `region` with another 3-D layout.
std::vector<ScenePoint> LookAlike(std::vector<ScenePoint> region, std::mt19937& rng) {
    std::vector<size_t> order(region.size());
    for (size_t i = 0; i < order.size(); ++i) order[i] = i;
    std::shuffle(order.begin(), order.end(), rng);
    std::vector<ScenePoint> shuffled = region;
    for (size_t i = 0; i < region.size(); ++i) {
        shuffled[i].descriptor = region[order[i]].descriptor;
        shuffled[i].angle = region[order[i]].angle;
    }
    return shuffled;
}

// A Frame as the Mapper publishes it.
struct SceneFrame {
    map::FrameId id;
    RigidTransformation T_cw;  // in the map: drifted for a drifted revisit
    TMeasurement measurement;
    std::vector<uint32_t> features;
    std::vector<map::MapPointId> points;
};

// Frame `id` at true pose `T_true` (map pose `T_map`) observing every point of
// `region` in its image as point first_id + i, plus kExtras random features.
// Binocular frames carry the left image twice (detection reads the left one).
SceneFrame Observe(map::FrameId id, const RigidTransformation& T_true, const RigidTransformation& T_map,
                   const std::vector<ScenePoint>& region, uint32_t first_id, bool binocular, std::mt19937& rng) {
    const geometry::TMatrix33F K = test::SyntheticIntrinsics();
    std::normal_distribution<double> noise(0.0, 1.0);
    std::uniform_real_distribution<double> any_pixel(0.0, kImageSize);
    std::uniform_real_distribution<double> any_angle(0.0, 2.0 * std::numbers::pi);
    std::vector<geometry::TPoint2DF> pixels;
    std::vector<TDescriptor> descriptors;
    std::vector<double> angles;
    SceneFrame frame{id, T_map, {}, {}, {}};
    for (size_t i = 0; i < region.size(); ++i) {
        const TPoint3DF Xc = T_true.Transform(region[i].X);
        if (Xc.z() <= 0.0) continue;
        const geometry::TPoint2DF uv = (K * Xc).hnormalized() + kPixelNoise * geometry::TPoint2DF(noise(rng), noise(rng));
        if (uv.x() < 0.0 || uv.x() >= kImageSize || uv.y() < 0.0 || uv.y() >= kImageSize) continue;
        frame.features.push_back(static_cast<uint32_t>(pixels.size()));
        frame.points.push_back(map::MapPointId{first_id + static_cast<uint32_t>(i)});
        pixels.push_back(uv);
        descriptors.push_back(test::detail::FlipBits(region[i].descriptor, kBitFlips, rng));
        angles.push_back(region[i].angle + kAngleNoise * noise(rng));
    }
    for (size_t e = 0; e < kExtras; ++e) {
        pixels.emplace_back(any_pixel(rng), any_pixel(rng));
        descriptors.push_back(test::detail::RandomDescriptor(rng));
        angles.push_back(any_angle(rng));
    }
    const auto n = static_cast<Eigen::Index>(pixels.size());
    geometry::TPoints2DF points(2, n);
    geometry::TRowVectorF feature_angles(n);
    test::TestImageFeatures::TDescriptors feature_descriptors(Extractor::DescriptorSize, n);
    for (Eigen::Index j = 0; j < n; ++j) {
        points.col(j) = pixels[static_cast<size_t>(j)];
        feature_angles[j] = angles[static_cast<size_t>(j)];
        feature_descriptors.col(j) = descriptors[static_cast<size_t>(j)];
    }
    const auto make_features = [&] {
        return test::TestImageFeatures(geometry::TPoints2DF(points), geometry::TRowVectorF::Ones(n),
                                       geometry::TRowVectorF(feature_angles),
                                       test::TestImageFeatures::TDescriptors(feature_descriptors),
                                       features::TLevels::Zero(n));
    };
    const double timestamp = 0.05 * static_cast<double>(id.value);
    if (binocular) {
        features::StereoRig rig{.K_left = K,
                                .K_right = K,
                                .T_rl = RigidTransformation(geometry::TMatrix33F::Identity(),
                                                            geometry::TVector3DF(-0.11, 0.0, 0.0)),
                                .baseline = 0.11};
        frame.measurement = features::TBinocularMeasurement<Extractor>(make_features(), make_features(), rig, timestamp);
    } else {
        frame.measurement = features::TMonocularMeasurement<Extractor>(make_features(), K, nullptr, timestamp);
    }
    return frame;
}

// New points of a transaction: ids first_id.., map positions, descriptors,
// normals from the first observer's centre. A map position is to_map(X) plus
// kMapNoise of Gaussian noise per axis (triangulation error, about 1 px at 6
// m).
struct NewPoints {
    uint32_t first_id = 0;
    std::vector<TPoint3DF> positions;
    std::vector<TDescriptor> descriptors;
    std::vector<TPoint3DF> normals;
};

NewPoints PointsOf(const std::vector<ScenePoint>& region, uint32_t first_id, const Sim3& to_map,
                   const geometry::TVector3DF& observer_centre, std::mt19937& rng) {
    std::normal_distribution<double> noise(0.0, kMapNoise);
    NewPoints points{first_id, {}, {}, {}};
    for (const ScenePoint& point : region) {
        const TPoint3DF X = geometry::Apply(to_map, point.X) + TPoint3DF(noise(rng), noise(rng), noise(rng));
        points.positions.push_back(X);
        points.descriptors.push_back(point.descriptor);
        points.normals.push_back((X - observer_centre).normalized());
    }
    return points;
}

// What the LoopCloser published.
struct Loop {
    map::MapId map_id;
    map::MapId query_map_id;
    map::MapId match_map_id;
    map::FrameId query_frame;
    map::FrameId match_frame;
    Sim3 S;
    std::vector<std::pair<map::MapPointId, map::MapPointId>> matched_points;
};

// A LoopCloser fed hand-built MapTransactions in place of the Mapper.
class Harness {
public:
    explicit Harness(std::shared_ptr<const TVocabulary> vocabulary, LoopCloserOptions options = {})
        : loop_closer_(options, std::move(vocabulary)) {
        mapper_.ConnectAsync(loop_closer_.inbox());
        tracker_.ConnectAsync(loop_closer_.inbox());
        loop_closer_.AddObserver(capture_);
    }
    // The inbox holds this harness's events and capture_ the LoopCloser's:
    // both are emptied before anything is destroyed.
    ~Harness() {
        while (loop_closer_.ProcessPending()) {
        }
        Take();
    }

    // Publishes `frame` (with its parent, the previous Frame) and `points`,
    // then runs the LoopCloser; returns what it published.
    std::vector<Loop> Publish(const SceneFrame& frame, const NewPoints& points = {}, bool new_map = false,
                              map::MapId map_id = map::MapId{1}) {
        auto* transaction = transaction_pool_.Create();
        transaction->map_id = map_id;
        transaction->new_map = new_map;
        if (new_map) transaction->start_frame = frame.id;
        events::NewFrame<Extractor>& added = transaction->added_frames.emplace_back();
        added.id = frame.id;
        added.T_cw = frame.T_cw;
        added.measurement = frame.measurement;
        added.observed_features = frame.features;
        added.observed_points = frame.points;
        if (!new_map) added.parent = previous_;
        const auto& left = std::visit(
            [](const auto& m) -> const features::TMonocularMeasurement<Extractor>& {
                if constexpr (requires { m.left(); }) return m.left();
                else return m;
            },
            frame.measurement);
        vocabulary_->Transform(left.descriptors(), mapping::Mapper<Extractor>::kBowLevelsUp, added.bow,
                               added.features);
        const auto count = static_cast<uint32_t>(points.positions.size());
        transaction->new_points_start_id = map::MapPointId{points.first_id};
        transaction->ResizeNewPoints(count);
        for (uint32_t i = 0; i < count; ++i) {
            transaction->new_positions.col(i) = points.positions[i];
            transaction->new_descriptors.col(i) = points.descriptors[i];
            transaction->new_normals.col(i) = points.normals[i];
        }
        previous_ = frame.id;
        mapper_.Notify(transaction);
        return Run();
    }

    std::vector<Loop> PublishTrackingLost(map::MapId map_id, bool final) {
        auto* lost = lost_pool_.Create();
        lost->map_id = map_id;
        lost->final = final;
        tracker_.Notify(lost);
        return Run();
    }

    std::vector<Loop> Run() {
        while (loop_closer_.ProcessPending()) {
        }
        return Take();
    }

    LoopCloser<Extractor>& loop_closer() { return loop_closer_; }

private:
    std::vector<Loop> Take() {
        std::vector<Loop> loops;
        while (events::Event* event = listener_.GetNextEvent()) {
            if (event->type != events::EventType::LoopDetected) continue;
            const auto& loop = *static_cast<events::LoopDetected*>(event);
            loops.push_back({loop.map_id, loop.query_map_id, loop.match_map_id, loop.query_frame,
                             loop.match_frame, loop.S_query_to_match, loop.matched_points});
        }
        return loops;
    }

    // Declaration order: the pools outlive the LoopCloser's inbox, the
    // LoopCloser outlives capture_.
    memory::ObjectPool<TTransaction, memory::ThreadSafePolicy> transaction_pool_{4};
    memory::ObjectPool<events::TrackingLost, memory::ThreadSafePolicy> lost_pool_{2};
    std::shared_ptr<const TVocabulary> vocabulary_ = MakeRegionVocabulary();
    events::Observable mapper_;
    events::Observable tracker_;
    LoopCloser<Extractor> loop_closer_;
    events::AsyncObserver capture_;
    events::Listener listener_ = capture_.GetListener();
    std::optional<map::FrameId> previous_;
};

// How the camera comes back to region A.
enum class Revisit {
    Drifted,    // through new points, drifted by `drift`: a loop
    Tracked,    // through region A's own points: covisible, not a loop
    LookAlike,  // through new points of a place that only looks like A
};

constexpr size_t kStartFrames = 5;   // E0..E4 map region A
constexpr size_t kMiddleFrames = 7;  // B0..B6 map region B
constexpr size_t kReturnFrames = 5;  // L0..L4 come back to region A

// The synthetic loop. Frames 0..4 (E) map region A (points 0..399) from the
// start; frames 5..11 (B) map region B (points 400..799) elsewhere; frames
// 12..16 (L) return to region A's place, 5 cm off the start poses, through
// `revisit`. A drifted revisit sees region A through new points 800..1199
// whose map positions are drift(X), from map poses consistent with them (scale
// folded into the translation).
class LoopScene {
public:
    LoopScene(uint32_t seed, Revisit revisit, const Sim3& drift, bool binocular)
        : rng_(seed), revisit_(revisit), drift_(drift), binocular_(binocular) {
        region_a_ = MakeRegion(kRegionPoints, StartPose(0), 0x00, rng_);
        region_b_ = MakeRegion(kRegionPoints, MiddlePose(0), 0xFF, rng_);
        revisited_ = revisit == Revisit::LookAlike ? LookAlike(region_a_, rng_) : region_a_;
    }

    static RigidTransformation StartPose(size_t k) {
        return Pose(0.1 * static_cast<double>(k), 0.0, 0.0, 0.5 * static_cast<double>(k));
    }
    static RigidTransformation MiddlePose(size_t k) {
        return Pose(100.0 + 0.1 * static_cast<double>(k), 0.0, 0.0, 0.5 * static_cast<double>(k));
    }
    // Half a metre off the start poses: the parallax that makes the scale of
    // a similarity observable through reprojections.
    static RigidTransformation ReturnPose(size_t k) {
        return Pose(0.4 + 0.1 * static_cast<double>(k), 0.1, 0.3, 2.0 + 0.5 * static_cast<double>(k));
    }
    static map::FrameId StartId(size_t k) { return map::FrameId{static_cast<uint32_t>(k)}; }
    static map::FrameId ReturnId(size_t k) {
        return map::FrameId{static_cast<uint32_t>(kStartFrames + kMiddleFrames + k)};
    }
    static constexpr uint32_t kRegionB = kRegionPoints;
    static constexpr uint32_t kRevisit = 2 * kRegionPoints;

    // Publishes E0..E4 and B0..B6; returns what the LoopCloser published.
    std::vector<Loop> PublishStartAndMiddle(Harness& harness) {
        std::vector<Loop> loops;
        const Sim3 identity;
        for (size_t k = 0; k < kStartFrames; ++k) {
            const SceneFrame frame = Observe(StartId(k), StartPose(k), StartPose(k), region_a_, 0, binocular_, rng_);
            const NewPoints points = k == 0 ? PointsOf(region_a_, 0, identity, Centre(StartPose(0)), rng_) : NewPoints{};
            Append(loops, harness.Publish(frame, points, k == 0));
        }
        for (size_t k = 0; k < kMiddleFrames; ++k) {
            const map::FrameId id{static_cast<uint32_t>(kStartFrames + k)};
            const SceneFrame frame = Observe(id, MiddlePose(k), MiddlePose(k), region_b_, kRegionB, binocular_, rng_);
            const NewPoints points =
                k == 0 ? PointsOf(region_b_, kRegionB, identity, Centre(MiddlePose(0)), rng_) : NewPoints{};
            Append(loops, harness.Publish(frame, points));
        }
        return loops;
    }

    // Publishes return frame L_k; returns what the LoopCloser published.
    std::vector<Loop> PublishReturn(Harness& harness, size_t k) {
        const RigidTransformation T_true = ReturnPose(k);
        if (revisit_ == Revisit::Tracked) {
            return harness.Publish(Observe(ReturnId(k), T_true, T_true, region_a_, 0, binocular_, rng_));
        }
        const SceneFrame frame = Observe(ReturnId(k), T_true, MapPose(T_true), revisited_, kRevisit, binocular_, rng_);
        const NewPoints points =
            k == 0 ? PointsOf(revisited_, kRevisit, drift_, Centre(MapPose(T_true)), rng_) : NewPoints{};
        return harness.Publish(frame, points);
    }

    // The true camera similarity of a drifted revisit, x_query = S x_match:
    // query map coordinates are drift(true) coordinates, scaled.
    [[nodiscard]] Sim3 TrueS(size_t return_k, map::FrameId match) const {
        const RigidTransformation T_qm = ReturnPose(return_k) * StartPose(match.value).Inverse();
        return {drift_.scale, T_qm.rotation(), drift_.scale * T_qm.translation()};
    }

    // Region A point index of a point id of either side, or -1.
    static int64_t ScenePointOf(map::MapPointId id) {
        if (id.value < kRegionB) return id.value;
        if (id.value >= kRevisit && id.value < kRevisit + kRegionPoints) return id.value - kRevisit;
        return -1;
    }

private:
    // drift ∘ (true world), with the camera scaled by the drift's scale:
    // T_map = s T_true ∘ drift⁻¹ as a rigid motion.
    [[nodiscard]] RigidTransformation MapPose(const RigidTransformation& T_true) const {
        const geometry::TMatrix33F R = T_true.rotation() * drift_.R.transpose();
        return {R, drift_.scale * T_true.translation() - R * drift_.t};
    }

    static void Append(std::vector<Loop>& out, std::vector<Loop> loops) {
        for (Loop& loop : loops) out.push_back(std::move(loop));
    }

    std::mt19937 rng_;
    Revisit revisit_;
    Sim3 drift_;
    bool binocular_;
    std::vector<ScenePoint> region_a_;
    std::vector<ScenePoint> region_b_;
    std::vector<ScenePoint> revisited_;
};

// Mono drift: 10 % scale, 4 deg of rotation, 0.5 m of translation.
Sim3 MonoDrift() {
    const geometry::TVector3DF axis = geometry::TVector3DF(0.2, 1.0, -0.1).normalized();
    return {1.1, Eigen::AngleAxis<double>(4.0 * std::numbers::pi / 180.0, axis).toRotationMatrix(),
            geometry::TVector3DF(0.4, -0.1, 0.3)};
}

Sim3 StereoDrift() {
    Sim3 drift = MonoDrift();
    drift.scale = 1.0;
    return drift;
}

double RotationErrorDeg(const geometry::TMatrix33F& a, const geometry::TMatrix33F& b) {
    return Eigen::AngleAxis<double>(a.transpose() * b).angle() * 180.0 / std::numbers::pi;
}

constexpr uint32_t kSeeds = 5;  // seeds 1..5 of every scene

// Drives a drifted revisit: no loop on L0..L2, LoopDetected on L3 (its third
// confirmation), close to the true similarity. Records the gate counts of
// DetectCommonRegion on L0.
void ExpectLoopAtThirdConfirmation(uint32_t seed, const Sim3& drift, bool binocular) {
    SCOPED_TRACE("seed " + std::to_string(seed));
    Harness harness(MakeRegionVocabulary());
    LoopScene scene(seed, Revisit::Drifted, drift, binocular);
    ASSERT_TRUE(scene.PublishStartAndMiddle(harness).empty());

    const LoopCloser<Extractor>& loop_closer = harness.loop_closer();
    for (size_t k = 0; k < 3; ++k) {
        EXPECT_TRUE(scene.PublishReturn(harness, k).empty()) << "L" << k;
        EXPECT_EQ(loop_closer.num_coincidences(), k) << "L" << k;
        if (k == 0) {
            // Gates of the first detection, with margins over the thresholds.
            const CommonRegionResult result =
                DetectCommonRegion(loop_closer.replica(), LoopScene::ReturnId(0), LoopCloserOptions{});

            const CommonRegionStats& stats = result.stats;
            ::testing::Test::RecordProperty("seed" + std::to_string(seed) + (binocular ? "_stereo" : "_mono"),
                                            "bow " + std::to_string(stats.bow_matches) + " ransac " +
                                                std::to_string(stats.ransac_inliers) + " optimized " +
                                                std::to_string(stats.optimized_inliers) + " projection " +
                                                std::to_string(stats.projection_matches));
            ASSERT_TRUE(result.region.has_value());
            EXPECT_GE(stats.bow_matches, 2 * 20u);
            EXPECT_GE(stats.ransac_inliers, 2 * 20u);
            EXPECT_GE(stats.optimized_inliers, 2 * 15u);
            EXPECT_GE(stats.projection_matches, 2 * 100u);
        }
    }

    const std::vector<Loop> loops = scene.PublishReturn(harness, 3);
    ASSERT_EQ(loops.size(), 1u);
    const Loop& loop = loops[0];
    EXPECT_EQ(loop.map_id, map::MapId{1});
    EXPECT_EQ(loop.query_map_id, map::MapId{1});
    EXPECT_EQ(loop.match_map_id, map::MapId{1});
    EXPECT_EQ(loop.query_frame, LoopScene::ReturnId(3));
    ASSERT_LT(loop.match_frame.value, kStartFrames);
    EXPECT_EQ(loop_closer.num_coincidences(), 0u);  // dropped once published

    const Sim3 truth = scene.TrueS(3, loop.match_frame);
    // Seeds 1..5 measured at most 0.09 deg, 1 % of scale (of a 10 % drift)
    // and 1.4 cm (of 0.4..0.9 m): bounds 3x over.
    EXPECT_LT(RotationErrorDeg(loop.S.R, truth.R), 0.3);
    if (binocular) {
        EXPECT_EQ(loop.S.scale, 1.0);
    } else {
        EXPECT_NEAR(loop.S.scale / truth.scale, 1.0, 0.03);
    }
    EXPECT_LT((loop.S.t - truth.t).norm(), 0.05);

    // The pairs: the query's points matched to the same scene points.
    EXPECT_GE(loop.matched_points.size(), 100u);
    size_t correct = 0;
    for (const auto& [query_point, match_point] : loop.matched_points) {
        EXPECT_GE(query_point.value, LoopScene::kRevisit);
        EXPECT_LT(match_point.value, LoopScene::kRegionB);
        const int64_t scene_point = LoopScene::ScenePointOf(query_point);
        correct += scene_point >= 0 && scene_point == LoopScene::ScenePointOf(match_point) ? 1 : 0;
    }
    EXPECT_GE(static_cast<double>(correct), 0.95 * static_cast<double>(loop.matched_points.size()));
}

TEST(LoopCloserTest, DetectsLoopAfterThreeConfirmations) {
    for (uint32_t seed = 1; seed <= kSeeds; ++seed) {
        ExpectLoopAtThirdConfirmation(seed, MonoDrift(), /*binocular=*/false);
    }
}

TEST(LoopCloserTest, StereoLoopKeepsUnitScale) {
    for (uint32_t seed = 1; seed <= kSeeds; ++seed) {
        ExpectLoopAtThirdConfirmation(seed, StereoDrift(), /*binocular=*/true);
    }
}

// Review Focus 4: coming back to region A through its own (tracked) points
// makes the return Frames covisible with every Frame of the place: no
// candidate survives, and no loop is published.
TEST(LoopCloserTest, CovisibleRevisitIsNotALoop) {
    for (uint32_t seed = 1; seed <= kSeeds; ++seed) {
        SCOPED_TRACE("seed " + std::to_string(seed));
        Harness harness(MakeRegionVocabulary());
        LoopScene scene(seed, Revisit::Tracked, Sim3{}, false);
        ASSERT_TRUE(scene.PublishStartAndMiddle(harness).empty());
        for (size_t k = 0; k < kReturnFrames; ++k) {
            EXPECT_TRUE(scene.PublishReturn(harness, k).empty()) << "L" << k;
            EXPECT_EQ(harness.loop_closer().num_coincidences(), 0u) << "L" << k;
            // Region B's Frames are the only candidates, and do not match.
            EXPECT_EQ(harness.loop_closer().last_stats().bow_matches, 0u) << "L" << k;
        }
    }
}

// A place that looks like region A (the same descriptors) with another 3-D
// layout: enough SearchByBoW matches, no consistent similarity.
TEST(LoopCloserTest, UnmappedLookAlikeRejected) {
    for (uint32_t seed = 1; seed <= kSeeds; ++seed) {
        SCOPED_TRACE("seed " + std::to_string(seed));
        Harness harness(MakeRegionVocabulary());
        LoopScene scene(seed, Revisit::LookAlike, MonoDrift(), false);
        ASSERT_TRUE(scene.PublishStartAndMiddle(harness).empty());
        for (size_t k = 0; k < kReturnFrames; ++k) {
            EXPECT_TRUE(scene.PublishReturn(harness, k).empty()) << "L" << k;
            const CommonRegionStats& stats = harness.loop_closer().last_stats();
            EXPECT_GE(stats.bow_matches, 2 * 20u) << "L" << k;
            EXPECT_EQ(stats.projection_matches, 0u) << "L" << k;
            EXPECT_EQ(harness.loop_closer().num_coincidences(), 0u) << "L" << k;
        }
    }
}

// Review Focus 3: a new map, or a final TrackingLost of the current one,
// drops the replica and the pending region.
TEST(LoopCloserTest, ClearsOnNewMap) {
    for (const bool by_tracking_lost : {false, true}) {
        SCOPED_TRACE(by_tracking_lost ? "final TrackingLost" : "new map");
        Harness harness(MakeRegionVocabulary());
        LoopScene scene(1, Revisit::Drifted, MonoDrift(), false);
        ASSERT_TRUE(scene.PublishStartAndMiddle(harness).empty());
        for (size_t k = 0; k < 3; ++k) {
            ASSERT_TRUE(scene.PublishReturn(harness, k).empty());
        }
        ASSERT_EQ(harness.loop_closer().num_coincidences(), 2u);

        std::mt19937 rng(7);
        const auto region = MakeRegion(50, Pose(0.0, 0.0, 0.0, 0.0), 0x00, rng);
        const SceneFrame first = Observe(map::FrameId{100}, Pose(0, 0, 0, 0), Pose(0, 0, 0, 0), region, 0, false, rng);
        if (by_tracking_lost) {
            // A non-final loss keeps everything.
            EXPECT_TRUE(harness.PublishTrackingLost(map::MapId{1}, false).empty());
            EXPECT_EQ(harness.loop_closer().num_coincidences(), 2u);
            EXPECT_EQ(harness.loop_closer().replica().num_keyframes(), 15u);
            EXPECT_TRUE(harness.PublishTrackingLost(map::MapId{1}, true).empty());
            EXPECT_EQ(harness.loop_closer().replica().num_keyframes(), 0u);
            EXPECT_EQ(harness.loop_closer().num_coincidences(), 0u);
            // A late transaction of the lost map is ignored.
            EXPECT_TRUE(scene.PublishReturn(harness, 3).empty());
            EXPECT_EQ(harness.loop_closer().replica().num_keyframes(), 0u);
            EXPECT_EQ(harness.loop_closer().num_coincidences(), 0u);
        } else {
            EXPECT_TRUE(harness.Publish(first, PointsOf(region, 0, Sim3{}, Centre(Pose(0, 0, 0, 0)), rng), true,
                                        map::MapId{2})
                            .empty());
            EXPECT_EQ(harness.loop_closer().replica().map_id(), map::MapId{2});
            EXPECT_EQ(harness.loop_closer().replica().num_keyframes(), 1u);
            EXPECT_EQ(harness.loop_closer().num_coincidences(), 0u);
        }
    }
}

// Without a vocabulary the LoopCloser only acknowledges: no replica, no loop.
TEST(LoopCloserTest, NoVocabularyDoesNothing) {
    Harness harness(nullptr);
    LoopScene scene(1, Revisit::Drifted, MonoDrift(), false);
    EXPECT_TRUE(scene.PublishStartAndMiddle(harness).empty());
    for (size_t k = 0; k < kReturnFrames; ++k) {
        EXPECT_TRUE(scene.PublishReturn(harness, k).empty()) << "L" << k;
    }
    EXPECT_EQ(harness.loop_closer().replica().num_keyframes(), 0u);
    EXPECT_EQ(harness.loop_closer().num_coincidences(), 0u);
}

TEST(LoopCloserOptionsTest, SpecDefaults) {
    const LoopCloserOptions options;
    EXPECT_EQ(options.min_map_frames, 12u);
    EXPECT_EQ(options.num_candidates, 3u);
    EXPECT_EQ(options.window, 5u);
    EXPECT_EQ(options.min_bow_matches, 20u);
    EXPECT_EQ(options.min_sim3_inliers, 20u);
    EXPECT_EQ(options.min_optimized, 15u);
    EXPECT_EQ(options.min_projection, 100u);
    EXPECT_EQ(options.confirmation_projection, 50u);
    EXPECT_EQ(options.required_coincidences, 3u);
    EXPECT_EQ(options.max_unconfirmed_queries, 3u);
    EXPECT_EQ(options.ransac_iterations, 300);
    EXPECT_DOUBLE_EQ(options.ransac_confidence, 0.99);
    EXPECT_DOUBLE_EQ(options.ransac_chi2, 9.21);
}

} // namespace
} // namespace voyahslam::loop_closing
