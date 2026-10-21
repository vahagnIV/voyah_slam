#include <gtest/gtest.h>
#include <support/synthetic_sequence.h>
#include <support/test_measurements.h>
#include <test_access.h>
#include <voyahslam/bow/vocabulary.h>
#include <voyahslam/events/async_observer.h>
#include <voyahslam/events/events.h>
#include <voyahslam/events/observable.h>
#include <voyahslam/geometry/sim3.h>
#include <voyahslam/loop_closing/loop_closer.h>
#include <voyahslam/mapping/mapper.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <numbers>
#include <optional>
#include <random>
#include <set>
#include <string>
#include <thread>
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

// Frame `id` at true pose `T_true` (map pose `T_map`) observing every point i
// of `region` in its image as point ids[i] (Observe: first_id + i), plus
// kExtras random features.
// Binocular frames carry the left image twice (detection reads the left one).
SceneFrame ObserveIds(map::FrameId id, const RigidTransformation& T_true, const RigidTransformation& T_map,
                      const std::vector<ScenePoint>& region, const std::vector<uint32_t>& ids, bool binocular,
                      std::mt19937& rng) {
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
        frame.points.push_back(map::MapPointId{ids[i]});
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

SceneFrame Observe(map::FrameId id, const RigidTransformation& T_true, const RigidTransformation& T_map,
                   const std::vector<ScenePoint>& region, uint32_t first_id, bool binocular, std::mt19937& rng) {
    std::vector<uint32_t> ids(region.size());
    for (size_t i = 0; i < ids.size(); ++i) ids[i] = first_id + static_cast<uint32_t>(i);
    return ObserveIds(id, T_true, T_map, region, ids, binocular, rng);
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
    bool priority = false;
};

// The default options with loop closing enabled (it is opt-in).
LoopCloserOptions EnabledOptions() {
    LoopCloserOptions options;
    options.enabled = true;
    return options;
}

// A LoopCloser fed hand-built MapTransactions in place of the Mapper;
// enabled unless `options` says otherwise.
class Harness {
public:
    // `pool_size`: the MapTransactions this harness may have alive at once.
    explicit Harness(std::shared_ptr<const TVocabulary> vocabulary, LoopCloserOptions options = EnabledOptions(),
                     size_t pool_size = 4)
        : pool_size_(pool_size), loop_closer_(options, std::move(vocabulary)) {
        subscriptions_.push_back(mapper_.ConnectAsync(loop_closer_.inbox()));
        subscriptions_.push_back(tracker_.ConnectAsync(loop_closer_.inbox()));
        subscriptions_.push_back(loop_closer_.AddObserver(capture_));
    }
    // The inbox holds this harness's events and capture_ the LoopCloser's:
    // both are emptied before anything is destroyed.
    ~Harness() {
        while (loop_closing::LoopCloserTestAccess::ProcessPending(loop_closer_)) {
        }
        Take();
    }

    // Publishes `frame` (with its parent, the previous Frame) and `points`,
    // then runs the LoopCloser; returns what it published. The map is map(),
    // unless `map_id` is given.
    std::vector<Loop> Publish(const SceneFrame& frame, const NewPoints& points = {}, bool new_map = false,
                              std::optional<map::MapId> map_id = std::nullopt) {
        EXPECT_TRUE(Send(frame, points, new_map, map_id));
        return Run();
    }

    // Publishes what Publish() does without running the LoopCloser. False
    // (counted in num_exhausted()) if pool_size transactions it sent are
    // still alive (the LoopCloser has not released them). The
    // active map is the transaction's, unless `active_map` is given (the
    // Mapper never names another).
    bool Send(const SceneFrame& frame, const NewPoints& points = {}, bool new_map = false,
              std::optional<map::MapId> map_id = std::nullopt, std::optional<map::MapId> active_map = std::nullopt) {
        auto transaction = TryNewTransaction();
        if (transaction == nullptr) {
            ++num_exhausted_;
            return false;
        }
        transaction->map_id = map_id.value_or(map_);
        transaction->map_epoch = map_epoch_;
        transaction->active_map = active_map.value_or(transaction->map_id);  // as the Mapper: the map it builds
        transaction->new_map = new_map;
        if (new_map) transaction->start_frame = frame.id;
        events::NewFrame<Extractor>& added = transaction->added_frames.emplace_back();
        added.id = frame.id;
        added.T_cw = frame.T_cw;
        added.measurement = test::Share(frame.measurement);
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
        mapper_.Notify(std::move(transaction));
        return true;
    }

    // Publishes a transaction of map 1 culling Frame `frame` (a leaf of the
    // spanning tree) without running the LoopCloser.
    void SendCull(map::FrameId frame) {
        auto transaction = NewTransaction();
        transaction->map_id = map::MapId{1};
        transaction->map_epoch = map_epoch_;
        transaction->active_map = map::MapId{1};
        transaction->removed_frames.push_back(frame);
        mapper_.Notify(std::move(transaction));
    }

    // Publishes an empty transaction of map 1 at `map_epoch` (a loop
    // correction's epoch; later transactions carry it too), then runs the
    // LoopCloser; returns what it published.
    std::vector<Loop> PublishEpoch(uint32_t map_epoch) {
        map_epoch_ = map_epoch;
        auto transaction = NewTransaction();
        transaction->map_id = map::MapId{1};
        transaction->map_epoch = map_epoch;
        transaction->active_map = map::MapId{1};
        mapper_.Notify(std::move(transaction));
        return Run();
    }

    // Publishes the Mapper's answer to a final loss of map `map_id` (removed,
    // none active), then runs the LoopCloser; returns what it published.
    std::vector<Loop> PublishRemoval(map::MapId map_id) {
        auto transaction = NewTransaction();
        transaction->map_id = map_id;
        transaction->map_epoch = map_epoch_;
        transaction->removed_maps.push_back(map_id);
        mapper_.Notify(std::move(transaction));
        return Run();
    }

    // Publishes an empty transaction of map `map_id` at `map_epoch` with
    // `active_map` (none: the Mapper keeping a lost map inactive) and
    // `removed_maps`, then runs the LoopCloser; returns what it published.
    std::vector<Loop> PublishEmpty(map::MapId map_id, std::optional<map::MapId> active_map, uint32_t map_epoch = 0,
                                   std::vector<map::MapId> removed_maps = {}) {
        auto transaction = NewTransaction();
        transaction->map_id = map_id;
        transaction->map_epoch = map_epoch;
        transaction->active_map = active_map;
        transaction->removed_maps = std::move(removed_maps);
        mapper_.Notify(std::move(transaction));
        return Run();
    }

    std::vector<Loop> PublishTrackingLost(map::MapId map_id, bool final) {
        auto lost = std::make_shared<events::TrackingLost>();
        lost->map_id = map_id;
        lost->final = final;
        tracker_.Notify(std::move(lost));
        return Run();
    }

    // Runs the LoopCloser; returns what it published, or nothing while held.
    std::vector<Loop> Run() {
        while (loop_closing::LoopCloserTestAccess::ProcessPending(loop_closer_)) {
        }
        if (hold_) {
            // The consumer keeps what was published, unacknowledged (legal: holding costs memory only).
            while (events::EventPtr event = listener_.GetNextEvent()) {
                held_.push_back(std::move(event));
            }
            return {};
        }
        return Take();
    }

    // While held, the consumer keeps every published event until Take().
    void set_hold(bool hold) { hold_ = hold; }
    // The map Publish() and Send() name by default (1).
    void set_map(map::MapId map_id) { map_ = map_id; }

    LoopCloser<Extractor>& loop_closer() { return loop_closer_; }
    [[nodiscard]] size_t num_exhausted() const { return num_exhausted_; }

    // Takes (and acknowledges) everything published, held events first.
    std::vector<Loop> Take() {
        std::vector<Loop> loops;
        const auto collect = [&loops](const events::Event& event) {
            if (event.type != events::EventType::LoopDetected) return;
            const auto& loop = static_cast<const events::LoopDetected&>(event);
            loops.push_back({loop.map_id, loop.query_map_id, loop.match_map_id, loop.query_frame,
                             loop.match_frame, loop.S_query_match, loop.matched_points, loop.priority});
        };
        for (const events::EventPtr& event : std::exchange(held_, {})) collect(*event);
        while (events::EventPtr event = listener_.GetNextEvent()) collect(*event);
        return loops;
    }

    // A new MapTransaction counted against pool_size while it is alive.
    std::shared_ptr<TTransaction> NewTransaction() {
        auto transaction = std::make_shared<TTransaction>();
        in_flight_.push_back(transaction);
        return transaction;
    }

    // NewTransaction(), or null when pool_size transactions sent are still alive.
    std::shared_ptr<TTransaction> TryNewTransaction() {
        std::erase_if(in_flight_, [](const std::weak_ptr<TTransaction>& weak) { return weak.expired(); });
        return in_flight_.size() < pool_size_ ? NewTransaction() : nullptr;
    }

    // Declaration order: the subscriptions are declared before the producers
    // and the LoopCloser they connect; the LoopCloser outlives capture_.
    std::vector<events::Subscription> subscriptions_;
    size_t pool_size_;  // the transactions this harness may have alive at once
    std::vector<std::weak_ptr<TTransaction>> in_flight_;  // every transaction sent; alive until the LoopCloser drops it
    std::vector<events::EventPtr> held_;  // published events the consumer keeps while set_hold(true)
    std::shared_ptr<const TVocabulary> vocabulary_ = MakeRegionVocabulary();
    events::Observable mapper_;
    events::Observable tracker_;
    LoopCloser<Extractor> loop_closer_;
    events::AsyncObserver capture_;
    events::Listener listener_ = capture_.GetListener();
    std::optional<map::FrameId> previous_;
    bool hold_ = false;
    map::MapId map_{1};
    uint32_t map_epoch_ = 0;
    size_t num_exhausted_ = 0;
};

// How the camera comes back to region A.
enum class Revisit {
    Drifted,    // through new points, drifted by `drift`: a loop
    Tracked,    // through region A's own points: covisible, not a loop
    LookAlike,  // through new points of a place that only looks like A
    // Through new points of region A turned 180 deg about a vertical axis
    // beyond it (Opposite()), seen from return poses turned alike: every image
    // is that of a true revisit, so every ORB-SLAM3 gate passes, but the place
    // faces the other way (the corridor seen from its other end).
    Opposite,
};

// The rigid turn of a Revisit::Opposite scene: 180 deg about the vertical
// axis through (0, 0, 12), beyond region A (4..8 m in front of the start).
RigidTransformation Opposite() {
    const geometry::TMatrix33F R =
        Eigen::AngleAxis<double>(std::numbers::pi, geometry::TVector3DF::UnitY()).toRotationMatrix();
    const geometry::TVector3DF pivot(0.0, 0.0, 12.0);
    return {R, pivot - R * pivot};
}

constexpr size_t kStartFrames = 5;   // E0..E4 map region A
constexpr size_t kMiddleFrames = 7;  // B0..B6 map region B
constexpr size_t kReturnFrames = 5;  // L0..L4 come back to region A

// The synthetic loop. Frames 0..4 (E) map region A (points 0..399) from the
// start; frames 5..11 (B) map region B (points 400..799) elsewhere; frames
// 12..16 (L) return to region A's place, 0.4..0.8 m off the start poses, through
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
        if (revisit == Revisit::Opposite) {
            for (ScenePoint& point : revisited_) point.X = Opposite().Transform(point.X);
        }
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
    static constexpr uint32_t kElsewhere = 3 * kRegionPoints;

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
        const RigidTransformation T_true =
            revisit_ == Revisit::Opposite ? ReturnPose(k) * Opposite().Inverse() : ReturnPose(k);
        if (revisit_ == Revisit::Tracked) {
            return harness.Publish(Observe(ReturnId(k), T_true, T_true, region_a_, 0, binocular_, rng_));
        }
        const SceneFrame frame = Observe(ReturnId(k), T_true, MapPose(T_true), revisited_, kRevisit, binocular_, rng_);
        const NewPoints points =
            k == 0 ? PointsOf(revisited_, kRevisit, drift_, Centre(MapPose(T_true)), rng_) : NewPoints{};
        return harness.Publish(frame, points);
    }

    // Publishes Frame `id` at the first return pose observing region A
    // through its own points (even i) and the revisit's (odd i): a Frame
    // covisible with both the start Frames and the return Frames. After L0.
    std::vector<Loop> PublishBridge(Harness& harness, map::FrameId id) {
        std::vector<uint32_t> ids(region_a_.size());
        for (size_t i = 0; i < ids.size(); ++i) {
            ids[i] = static_cast<uint32_t>(i % 2 == 0 ? i : kRevisit + i);
        }
        const RigidTransformation T_true = ReturnPose(0);
        return harness.Publish(ObserveIds(id, T_true, MapPose(T_true), region_a_, ids, binocular_, rng_));
    }

    // Publishes Frame `id` looking at a region C far from everything (its
    // points are added with the first such Frame).
    std::vector<Loop> PublishElsewhere(Harness& harness, map::FrameId id) {
        const RigidTransformation T = Pose(200.0, 0.0, 0.0, 0.0);
        NewPoints points;
        if (region_c_.empty()) {
            region_c_ = MakeRegion(kRegionPoints, T, 0xFF, rng_);
            points = PointsOf(region_c_, kElsewhere, Sim3{}, Centre(T), rng_);
        }
        return harness.Publish(Observe(id, T, T, region_c_, kElsewhere, binocular_, rng_), points);
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
    std::vector<ScenePoint> region_c_;
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
        EXPECT_EQ(LoopCloserTestAccess::NumCoincidences(loop_closer), k) << "L" << k;
        if (k == 0) {
            // Gates of the first detection, with margins over the thresholds.
            const CommonRegionResult result =
                DetectCommonRegion(LoopCloserTestAccess::MapReplica(loop_closer), LoopScene::ReturnId(0), LoopCloserOptions{});

            const CommonRegionStats& stats = result.stats;
            ::testing::Test::RecordProperty(
                "seed" + std::to_string(seed) + (binocular ? "_stereo" : "_mono"),
                "bow " + std::to_string(stats.bow_matches) + " ransac " + std::to_string(stats.ransac_inliers) +
                    " coarse " + std::to_string(stats.coarse_projection_matches) + " optimized " +
                    std::to_string(stats.optimized_inliers) + " projection " +
                    std::to_string(stats.projection_matches));
            ASSERT_TRUE(result.region.has_value());
            const LoopCloserOptions options;
            EXPECT_GE(stats.bow_matches, 2 * options.min_bow_matches);
            EXPECT_GE(stats.ransac_inliers, 2 * options.min_sim3_inliers);
            EXPECT_GE(stats.coarse_projection_matches, 2 * options.min_coarse_projection);
            EXPECT_GE(stats.optimized_inliers, 2 * options.min_optimized);
            EXPECT_GE(stats.projection_matches, 2 * options.min_projection);
            // The normal gate measured the final pairs of the true loop and
            // found them consistent.
            EXPECT_GE(result.region->normals.pairs, options.min_projection);
            EXPECT_LE(result.region->normals.disagreement, 0.05);
            EXPECT_TRUE(stats.normal_rejections.empty());
        }
    }

    const std::vector<Loop> loops = scene.PublishReturn(harness, 3);
    const QueryStats& queries = LoopCloserTestAccess::Queries(loop_closer);
    ::testing::Test::RecordProperty(
        "seed" + std::to_string(seed) + (binocular ? "_stereo" : "_mono") + "_detection_ms",
        std::to_string(queries.detected) + " queries, mean " +
            std::to_string(1e3 * queries.total_seconds / static_cast<double>(std::max<size_t>(queries.detected, 1))) +
            " max " + std::to_string(1e3 * queries.max_seconds));
    ASSERT_EQ(loops.size(), 1u);
    const Loop& loop = loops[0];
    EXPECT_TRUE(loop.priority);  // the first of its region
    EXPECT_EQ(loop.map_id, map::MapId{1});
    EXPECT_EQ(loop.query_map_id, map::MapId{1});
    EXPECT_EQ(loop.match_map_id, map::MapId{1});
    EXPECT_EQ(loop.query_frame, LoopScene::ReturnId(3));
    ASSERT_LT(loop.match_frame.value, kStartFrames);
    EXPECT_EQ(LoopCloserTestAccess::NumCoincidences(loop_closer), 0u);  // dropped once published

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

bool IsStartFrame(map::FrameId frame) { return frame.value < kStartFrames; }

// Review Focus 4, the database filter: coming back to region A through its
// own (tracked) points makes the return Frames covisible with every Frame of
// the place, so no start Frame is a candidate (region B's Frames are, and do
// not match), and no loop is published.
TEST(LoopCloserTest, CovisibleRevisitIsNotALoop) {
    for (uint32_t seed = 1; seed <= kSeeds; ++seed) {
        SCOPED_TRACE("seed " + std::to_string(seed));
        Harness harness(MakeRegionVocabulary());
        LoopScene scene(seed, Revisit::Tracked, Sim3{}, false);
        ASSERT_TRUE(scene.PublishStartAndMiddle(harness).empty());
        for (size_t k = 0; k < kReturnFrames; ++k) {
            EXPECT_TRUE(scene.PublishReturn(harness, k).empty()) << "L" << k;
            EXPECT_EQ(LoopCloserTestAccess::NumCoincidences(harness.loop_closer()), 0u) << "L" << k;
            const CommonRegionStats& stats = LoopCloserTestAccess::LastStats(harness.loop_closer());
            EXPECT_FALSE(stats.candidates.empty()) << "L" << k;
            EXPECT_TRUE(std::none_of(stats.candidates.begin(), stats.candidates.end(), IsStartFrame)) << "L" << k;
            EXPECT_EQ(stats.bow_matches, 0u) << "L" << k;
        }
    }
}

// Review Focus 4, the window guard (bAbortByNearKF): a bridge Frame observes
// region A through both its own points and the revisit's, so the start
// Frames are not covisible with the return Frames but their windows hold a
// Frame that is. The start Frames are candidates, and none is matched.
TEST(LoopCloserTest, CandidateWindowTouchingQueryIsNotALoop) {
    for (uint32_t seed = 1; seed <= kSeeds; ++seed) {
        SCOPED_TRACE("seed " + std::to_string(seed));
        Harness harness(MakeRegionVocabulary());
        LoopScene scene(seed, Revisit::Drifted, MonoDrift(), false);
        ASSERT_TRUE(scene.PublishStartAndMiddle(harness).empty());
        ASSERT_TRUE(scene.PublishReturn(harness, 0).empty());
        ASSERT_TRUE(scene.PublishBridge(harness, map::FrameId{100}).empty());
        for (size_t k = 1; k < kReturnFrames; ++k) {
            EXPECT_TRUE(scene.PublishReturn(harness, k).empty()) << "L" << k;
            EXPECT_EQ(LoopCloserTestAccess::NumCoincidences(harness.loop_closer()), 0u) << "L" << k;
            const CommonRegionStats& stats = LoopCloserTestAccess::LastStats(harness.loop_closer());
            EXPECT_TRUE(std::any_of(stats.candidates.begin(), stats.candidates.end(), IsStartFrame)) << "L" << k;
            EXPECT_EQ(stats.bow_matches, 0u) << "L" << k;
        }
    }
}

// mnLoopNumNotFound: a pending region that max_unconfirmed_queries (2) later
// queries in a row fail to confirm is dropped.
TEST(LoopCloserTest, FailedConfirmationsDropPendingRegion) {
    Harness harness(MakeRegionVocabulary());
    LoopScene scene(1, Revisit::Drifted, MonoDrift(), false);
    ASSERT_TRUE(scene.PublishStartAndMiddle(harness).empty());
    ASSERT_TRUE(scene.PublishReturn(harness, 0).empty());
    ASSERT_TRUE(scene.PublishReturn(harness, 1).empty());
    ASSERT_EQ(LoopCloserTestAccess::NumCoincidences(harness.loop_closer()), 1u);

    EXPECT_TRUE(scene.PublishElsewhere(harness, map::FrameId{101}).empty());
    EXPECT_EQ(LoopCloserTestAccess::NumCoincidences(harness.loop_closer()), 1u);  // one failure: kept
    EXPECT_TRUE(scene.PublishElsewhere(harness, map::FrameId{102}).empty());
    EXPECT_EQ(LoopCloserTestAccess::NumCoincidences(harness.loop_closer()), 0u);  // two: dropped
}

// LoopDetected is never dropped. Until 2026-10-13 a loop was dropped when
// every event was in flight (an in-flight limit of 1, held by the consumer);
// events now own their memory (D-25), so a consumer that keeps the first
// LoopDetected only costs memory and the next detection is still published.
TEST(LoopCloserTest, DeliversLoopWhileEarlierLoopIsHeld) {
    Harness harness(MakeRegionVocabulary());
    LoopScene scene(1, Revisit::Drifted, MonoDrift(), false);
    ASSERT_TRUE(scene.PublishStartAndMiddle(harness).empty());
    harness.set_hold(true);
    for (size_t k = 0; k < kReturnFrames; ++k) {
        scene.PublishReturn(harness, k);
    }
    // L3 publishes the loop; L4 finds it again (3 covisible confirmations)
    // while the first event is still held.
    harness.set_hold(false);
    const std::vector<Loop> loops = harness.Take();
    ASSERT_GE(loops.size(), 2u);
    EXPECT_EQ(loops[0].query_frame, LoopScene::ReturnId(3));
}

// A loop found again before its correction reaches the LoopCloser's replica
// (the same epoch) is still published, for the Mapper to drop, but without
// priority: it does not stop a running local BA. After a correction (a new
// epoch) a loop is urgent again.
TEST(LoopCloserTest, RepeatedLoopIsNotUrgent) {
    Harness harness(MakeRegionVocabulary());
    LoopScene scene(1, Revisit::Drifted, MonoDrift(), false);
    ASSERT_TRUE(scene.PublishStartAndMiddle(harness).empty());
    for (size_t k = 0; k < 3; ++k) {
        ASSERT_TRUE(scene.PublishReturn(harness, k).empty());
    }
    std::vector<Loop> loops = scene.PublishReturn(harness, 3);
    ASSERT_EQ(loops.size(), 1u);
    EXPECT_TRUE(loops[0].priority);
    // L4 finds the region again with 3 covisible confirmations.
    loops = scene.PublishReturn(harness, 4);
    ASSERT_EQ(loops.size(), 1u);
    EXPECT_FALSE(loops[0].priority);

    EXPECT_TRUE(harness.PublishEpoch(1).empty());
    loops = scene.PublishReturn(harness, 5);
    ASSERT_EQ(loops.size(), 1u);
    EXPECT_TRUE(loops[0].priority);
}

// M1: a pending region's Sim3 is in pre-correction coordinates, so a
// transaction raising the map epoch drops it; one at the same epoch keeps it.
TEST(LoopCloserTest, CorrectionDropsPendingRegion) {
    Harness harness(MakeRegionVocabulary());
    LoopScene scene(1, Revisit::Drifted, MonoDrift(), false);
    ASSERT_TRUE(scene.PublishStartAndMiddle(harness).empty());
    ASSERT_TRUE(scene.PublishReturn(harness, 0).empty());
    ASSERT_TRUE(scene.PublishReturn(harness, 1).empty());
    ASSERT_EQ(LoopCloserTestAccess::NumCoincidences(harness.loop_closer()), 1u);
    EXPECT_TRUE(harness.PublishEpoch(0).empty());
    EXPECT_EQ(LoopCloserTestAccess::NumCoincidences(harness.loop_closer()), 1u);
    EXPECT_TRUE(harness.PublishEpoch(1).empty());
    EXPECT_EQ(LoopCloserTestAccess::NumCoincidences(harness.loop_closer()), 0u);
    EXPECT_EQ(LoopCloserTestAccess::MapReplica(harness.loop_closer()).map_epoch(), 1u);
}

// Frames of one place, each a query (min_map_frames 1): Frame 0 adds the
// place's points.
class QueryFeed {
public:
    explicit QueryFeed(Harness& harness) : harness_(harness), region_(MakeRegion(50, T_, 0x00, rng_)) {}

    bool Send() {
        const map::FrameId id{next_++};
        const NewPoints points = id.value == 0 ? PointsOf(region_, 0, Sim3{}, Centre(T_), rng_) : NewPoints{};
        return harness_.Send(Observe(id, T_, T_, region_, 0, false, rng_), points, id.value == 0);
    }
    [[nodiscard]] uint32_t num_sent() const { return next_; }

private:
    Harness& harness_;
    std::mt19937 rng_{1};
    RigidTransformation T_ = Pose(0.0, 0.0, 0.0, 0.0);
    std::vector<ScenePoint> region_;
    uint32_t next_ = 0;
};

LoopCloserOptions EveryFrameAQuery() {
    LoopCloserOptions options = EnabledOptions();
    options.min_map_frames = 1;
    return options;
}

// I1: a slow detector holds no MapTransaction. While each query is detected
// the Mapper publishes two more Frames, so detection falls further behind
// with every query; the transactions are applied (and released) on receipt
// and only their Frame ids wait, the oldest beyond max_query_backlog dropped.
// Before, each transaction was held until its Frames were detected and the
// Mapper's pool ran out.
TEST(LoopCloserTest, SlowDetectorHoldsNoTransaction) {
    constexpr uint32_t kFrames = 200;
    LoopCloserOptions options = EveryFrameAQuery();
    options.max_query_backlog = 4;
    size_t detected = 0;
    Harness harness(MakeRegionVocabulary(), options, /*pool_size=*/8);
    QueryFeed feed(harness);
    ASSERT_TRUE(feed.Send());
    harness.loop_closer().set_query_hook([&](map::FrameId) {
        ++detected;
        for (int i = 0; i < 2 && feed.num_sent() < kFrames; ++i) {
            feed.Send();
        }
    });
    harness.Run();
    harness.loop_closer().set_query_hook({});

    EXPECT_EQ(harness.num_exhausted(), 0u);
    EXPECT_EQ(feed.num_sent(), kFrames);
    EXPECT_EQ(LoopCloserTestAccess::MapReplica(harness.loop_closer()).num_keyframes(), kFrames);
    const QueryStats& queries = LoopCloserTestAccess::Queries(harness.loop_closer());
    EXPECT_EQ(queries.detected, detected);
    EXPECT_GT(queries.dropped, kFrames / 4);
    EXPECT_EQ(queries.detected + queries.dropped, kFrames);
}

// A queued query whose Frame is culled before its detection is skipped.
TEST(LoopCloserTest, CulledQueryIsSkipped) {
    std::vector<map::FrameId> detected;
    Harness harness(MakeRegionVocabulary(), EveryFrameAQuery());
    harness.loop_closer().set_query_hook([&](map::FrameId query) { detected.push_back(query); });
    QueryFeed feed(harness);
    ASSERT_TRUE(feed.Send());
    ASSERT_TRUE(feed.Send());
    harness.SendCull(map::FrameId{1});
    EXPECT_TRUE(harness.Run().empty());
    harness.loop_closer().set_query_hook({});

    EXPECT_EQ(detected, std::vector<map::FrameId>{map::FrameId{0}});
    EXPECT_EQ(LoopCloserTestAccess::MapReplica(harness.loop_closer()).keyframe(map::FrameId{1}), nullptr);
    EXPECT_EQ(LoopCloserTestAccess::Queries(harness.loop_closer()).detected, 1u);
}

// M3: Stop() returns after the query being detected, not after the backlog.
TEST(LoopCloserTest, StopDoesNotWaitForBacklog) {
    constexpr uint32_t kFrames = 8;
    std::atomic<size_t> detected{0};
    LoopCloserOptions options = EveryFrameAQuery();
    options.max_query_backlog = kFrames;
    Harness harness(MakeRegionVocabulary(), options, /*pool_size=*/kFrames);
    QueryFeed feed(harness);
    for (uint32_t i = 0; i < kFrames; ++i) {
        ASSERT_TRUE(feed.Send());
    }
    harness.loop_closer().set_query_hook([&](map::FrameId) {
        detected.fetch_add(1);
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    });
    harness.loop_closer().Start();
    while (detected.load() == 0) {
        std::this_thread::yield();
    }
    harness.loop_closer().Stop();
    EXPECT_LE(detected.load(), 2u);
    harness.loop_closer().set_query_hook({});
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
            const CommonRegionStats& stats = LoopCloserTestAccess::LastStats(harness.loop_closer());
            EXPECT_GE(stats.bow_matches, 2 * LoopCloserOptions{}.min_bow_matches) << "L" << k;
            EXPECT_EQ(stats.projection_matches, 0u) << "L" << k;
            EXPECT_EQ(LoopCloserTestAccess::NumCoincidences(harness.loop_closer()), 0u) << "L" << k;
        }
    }
}

// Review Focus 3: a new map, or the Mapper dropping the current one after a
// final TrackingLost, drops the replica and the pending region; a
// TrackingLost alone changes nothing.
TEST(LoopCloserTest, ClearsOnNewMap) {
    for (const bool by_tracking_lost : {false, true}) {
        SCOPED_TRACE(by_tracking_lost ? "final TrackingLost" : "new map");
        Harness harness(MakeRegionVocabulary());
        LoopScene scene(1, Revisit::Drifted, MonoDrift(), false);
        ASSERT_TRUE(scene.PublishStartAndMiddle(harness).empty());
        for (size_t k = 0; k < 3; ++k) {
            ASSERT_TRUE(scene.PublishReturn(harness, k).empty());
        }
        ASSERT_EQ(LoopCloserTestAccess::NumCoincidences(harness.loop_closer()), 2u);

        std::mt19937 rng(7);
        const auto region = MakeRegion(50, Pose(0.0, 0.0, 0.0, 0.0), 0x00, rng);
        const SceneFrame first = Observe(map::FrameId{100}, Pose(0, 0, 0, 0), Pose(0, 0, 0, 0), region, 0, false, rng);
        if (by_tracking_lost) {
            // A non-final loss keeps everything.
            EXPECT_TRUE(harness.PublishTrackingLost(map::MapId{1}, false).empty());
            EXPECT_EQ(LoopCloserTestAccess::NumCoincidences(harness.loop_closer()), 2u);
            EXPECT_EQ(LoopCloserTestAccess::MapReplica(harness.loop_closer()).num_keyframes(), 15u);
            EXPECT_TRUE(harness.PublishTrackingLost(map::MapId{1}, true).empty());
            EXPECT_EQ(LoopCloserTestAccess::MapReplica(harness.loop_closer()).num_keyframes(), 15u);
            EXPECT_TRUE(harness.PublishRemoval(map::MapId{1}).empty());
            EXPECT_EQ(LoopCloserTestAccess::MapReplica(harness.loop_closer()).num_keyframes(), 0u);
            EXPECT_EQ(LoopCloserTestAccess::NumCoincidences(harness.loop_closer()), 0u);
            // A late transaction of the lost map is ignored.
            EXPECT_TRUE(scene.PublishReturn(harness, 3).empty());
            EXPECT_EQ(LoopCloserTestAccess::MapReplica(harness.loop_closer()).num_keyframes(), 0u);
            EXPECT_EQ(LoopCloserTestAccess::NumCoincidences(harness.loop_closer()), 0u);
        } else {
            EXPECT_TRUE(harness.Publish(first, PointsOf(region, 0, Sim3{}, Centre(Pose(0, 0, 0, 0)), rng), true,
                                        map::MapId{2})
                            .empty());
            EXPECT_EQ(LoopCloserTestAccess::MapReplica(harness.loop_closer()).map_id(), map::MapId{2});
            EXPECT_EQ(LoopCloserTestAccess::MapReplica(harness.loop_closer()).num_keyframes(), 1u);
            EXPECT_EQ(LoopCloserTestAccess::NumCoincidences(harness.loop_closer()), 0u);
        }
    }
}

// A transaction naming an inactive live map (fed by hand: the Mapper never
// publishes one) leaves the LoopCloser idle: the map's replica follows it,
// but no query is detected, and its epoch does not drop the active map's
// pending region. Map 1 is kept inactive; the scene is map 2.
TEST(LoopCloserTest, InactiveMapLeavesLoopCloserIdle) {
    Harness harness(MakeRegionVocabulary());
    std::mt19937 rng(7);
    const auto region = MakeRegion(50, Pose(0.0, 0.0, 0.0, 0.0), 0x00, rng);
    const auto observe = [&](uint32_t id) {
        return Observe(map::FrameId{id}, Pose(0, 0, 0, 0), Pose(0, 0, 0, 0), region, 0, false, rng);
    };
    ASSERT_TRUE(harness.Publish(observe(200), PointsOf(region, 0, Sim3{}, Centre(Pose(0, 0, 0, 0)), rng), true)
                    .empty());
    ASSERT_TRUE(harness.PublishEmpty(map::MapId{1}, std::nullopt).empty());  // kept inactive
    const auto& atlas = LoopCloserTestAccess::Atlas(harness.loop_closer());
    ASSERT_EQ(atlas.map_ids(), std::vector<map::MapId>{map::MapId{1}});
    ASSERT_FALSE(atlas.active_map().has_value());

    harness.set_map(map::MapId{2});
    LoopScene scene(1, Revisit::Drifted, MonoDrift(), false);
    ASSERT_TRUE(scene.PublishStartAndMiddle(harness).empty());
    for (size_t k = 0; k < 3; ++k) {
        ASSERT_TRUE(scene.PublishReturn(harness, k).empty());
    }
    ASSERT_EQ(LoopCloserTestAccess::NumCoincidences(harness.loop_closer()), 2u);
    const size_t detected = LoopCloserTestAccess::Queries(harness.loop_closer()).detected;

    // Map 1, inactive, gains a Frame and a correction epoch.
    ASSERT_TRUE(harness.Send(observe(201), {}, false, map::MapId{1}, map::MapId{2}));
    EXPECT_TRUE(harness.Run().empty());
    EXPECT_TRUE(harness.PublishEmpty(map::MapId{1}, map::MapId{2}, 5).empty());

    EXPECT_EQ(atlas.map(map::MapId{1})->num_keyframes(), 2u);  // applied to its map
    EXPECT_EQ(atlas.map(map::MapId{1})->map_epoch(), 5u);
    EXPECT_EQ(atlas.active_map(), map::MapId{2});
    EXPECT_EQ(LoopCloserTestAccess::Queries(harness.loop_closer()).detected, detected);
    EXPECT_EQ(LoopCloserTestAccess::NumCoincidences(harness.loop_closer()), 2u);
    // Map 2's pending region goes on to its loop.
    const std::vector<Loop> loops = scene.PublishReturn(harness, 3);
    ASSERT_EQ(loops.size(), 1u);
    EXPECT_EQ(loops[0].map_id, map::MapId{2});
    EXPECT_EQ(loops[0].query_frame, LoopScene::ReturnId(3));
}

// Without a vocabulary the LoopCloser only acknowledges: no replica, no loop.
TEST(LoopCloserTest, NoVocabularyDoesNothing) {
    Harness harness(nullptr);
    LoopScene scene(1, Revisit::Drifted, MonoDrift(), false);
    EXPECT_TRUE(scene.PublishStartAndMiddle(harness).empty());
    for (size_t k = 0; k < kReturnFrames; ++k) {
        EXPECT_TRUE(scene.PublishReturn(harness, k).empty()) << "L" << k;
    }
    EXPECT_EQ(LoopCloserTestAccess::MapReplica(harness.loop_closer()).num_keyframes(), 0u);
    EXPECT_EQ(LoopCloserTestAccess::NumCoincidences(harness.loop_closer()), 0u);
}

LoopCloserOptions WithoutNormalGate() {
    LoopCloserOptions options = EnabledOptions();
    options.max_normal_disagreement = 1.0;
    return options;
}

// The normal gate (an addition to ORB-SLAM3): region A seen again from the
// opposite side through new points passes every ORB-SLAM3 gate (without the
// normal gate it is a loop at L3), but nearly all its pairs' normals are
// opposed, so with the gate no loop is published and each detection's
// candidate is rejected by it.
void ExpectOppositeRegionRejected(uint32_t seed, bool binocular) {
    SCOPED_TRACE("seed " + std::to_string(seed) + (binocular ? " stereo" : " mono"));
    {
        Harness harness(MakeRegionVocabulary(), WithoutNormalGate());
        LoopScene scene(seed, Revisit::Opposite, Sim3{}, binocular);
        ASSERT_TRUE(scene.PublishStartAndMiddle(harness).empty());
        std::vector<Loop> loops;
        for (size_t k = 0; k <= 3; ++k) {
            for (Loop& loop : scene.PublishReturn(harness, k)) loops.push_back(std::move(loop));
        }
        ASSERT_EQ(loops.size(), 1u) << "without the normal gate";
        EXPECT_LT(loops[0].match_frame.value, kStartFrames);
    }
    Harness harness(MakeRegionVocabulary());
    LoopScene scene(seed, Revisit::Opposite, Sim3{}, binocular);
    ASSERT_TRUE(scene.PublishStartAndMiddle(harness).empty());
    for (size_t k = 0; k < kReturnFrames; ++k) {
        EXPECT_TRUE(scene.PublishReturn(harness, k).empty()) << "L" << k;
        EXPECT_EQ(LoopCloserTestAccess::NumCoincidences(harness.loop_closer()), 0u) << "L" << k;
        const CommonRegionStats& stats = LoopCloserTestAccess::LastStats(harness.loop_closer());
        EXPECT_GE(stats.projection_matches, LoopCloserOptions{}.min_projection) << "L" << k;
        ASSERT_FALSE(stats.normal_rejections.empty()) << "L" << k;
        for (const NormalRejection& rejection : stats.normal_rejections) {
            EXPECT_EQ(rejection.frame, LoopScene::ReturnId(k));
            EXPECT_LT(rejection.match_frame.value, kStartFrames);
            EXPECT_GE(rejection.agreement.pairs, LoopCloserOptions{}.min_projection);
            EXPECT_GE(rejection.agreement.disagreement, 0.9);
        }
    }
}

TEST(LoopCloserTest, OppositeFacingRegionRejectedByNormals) {
    for (uint32_t seed = 1; seed <= kSeeds; ++seed) {
        ExpectOppositeRegionRejected(seed, /*binocular=*/false);
        ExpectOppositeRegionRejected(seed, /*binocular=*/true);
    }
}

// A later query's confirmation (RefineCommonRegion) passes the normal gate
// too: the opposite region found at L0 without the gate is not confirmed by
// L1 with it, but is without it.
TEST(LoopCloserTest, RefinementRejectedByNormals) {
    Harness harness(MakeRegionVocabulary(), WithoutNormalGate());
    LoopScene scene(1, Revisit::Opposite, Sim3{}, /*binocular=*/false);
    ASSERT_TRUE(scene.PublishStartAndMiddle(harness).empty());
    ASSERT_TRUE(scene.PublishReturn(harness, 0).empty());
    ASSERT_TRUE(scene.PublishReturn(harness, 1).empty());
    const auto& replica = LoopCloserTestAccess::MapReplica(harness.loop_closer());
    const CommonRegionResult found = DetectCommonRegion(replica, LoopScene::ReturnId(0), WithoutNormalGate());
    ASSERT_TRUE(found.region.has_value());
    // L1's guess: L0's similarity moved by their relative pose.
    const RigidTransformation T_10 =
        replica.keyframe(LoopScene::ReturnId(1))->T_cw * replica.keyframe(LoopScene::ReturnId(0))->T_cw.Inverse();
    const Sim3 S_1w = geometry::Compose(Sim3{1.0, T_10.rotation(), T_10.translation()}, found.region->S_query_world);

    EXPECT_TRUE(RefineCommonRegion(replica, LoopScene::ReturnId(1), replica, found.region->match_frame, S_1w,
                                   WithoutNormalGate())
                    .has_value());
    CommonRegionStats stats;
    EXPECT_FALSE(RefineCommonRegion(replica, LoopScene::ReturnId(1), replica, found.region->match_frame, S_1w,
                                    LoopCloserOptions{}, &stats)
                     .has_value());
    ASSERT_EQ(stats.normal_rejections.size(), 1u);
    EXPECT_EQ(stats.normal_rejections[0].frame, LoopScene::ReturnId(1));
    EXPECT_GE(stats.normal_rejections[0].agreement.disagreement, 0.9);
}

// Loop closing is opt-in (user decision 2026-10-10): with the default
// options and a vocabulary, the LoopCloser keeps no replica and publishes no
// loop on a scene where an enabled one does (DetectsLoopAfterThreeConfirmations).
TEST(LoopCloserTest, OffByDefault) {
    Harness harness(MakeRegionVocabulary(), LoopCloserOptions{});
    LoopScene scene(1, Revisit::Drifted, MonoDrift(), false);
    EXPECT_TRUE(scene.PublishStartAndMiddle(harness).empty());
    for (size_t k = 0; k < kReturnFrames; ++k) {
        EXPECT_TRUE(scene.PublishReturn(harness, k).empty()) << "L" << k;
    }
    EXPECT_EQ(LoopCloserTestAccess::MapReplica(harness.loop_closer()).num_keyframes(), 0u);
}

// Disabled (the default; the apps without --loop_closing), the LoopCloser
// does what it does without a vocabulary, which the Mapper and the Tracker
// keep.
TEST(LoopCloserTest, DisabledDoesNothing) {
    LoopCloserOptions options;
    options.enabled = false;
    Harness harness(MakeRegionVocabulary(), options);
    LoopScene scene(1, Revisit::Drifted, MonoDrift(), false);
    EXPECT_TRUE(scene.PublishStartAndMiddle(harness).empty());
    for (size_t k = 0; k < kReturnFrames; ++k) {
        EXPECT_TRUE(scene.PublishReturn(harness, k).empty()) << "L" << k;
    }
    EXPECT_EQ(LoopCloserTestAccess::MapReplica(harness.loop_closer()).num_keyframes(), 0u);
    EXPECT_EQ(LoopCloserTestAccess::NumCoincidences(harness.loop_closer()), 0u);
}

// ---- Map merging (ORB-SLAM3 NewDetectCommonRegions, merge part) ----

// What a merge-scene return Frame observes of the start's place.
enum class Sees {
    A,     // region A, mapped in map 1: a merge
    D,     // region D, mapped in map 2: a loop
    Both,  // both regions, one image
};

constexpr size_t kMergeStartFrames = 5;   // F0..F4 of map 2
constexpr size_t kMergeMiddleFrames = 7;  // G0..G6 of map 2
constexpr uint32_t kMap2FirstFrame = 20;

// The synthetic two-map scene (spec section 6, Detection).
// Map 1, built first and then kept inactive: Frames 0..4 (E) map region A
// (map 1's points 0..399) from the start poses, Frames 5..11 (B) map region B
// (400..799) elsewhere. Its coordinates are the true ones.
// Map 2, active: its coordinates are drift(true) (for a mono map the scale is
// unrelated to map 1's). Frames 20..24 (F) map region D (map 2's points
// 0..399: point ids restart per map), other points at the start's place;
// Frames 25..31 (G) map region G (400..799) elsewhere. Return Frames 32..
// come back to the start's place 0.4..0.8 m off (LoopScene::ReturnPose) and
// observe region A through new points 800..1199 (a merge with map 1) and/or
// region D through new points 1200..1599 (a loop in map 2).
// `map1_region_a` moves map 1's region A ids (0..399 by default; ids
// 0..map1_region_a-1 then stay unused, which the replica allows): at
// kReturnA every true merge pair shares its id across the two maps.
class MergeScene {
public:
    MergeScene(uint32_t seed, const Sim3& drift, bool binocular, uint32_t map1_region_a = 0)
        : rng_(seed), drift_(drift), binocular_(binocular), map1_region_a_(map1_region_a) {
        region_a_ = MakeRegion(kRegionPoints, LoopScene::StartPose(0), 0x00, rng_);
        region_b_ = MakeRegion(kRegionPoints, LoopScene::MiddlePose(0), 0xFF, rng_);
        region_d_ = MakeRegion(kRegionPoints, LoopScene::StartPose(0), 0x00, rng_);
        region_g_ = MakeRegion(kRegionPoints, ElsewherePose(0), 0xFF, rng_);
    }

    static constexpr map::MapId kMap1{1};
    static constexpr map::MapId kMap2{2};
    static constexpr uint32_t kRegionG = kRegionPoints;
    static constexpr uint32_t kReturnA = 2 * kRegionPoints;
    static constexpr uint32_t kReturnD = 3 * kRegionPoints;

    static RigidTransformation ElsewherePose(size_t k) {
        return Pose(300.0 + 0.1 * static_cast<double>(k), 0.0, 0.0, 0.5 * static_cast<double>(k));
    }
    static map::FrameId ReturnId(size_t k) {
        return map::FrameId{kMap2FirstFrame + static_cast<uint32_t>(kMergeStartFrames + kMergeMiddleFrames + k)};
    }
    static bool InMap1(map::FrameId frame) { return frame.value < kStartFrames + kMiddleFrames; }
    static bool InMap2(map::FrameId frame) { return frame.value >= kMap2FirstFrame; }

    // Map 1 (the LoopScene's E and B Frames), then the Mapper keeping it
    // inactive. Returns what the LoopCloser published.
    std::vector<Loop> PublishMap1(Harness& harness) {
        std::vector<Loop> loops;
        harness.set_map(kMap1);
        const Sim3 identity;
        for (size_t k = 0; k < kStartFrames; ++k) {
            const RigidTransformation T = LoopScene::StartPose(k);
            const NewPoints points =
                k == 0 ? PointsOf(region_a_, map1_region_a_, identity, Centre(T), rng_) : NewPoints{};
            Append(loops, harness.Publish(Observe(LoopScene::StartId(k), T, T, region_a_, map1_region_a_, binocular_,
                                                  rng_),
                                          points,
                                          k == 0));
        }
        for (size_t k = 0; k < kMiddleFrames; ++k) {
            const map::FrameId id{static_cast<uint32_t>(kStartFrames + k)};
            const RigidTransformation T = LoopScene::MiddlePose(k);
            const NewPoints points =
                k == 0 ? PointsOf(region_b_, LoopScene::kRegionB, identity, Centre(T), rng_) : NewPoints{};
            Append(loops, harness.Publish(Observe(id, T, T, region_b_, LoopScene::kRegionB, binocular_, rng_), points));
        }
        Append(loops, harness.PublishEmpty(kMap1, std::nullopt));
        return loops;
    }

    // Map 2's F and G Frames (a new map, active).
    std::vector<Loop> PublishMap2(Harness& harness) {
        std::vector<Loop> loops;
        harness.set_map(kMap2);
        for (size_t k = 0; k < kMergeStartFrames; ++k) {
            const RigidTransformation T = LoopScene::StartPose(k);
            const NewPoints points = k == 0 ? PointsOf(region_d_, 0, drift_, Centre(MapPose(T)), rng_) : NewPoints{};
            const map::FrameId id{kMap2FirstFrame + static_cast<uint32_t>(k)};
            Append(loops, harness.Publish(Observe(id, T, MapPose(T), region_d_, 0, binocular_, rng_), points, k == 0));
        }
        for (size_t k = 0; k < kMergeMiddleFrames; ++k) {
            const RigidTransformation T = ElsewherePose(k);
            const NewPoints points =
                k == 0 ? PointsOf(region_g_, kRegionG, drift_, Centre(MapPose(T)), rng_) : NewPoints{};
            const map::FrameId id{kMap2FirstFrame + static_cast<uint32_t>(kMergeStartFrames + k)};
            Append(loops, harness.Publish(Observe(id, T, MapPose(T), region_g_, kRegionG, binocular_, rng_), points));
        }
        return loops;
    }

    // Return Frame R_k observing `sees`; a region's new points come with the
    // first return Frame observing it.
    std::vector<Loop> PublishReturn(Harness& harness, size_t k, Sees sees) {
        const RigidTransformation T_true = LoopScene::ReturnPose(k);
        std::vector<ScenePoint> region;
        std::vector<uint32_t> ids;
        NewPoints points;
        const auto add = [&](const std::vector<ScenePoint>& observed, uint32_t first_id, bool& added) {
            for (size_t i = 0; i < observed.size(); ++i) {
                region.push_back(observed[i]);
                ids.push_back(first_id + static_cast<uint32_t>(i));
            }
            if (!added) {
                const NewPoints fresh = PointsOf(observed, first_id, drift_, Centre(MapPose(T_true)), rng_);
                if (points.positions.empty()) {
                    points = fresh;
                } else {  // contiguous: region D's points follow region A's
                    points.positions.insert(points.positions.end(), fresh.positions.begin(), fresh.positions.end());
                    points.descriptors.insert(points.descriptors.end(), fresh.descriptors.begin(),
                                              fresh.descriptors.end());
                    points.normals.insert(points.normals.end(), fresh.normals.begin(), fresh.normals.end());
                }
                added = true;
            }
        };
        if (sees != Sees::D) add(region_a_, kReturnA, added_a_);
        if (sees != Sees::A) add(region_d_, kReturnD, added_d_);
        return harness.Publish(ObserveIds(ReturnId(k), T_true, MapPose(T_true), region, ids, binocular_, rng_),
                               points);
    }

    // Map 1, map 2, then R0..R(count-1) observing `sees`.
    std::vector<Loop> PublishUpTo(Harness& harness, size_t count, Sees sees) {
        std::vector<Loop> loops = PublishMap1(harness);
        Append(loops, PublishMap2(harness));
        for (size_t k = 0; k < count; ++k) Append(loops, PublishReturn(harness, k, sees));
        return loops;
    }

    // The true camera similarity of a merge, x_query = S x_match: map 2's
    // coordinates are drift(true), map 1's the true ones.
    [[nodiscard]] Sim3 TrueS(size_t return_k, map::FrameId match) const {
        const RigidTransformation T_qm = LoopScene::ReturnPose(return_k) * LoopScene::StartPose(match.value).Inverse();
        return {drift_.scale, T_qm.rotation(), drift_.scale * T_qm.translation()};
    }

    // Region A point index of a map 1 point or a map 2 return point, or -1.
    [[nodiscard]] int64_t RegionAPointOfMap1(map::MapPointId id) const {
        return id.value >= map1_region_a_ && id.value < map1_region_a_ + kRegionPoints ? id.value - map1_region_a_
                                                                                       : -1;
    }
    static int64_t RegionAPointOfMap2(map::MapPointId id) {
        return id.value >= kReturnA && id.value < kReturnA + kRegionPoints ? id.value - kReturnA : -1;
    }

private:
    // As LoopScene::MapPose: T_map = s T_true ∘ drift⁻¹ as a rigid motion.
    [[nodiscard]] RigidTransformation MapPose(const RigidTransformation& T_true) const {
        const geometry::TMatrix33F R = T_true.rotation() * drift_.R.transpose();
        return {R, drift_.scale * T_true.translation() - R * drift_.t};
    }

    static void Append(std::vector<Loop>& out, std::vector<Loop> loops) {
        for (Loop& loop : loops) out.push_back(std::move(loop));
    }

    std::mt19937 rng_;
    Sim3 drift_;
    bool binocular_;
    uint32_t map1_region_a_;
    std::vector<ScenePoint> region_a_;
    std::vector<ScenePoint> region_b_;
    std::vector<ScenePoint> region_d_;
    std::vector<ScenePoint> region_g_;
    bool added_a_ = false;
    bool added_d_ = false;
};

std::string GateCounts(const CommonRegionStats& stats) {
    return "bow " + std::to_string(stats.bow_matches) + " ransac " + std::to_string(stats.ransac_inliers) +
           " coarse " + std::to_string(stats.coarse_projection_matches) + " optimized " +
           std::to_string(stats.optimized_inliers) + " projection " + std::to_string(stats.projection_matches);
}

// DetectNBestCandidates splits the hits by map: the loop candidates are map
// 2's (the query's, the active map), the merge candidates map 1's (inactive).
// The return Frames see only region A, mapped in map 1: map 2's start Frames
// (region D, the same words) are loop candidates that do not match, and the
// merge is found in map 1 and published there, never as a loop.
TEST(LoopCloserMergeTest, FindsMergeOnlyInInactiveMap) {
    for (uint32_t seed = 1; seed <= kSeeds; ++seed) {
        SCOPED_TRACE("seed " + std::to_string(seed));
        Harness harness(MakeRegionVocabulary());
        MergeScene scene(seed, MonoDrift(), /*binocular=*/false);
        ASSERT_TRUE(scene.PublishUpTo(harness, 1, Sees::A).empty());
        const LoopCloser<Extractor>& loop_closer = harness.loop_closer();
        ASSERT_EQ(LoopCloserTestAccess::Atlas(loop_closer).active_map(), MergeScene::kMap2);

        const CommonRegionStats& loop_stats = LoopCloserTestAccess::LastStats(loop_closer);
        EXPECT_FALSE(loop_stats.candidates.empty());
        EXPECT_TRUE(std::all_of(loop_stats.candidates.begin(), loop_stats.candidates.end(), MergeScene::InMap2));
        EXPECT_EQ(loop_stats.projection_matches, 0u);
        const CommonRegionStats& merge_stats = LoopCloserTestAccess::LastMergeStats(loop_closer);
        ASSERT_FALSE(merge_stats.candidates.empty());
        EXPECT_TRUE(std::all_of(merge_stats.candidates.begin(), merge_stats.candidates.end(), MergeScene::InMap1));
        EXPECT_TRUE(std::any_of(merge_stats.candidates.begin(), merge_stats.candidates.end(), IsStartFrame));
        // The gates of the first detection, with margins over the thresholds.
        ::testing::Test::RecordProperty("seed" + std::to_string(seed) + "_merge", GateCounts(merge_stats));
        const LoopCloserOptions options;
        EXPECT_GE(merge_stats.bow_matches, 2 * options.min_bow_matches);
        EXPECT_GE(merge_stats.ransac_inliers, 2 * options.min_sim3_inliers);
        EXPECT_GE(merge_stats.coarse_projection_matches, 2 * options.min_coarse_projection);
        EXPECT_GE(merge_stats.optimized_inliers, 2 * options.min_optimized);
        EXPECT_GE(merge_stats.projection_matches, 2 * options.min_projection);
        EXPECT_TRUE(merge_stats.normal_rejections.empty());
        EXPECT_EQ(LoopCloserTestAccess::NumCoincidences(loop_closer), 0u);

        std::vector<Loop> loops;
        for (size_t k = 1; k <= 3; ++k) {
            for (Loop& loop : scene.PublishReturn(harness, k, Sees::A)) loops.push_back(std::move(loop));
            EXPECT_EQ(LoopCloserTestAccess::NumCoincidences(loop_closer), 0u) << "R" << k;
        }
        ASSERT_EQ(loops.size(), 1u);
        EXPECT_EQ(loops[0].map_id, MergeScene::kMap2);
        EXPECT_EQ(loops[0].query_map_id, MergeScene::kMap2);
        EXPECT_EQ(loops[0].match_map_id, MergeScene::kMap1);
        EXPECT_TRUE(IsStartFrame(loops[0].match_frame));
    }
}

// mnMergeNumCoincidences: no event on R0..R2 (0, 1, 2 confirmations), the
// merge on R3, its third, close to the true similarity; mono with a free
// scale, stereo with scale 1. Its pairs match map 2's return points to map
// 1's region A points.
void ExpectMergeAtThirdConfirmation(uint32_t seed, const Sim3& drift, bool binocular) {
    SCOPED_TRACE("seed " + std::to_string(seed) + (binocular ? " stereo" : " mono"));
    Harness harness(MakeRegionVocabulary());
    MergeScene scene(seed, drift, binocular);
    ASSERT_TRUE(scene.PublishUpTo(harness, 0, Sees::A).empty());
    const LoopCloser<Extractor>& loop_closer = harness.loop_closer();
    for (size_t k = 0; k < 3; ++k) {
        EXPECT_TRUE(scene.PublishReturn(harness, k, Sees::A).empty()) << "R" << k;
        EXPECT_EQ(LoopCloserTestAccess::NumMergeCoincidences(loop_closer), k) << "R" << k;
        if (k == 0) {
            ::testing::Test::RecordProperty(
                "seed" + std::to_string(seed) + (binocular ? "_stereo" : "_mono") + "_scale" +
                    std::to_string(drift.scale) + "_merge",
                GateCounts(LoopCloserTestAccess::LastMergeStats(loop_closer)));
        }
    }
    const std::vector<Loop> loops = scene.PublishReturn(harness, 3, Sees::A);
    ASSERT_EQ(loops.size(), 1u);
    const Loop& merge = loops[0];
    EXPECT_TRUE(merge.priority);
    EXPECT_EQ(merge.query_map_id, MergeScene::kMap2);
    EXPECT_EQ(merge.match_map_id, MergeScene::kMap1);
    EXPECT_EQ(merge.query_frame, MergeScene::ReturnId(3));
    ASSERT_TRUE(IsStartFrame(merge.match_frame));
    EXPECT_EQ(LoopCloserTestAccess::NumMergeCoincidences(loop_closer), 0u);  // dropped once published

    const Sim3 truth = scene.TrueS(3, merge.match_frame);
    ::testing::Test::RecordProperty(
        "seed" + std::to_string(seed) + (binocular ? "_stereo" : "_mono") + "_scale" + std::to_string(drift.scale) +
            "_error",
        "rotation_deg " + std::to_string(RotationErrorDeg(merge.S.R, truth.R)) + " scale_ratio " +
            std::to_string(merge.S.scale / truth.scale) + " translation " +
            std::to_string((merge.S.t - truth.t).norm()));
    EXPECT_LT(RotationErrorDeg(merge.S.R, truth.R), 0.3);
    if (binocular) {
        EXPECT_EQ(merge.S.scale, 1.0);
    } else {
        EXPECT_NEAR(merge.S.scale / truth.scale, 1.0, 0.02);
    }
    EXPECT_LT((merge.S.t - truth.t).norm(), 0.05 * truth.scale);

    EXPECT_GE(merge.matched_points.size(), 100u);
    size_t correct = 0;
    for (const auto& [query_point, match_point] : merge.matched_points) {
        const int64_t scene_point = MergeScene::RegionAPointOfMap2(query_point);
        correct += scene_point >= 0 && scene_point == scene.RegionAPointOfMap1(match_point) ? 1 : 0;
    }
    EXPECT_GE(static_cast<double>(correct), 0.95 * static_cast<double>(merge.matched_points.size()));
}

TEST(LoopCloserMergeTest, MergeConfirmedThreeTimes) {
    for (uint32_t seed = 1; seed <= kSeeds; ++seed) {
        ExpectMergeAtThirdConfirmation(seed, MonoDrift(), /*binocular=*/false);
        ExpectMergeAtThirdConfirmation(seed, StereoDrift(), /*binocular=*/true);
    }
}

// Two mono maps have unrelated scales: the merge's similarity has a free
// scale, recovered within 2 % of the true ratio between the maps (here 0.5
// and 2).
TEST(LoopCloserMergeTest, MonoMergeFreeScale) {
    for (const double scale : {0.5, 2.0}) {
        SCOPED_TRACE("scale " + std::to_string(scale));
        Sim3 drift = MonoDrift();
        drift.scale = scale;
        for (uint32_t seed = 1; seed <= kSeeds; ++seed) {
            ExpectMergeAtThirdConfirmation(seed, drift, /*binocular=*/false);
        }
    }
}

// The normal gate is for loops only (controller ruling 2026-10-10): a
// merge's normals are in two unrelated worlds (each map's world is its first
// Frame's camera). Map 2's world here is turned 120 deg against map 1's, so
// every pair's normals are 120 deg apart in world coordinates (about 1.0 of
// them disagree; the gate would reject at 0.5), yet the merge is true: it is
// confirmed and published, mono and stereo.
TEST(LoopCloserMergeTest, AcceptsMergeAcrossRotatedWorlds) {
    for (const bool binocular : {false, true}) {
        Sim3 drift = binocular ? StereoDrift() : MonoDrift();
        drift.R = Eigen::AngleAxis<double>(120.0 * std::numbers::pi / 180.0, geometry::TVector3DF::UnitY())
                      .toRotationMatrix() *
                  drift.R;
        for (uint32_t seed = 1; seed <= kSeeds; ++seed) {
            ExpectMergeAtThirdConfirmation(seed, drift, binocular);
        }
    }
}

// Point ids restart per map, so a true merge pair may share its id: (map 2
// point k, map 1 point k) is not a self-match. With map 1's region A at the
// return points' ids (800..), every true pair collides; the published pairs,
// as region A point pairs, are exactly those of the same scene without the
// collision (the ids change nothing else), and each true pair has equal ids.
TEST(LoopCloserMergeTest, KeepsPairsWithCollidingIds) {
    using ScenePairs = std::set<std::pair<int64_t, int64_t>>;
    const auto merge_pairs = [](uint32_t seed, uint32_t map1_region_a) {
        Harness harness(MakeRegionVocabulary());
        MergeScene scene(seed, MonoDrift(), /*binocular=*/false, map1_region_a);
        std::vector<Loop> loops = scene.PublishUpTo(harness, 4, Sees::A);
        ScenePairs pairs;
        size_t equal_ids = 0;
        EXPECT_EQ(loops.size(), 1u);
        for (const Loop& loop : loops) {
            for (const auto& [query_point, match_point] : loop.matched_points) {
                pairs.emplace(MergeScene::RegionAPointOfMap2(query_point), scene.RegionAPointOfMap1(match_point));
                equal_ids += query_point == match_point ? 1 : 0;
            }
        }
        return std::pair{pairs, equal_ids};
    };
    for (uint32_t seed = 1; seed <= kSeeds; ++seed) {
        SCOPED_TRACE("seed " + std::to_string(seed));
        const auto [separate, separate_equal] = merge_pairs(seed, 0);
        const auto [colliding, colliding_equal] = merge_pairs(seed, MergeScene::kReturnA);
        EXPECT_EQ(separate_equal, 0u);
        ASSERT_GE(separate.size(), 100u);
        EXPECT_EQ(colliding, separate);
        size_t true_pairs = 0;
        for (const auto& [query_index, match_index] : colliding) {
            true_pairs += query_index >= 0 && query_index == match_index ? 1 : 0;
        }
        EXPECT_EQ(colliding_equal, true_pairs);
        EXPECT_GE(true_pairs, 100u);
    }
}

// The loop and merge regions have their own pending states
// (mnLoopNumCoincidences / mnMergeNumCoincidences and their not-found
// counters): R0 and R1 see both regions (one confirmation each); R2 sees only
// region A (the merge's second, the loop's first failure: kept), R3 only
// region D (the loop's second, the merge's first failure: kept). Neither
// resets the other, and nothing is published.
TEST(LoopCloserMergeTest, LoopAndMergePendingIndependent) {
    for (uint32_t seed = 1; seed <= kSeeds; ++seed) {
        SCOPED_TRACE("seed " + std::to_string(seed));
        Harness harness(MakeRegionVocabulary());
        MergeScene scene(seed, MonoDrift(), /*binocular=*/false);
        ASSERT_TRUE(scene.PublishUpTo(harness, 2, Sees::Both).empty());
        const LoopCloser<Extractor>& loop_closer = harness.loop_closer();
        EXPECT_EQ(LoopCloserTestAccess::NumCoincidences(loop_closer), 1u);
        EXPECT_EQ(LoopCloserTestAccess::NumMergeCoincidences(loop_closer), 1u);

        EXPECT_TRUE(scene.PublishReturn(harness, 2, Sees::A).empty());
        EXPECT_EQ(LoopCloserTestAccess::NumCoincidences(loop_closer), 1u);
        EXPECT_EQ(LoopCloserTestAccess::NumMergeCoincidences(loop_closer), 2u);

        EXPECT_TRUE(scene.PublishReturn(harness, 3, Sees::D).empty());
        EXPECT_EQ(LoopCloserTestAccess::NumCoincidences(loop_closer), 2u);
        EXPECT_EQ(LoopCloserTestAccess::NumMergeCoincidences(loop_closer), 2u);
    }
}

// A query that confirms the pending loop region still searches the merge
// candidates (ORB-SLAM3 runs DetectCommonRegionsFromBoW for each kind not
// confirmed in the query): R0 and R1 see only region D (a pending loop), R2
// both regions: the loop's second confirmation, and the merge region found
// (no covisible Frame of R2 sees region A: 0 confirmations). At R3 the loop's
// third confirmation publishes it, and no search runs (ORB-SLAM3 returns
// once a region is detected), so the merge region is left unconfirmed.
TEST(LoopCloserMergeTest, SearchesMergeWhileLoopConfirms) {
    for (uint32_t seed = 1; seed <= kSeeds; ++seed) {
        SCOPED_TRACE("seed " + std::to_string(seed));
        Harness harness(MakeRegionVocabulary());
        MergeScene scene(seed, MonoDrift(), /*binocular=*/false);
        ASSERT_TRUE(scene.PublishUpTo(harness, 2, Sees::D).empty());
        const LoopCloser<Extractor>& loop_closer = harness.loop_closer();
        ASSERT_EQ(LoopCloserTestAccess::NumCoincidences(loop_closer), 1u);

        EXPECT_TRUE(scene.PublishReturn(harness, 2, Sees::Both).empty());
        EXPECT_EQ(LoopCloserTestAccess::NumCoincidences(loop_closer), 2u);
        const CommonRegionStats& merge_stats = LoopCloserTestAccess::LastMergeStats(loop_closer);
        EXPECT_TRUE(std::any_of(merge_stats.candidates.begin(), merge_stats.candidates.end(), IsStartFrame));
        EXPECT_GE(merge_stats.projection_matches, LoopCloserOptions{}.min_projection);

        const std::vector<Loop> loops = scene.PublishReturn(harness, 3, Sees::Both);
        ASSERT_EQ(loops.size(), 1u);
        EXPECT_EQ(loops[0].match_map_id, MergeScene::kMap2);
        EXPECT_EQ(LoopCloserTestAccess::NumCoincidences(loop_closer), 0u);
        EXPECT_EQ(LoopCloserTestAccess::NumMergeCoincidences(loop_closer), 0u);
    }
}

// A merge and a loop confirmed at the same query (R3, the third of both):
// only the merge is published, and the loop's pending region is dropped
// (ORB-SLAM3 handles mbMergeDetected first and resets the loop).
TEST(LoopCloserMergeTest, MergeWinsOverLoop) {
    for (uint32_t seed = 1; seed <= kSeeds; ++seed) {
        SCOPED_TRACE("seed " + std::to_string(seed));
        Harness harness(MakeRegionVocabulary());
        MergeScene scene(seed, MonoDrift(), /*binocular=*/false);
        ASSERT_TRUE(scene.PublishUpTo(harness, 3, Sees::Both).empty());
        const LoopCloser<Extractor>& loop_closer = harness.loop_closer();
        ASSERT_EQ(LoopCloserTestAccess::NumCoincidences(loop_closer), 2u);
        ASSERT_EQ(LoopCloserTestAccess::NumMergeCoincidences(loop_closer), 2u);

        const std::vector<Loop> loops = scene.PublishReturn(harness, 3, Sees::Both);
        ASSERT_EQ(loops.size(), 1u);
        EXPECT_EQ(loops[0].query_map_id, MergeScene::kMap2);
        EXPECT_EQ(loops[0].match_map_id, MergeScene::kMap1);
        EXPECT_TRUE(IsStartFrame(loops[0].match_frame));
        EXPECT_EQ(LoopCloserTestAccess::NumCoincidences(loop_closer), 0u);
        EXPECT_EQ(LoopCloserTestAccess::NumMergeCoincidences(loop_closer), 0u);
    }
}

// The control of MergeWinsOverLoop: without map 1 (removed before map 2),
// the same return Frames publish the loop at R3.
TEST(LoopCloserMergeTest, LoopPublishedWithoutMerge) {
    Harness harness(MakeRegionVocabulary());
    MergeScene scene(1, MonoDrift(), /*binocular=*/false);
    ASSERT_TRUE(scene.PublishMap1(harness).empty());
    ASSERT_TRUE(harness.PublishEmpty(MergeScene::kMap1, std::nullopt, 0, {MergeScene::kMap1}).empty());
    ASSERT_TRUE(scene.PublishMap2(harness).empty());
    std::vector<Loop> loops;
    for (size_t k = 0; k <= 3; ++k) {
        for (Loop& loop : scene.PublishReturn(harness, k, Sees::Both)) loops.push_back(std::move(loop));
    }
    ASSERT_EQ(loops.size(), 1u);
    EXPECT_EQ(loops[0].query_frame, MergeScene::ReturnId(3));
    EXPECT_EQ(loops[0].match_map_id, MergeScene::kMap2);
    EXPECT_TRUE(MergeScene::InMap2(loops[0].match_frame));
}

// A removed map (its id in removed_maps) is never matched: map 1, removed
// while its merge region is pending (2 confirmations), drops that region, is
// no longer a candidate, and no merge is published.
TEST(LoopCloserMergeTest, NeverMatchesRemovedMap) {
    for (uint32_t seed = 1; seed <= kSeeds; ++seed) {
        SCOPED_TRACE("seed " + std::to_string(seed));
        Harness harness(MakeRegionVocabulary());
        MergeScene scene(seed, MonoDrift(), /*binocular=*/false);
        ASSERT_TRUE(scene.PublishUpTo(harness, 3, Sees::A).empty());
        const LoopCloser<Extractor>& loop_closer = harness.loop_closer();
        ASSERT_EQ(LoopCloserTestAccess::NumMergeCoincidences(loop_closer), 2u);

        EXPECT_TRUE(
            harness.PublishEmpty(MergeScene::kMap2, MergeScene::kMap2, 0, {MergeScene::kMap1}).empty());
        EXPECT_EQ(LoopCloserTestAccess::Atlas(loop_closer).map(MergeScene::kMap1), nullptr);
        EXPECT_EQ(LoopCloserTestAccess::NumMergeCoincidences(loop_closer), 0u);
        for (size_t k = 3; k < kReturnFrames; ++k) {
            EXPECT_TRUE(scene.PublishReturn(harness, k, Sees::A).empty()) << "R" << k;
            EXPECT_EQ(LoopCloserTestAccess::NumMergeCoincidences(loop_closer), 0u) << "R" << k;
            const CommonRegionStats& stats = LoopCloserTestAccess::LastMergeStats(loop_closer);
            EXPECT_TRUE(std::none_of(stats.candidates.begin(), stats.candidates.end(), MergeScene::InMap1))
                << "R" << k;
        }
    }
}

// The ORB-SLAM3 source values (LoopClosing.cc, KeyFrameDatabase.cc).
TEST(LoopCloserOptionsTest, OrbSlam3Defaults) {
    const LoopCloserOptions options;
    EXPECT_FALSE(options.enabled);  // opt-in, user decision 2026-10-10
    EXPECT_DOUBLE_EQ(options.max_normal_disagreement, 0.5);  // user decision, not ORB-SLAM3
    EXPECT_EQ(options.min_map_frames, 12u);
    EXPECT_EQ(options.num_candidates, 3u);
    EXPECT_EQ(options.num_covisibles, 10u);
    EXPECT_EQ(options.min_bow_matches, 20u);
    EXPECT_EQ(options.min_sim3_inliers, 15u);
    EXPECT_EQ(options.min_coarse_projection, 50u);
    EXPECT_EQ(options.min_optimized, 20u);
    EXPECT_EQ(options.min_projection, 80u);
    EXPECT_EQ(options.confirmation_projection, 30u);
    EXPECT_EQ(options.refine_min_optimized, 50u);
    EXPECT_EQ(options.refine_min_projection, 100u);
    EXPECT_EQ(options.required_coincidences, 3u);
    EXPECT_EQ(options.max_unconfirmed_queries, 2u);
    EXPECT_EQ(options.max_query_backlog, 8u);  // ours: ORB-SLAM3's queue is unbounded
    EXPECT_EQ(options.ransac_iterations, 300);
    EXPECT_DOUBLE_EQ(options.ransac_confidence, 0.99);
    EXPECT_DOUBLE_EQ(options.ransac_chi2, 9.21);
    EXPECT_DOUBLE_EQ(options.min_common_words_ratio, 0.8);
    EXPECT_DOUBLE_EQ(options.bow_nn_ratio, 0.9);
    EXPECT_DOUBLE_EQ(options.coarse_projection_radius, 8.0);
    EXPECT_DOUBLE_EQ(options.coarse_projection_max_distance, 75.0);
    EXPECT_DOUBLE_EQ(options.projection_radius, 5.0);
    EXPECT_DOUBLE_EQ(options.projection_max_distance, 50.0);
    EXPECT_DOUBLE_EQ(options.confirmation_radius, 3.0);
    EXPECT_DOUBLE_EQ(options.confirmation_max_distance, 75.0);
}

} // namespace
} // namespace voyahslam::loop_closing
