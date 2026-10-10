#include <gtest/gtest.h>
#include <support/test_measurements.h>
#include <voyahslam/events/async_observer.h>
#include <voyahslam/events/events.h>
#include <voyahslam/events/observable.h>
#include <voyahslam/global_ba/global_bundle_adjuster.h>
#include <voyahslam/memory/object_pool.h>

#include <Eigen/Geometry>
#include <algorithm>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <thread>
#include <utility>
#include <vector>

namespace voyahslam::global_ba {
namespace {

using Extractor = test::TestExtractor;
using TTransaction = events::MapTransaction<Extractor>;
using geometry::RigidTransformation;
using geometry::TPoint3DF;

constexpr size_t kNumFrames = 6;
constexpr map::MapId kMap{1};

// A deterministic scene (as GlobalBaTest's): a 7 x 5 grid of points at
// depths 5 and 7 m in front of a camera moving 0.2 m to the right and turning
// 0.02 rad per Frame.
struct Scene {
    std::vector<TPoint3DF> points;
    std::vector<RigidTransformation> poses;  // T_cw
};

const Scene& TheScene() {
    static const Scene scene = [] {
        Scene s;
        for (const double z : {5.0, 7.0}) {
            for (int ix = -3; ix <= 3; ++ix) {
                for (int iy = -2; iy <= 2; ++iy) {
                    s.points.emplace_back(0.5 * ix, 0.5 * iy, z);
                }
            }
        }
        for (size_t k = 0; k < kNumFrames; ++k) {
            const double step = static_cast<double>(k);
            const Eigen::Matrix3d R_wc = Eigen::AngleAxisd(0.02 * step, Eigen::Vector3d::UnitY()).toRotationMatrix();
            const RigidTransformation T_wc(R_wc.cast<precision_t>(),
                                           Eigen::Vector3d(0.2 * step, 0.0, 0.0).cast<precision_t>());
            s.poses.push_back(T_wc.Inverse());
        }
        return s;
    }();
    return scene;
}

// The exact monocular measurement of every scene point by a camera at T_cw:
// feature i is point i.
features::TMeasurement<Extractor> Measure(const RigidTransformation& T_cw) {
    const Scene& scene = TheScene();
    const auto n = static_cast<Eigen::Index>(scene.points.size());
    const geometry::TMatrix33F K = test::TestIntrinsics();
    geometry::TPoints2DF pixels(2, n);
    for (Eigen::Index i = 0; i < n; ++i) {
        pixels.col(i) = (K * T_cw.Transform(scene.points[static_cast<size_t>(i)])).hnormalized();
    }
    return features::TMonocularMeasurement<Extractor>(
        test::TestImageFeatures(geometry::TPoints2DF(pixels), geometry::TRowVectorF::Ones(n),
                                geometry::TRowVectorF::Zero(n),
                                test::TestImageFeatures::TDescriptors::Zero(Extractor::DescriptorSize, n),
                                features::TLevels::Zero(n)),
        K, nullptr, 0.0);
}

// What a GlobalBAFinished carried.
struct Result {
    map::MapId map_id;
    uint32_t map_epoch = 0;
    std::vector<uint32_t> frames;  // sorted
    std::vector<uint32_t> points;  // sorted
    bool priority = false;
    std::vector<std::pair<map::FrameId, RigidTransformation>> frame_poses;  // as published
    std::vector<std::pair<map::MapPointId, TPoint3DF>> point_positions;
};

std::vector<uint32_t> Range(uint32_t n) {
    std::vector<uint32_t> values(n);
    for (uint32_t i = 0; i < n; ++i) values[i] = i;
    return values;
}

// A GlobalBundleAdjuster fed hand-built MapTransactions in place of the
// Mapper.
class Harness {
public:
    // `pool_size`: the MapTransactions this harness can have in flight.
    explicit Harness(GlobalBaOptions options = {}, size_t pool_size = 16)
        : transaction_pool_(pool_size), global_ba_(options) {
        mapper_.ConnectAsync(global_ba_.inbox());
        global_ba_.AddObserver(capture_);
    }
    // The inbox holds this harness's events and capture_ the GlobalBA's: both
    // are emptied before anything is destroyed.
    ~Harness() {
        global_ba_.Stop();
        while (global_ba_.ProcessPending()) {
        }
        Take();
    }

    // Publishes a new map `map_id` at `map_epoch`: the scene's Frames at
    // their true poses, each observing every point (exactly), the points
    // slightly off.
    void SendMap(map::MapId map_id, uint32_t map_epoch = 0) {
        Send(map_id, map_epoch, [](TTransaction& transaction) {
            const Scene& scene = TheScene();
            transaction.new_map = true;
            transaction.start_frame = map::FrameId{0};
            for (size_t k = 0; k < kNumFrames; ++k) {
                events::NewFrame<Extractor>& added = transaction.added_frames.emplace_back();
                added.id = {static_cast<uint32_t>(k)};
                added.T_cw = scene.poses[k];
                added.measurement = Measure(scene.poses[k]);
                if (k > 0) added.parent = map::FrameId{static_cast<uint32_t>(k - 1)};
                for (size_t i = 0; i < scene.points.size(); ++i) {
                    added.observed_features.push_back(static_cast<uint32_t>(i));
                    added.observed_points.push_back({static_cast<uint32_t>(i)});
                }
            }
            transaction.new_points_start_id = {0};
            transaction.ResizeNewPoints(static_cast<uint32_t>(scene.points.size()));
            for (size_t i = 0; i < scene.points.size(); ++i) {
                const auto col = static_cast<Eigen::Index>(i);
                // Off by up to 5 cm (deterministic): the BA iterates.
                const double offset = 0.05 * (static_cast<double>(i % 5) - 2.0) / 2.0;
                transaction.new_positions.col(col) = scene.points[i] + TPoint3DF(offset, -offset, offset);
                transaction.new_descriptors.col(col).setZero();
                transaction.new_normals.col(col) = TPoint3DF(0, 0, -1);
            }
        });
    }

    // An ordinary transaction: no change.
    void SendOrdinary(map::MapId map_id, uint32_t map_epoch) {
        Send(map_id, map_epoch, [](TTransaction&) {});
    }

    // A transaction of map 1 at epoch 1 that culls Frame 5 and points 3 and
    // 4, and moves Frame 2.
    void SendEdit() {
        Send(kMap, 1, [](TTransaction& transaction) {
            transaction.removed_frames.push_back(map::FrameId{5});
            transaction.removed_points = {{3}, {4}};
            transaction.updated_poses.emplace_back(map::FrameId{2}, TheScene().poses[0]);
        });
    }

    // A loop correction at `map_epoch`: a loop edge between Frames a and b.
    void SendCorrection(map::MapId map_id, uint32_t map_epoch, uint32_t a = 0, uint32_t b = kNumFrames - 2) {
        Send(map_id, map_epoch, [&](TTransaction& transaction) {
            transaction.added_loop_edges.emplace_back(map::FrameId{a}, map::FrameId{b});
        });
    }

    // Runs the GlobalBA until it is idle; returns what it published, or
    // nothing while held.
    std::vector<Result> Run() {
        while (global_ba_.ProcessPending()) {
        }
        return hold_ ? std::vector<Result>{} : Take();
    }

    // While held, published events stay in the queue (in flight).
    void set_hold(bool hold) { hold_ = hold; }

    // Takes (and acknowledges) everything published.
    std::vector<Result> Take() {
        std::vector<Result> results;
        while (events::Event* event = listener_.GetNextEvent()) {
            if (event->type != events::EventType::GlobalBAFinished) continue;
            const auto& finished = *static_cast<events::GlobalBAFinished*>(event);
            Result& result = results.emplace_back();
            result.map_id = finished.map_id;
            result.map_epoch = finished.map_epoch;
            result.priority = finished.priority;
            result.frame_poses = finished.frame_poses;
            result.point_positions = finished.point_positions;
            for (const auto& [id, pose] : finished.frame_poses) result.frames.push_back(id.value);
            for (const auto& [id, position] : finished.point_positions) result.points.push_back(id.value);
            std::sort(result.frames.begin(), result.frames.end());
            std::sort(result.points.begin(), result.points.end());
        }
        return results;
    }

    GlobalBundleAdjuster<Extractor>& global_ba() { return global_ba_; }
    // Sends that found every transaction in flight (none was published).
    [[nodiscard]] size_t num_exhausted() const { return num_exhausted_; }
    [[nodiscard]] size_t num_sent() const { return num_sent_; }

private:
    template <typename TFill>
    void Send(map::MapId map_id, uint32_t map_epoch, TFill fill) {
        TTransaction* transaction = transaction_pool_.TryCreate();
        if (transaction == nullptr) {
            ++num_exhausted_;
            return;
        }
        ++num_sent_;
        transaction->map_id = map_id;
        transaction->map_epoch = map_epoch;
        fill(*transaction);
        mapper_.Notify(transaction);
    }

    // Declaration order: the pool outlives the GlobalBA's inbox, the GlobalBA
    // outlives capture_.
    memory::ObjectPool<TTransaction, memory::ThreadSafePolicy> transaction_pool_{16};
    events::Observable mapper_;
    GlobalBundleAdjuster<Extractor> global_ba_;
    events::AsyncObserver capture_;
    events::Listener listener_ = capture_.GetListener();
    bool hold_ = false;
    size_t num_exhausted_ = 0;
    size_t num_sent_ = 0;
};

// Ordinary transactions start nothing; a correction starts one run whose
// result covers every Frame and point at the correction's epoch, and is
// urgent for the Mapper.
TEST(GlobalBundleAdjusterTest, StartsOnlyOnLoopEdge) {
    Harness harness;
    harness.SendMap(kMap);
    harness.SendOrdinary(kMap, 0);
    harness.SendOrdinary(kMap, 0);
    EXPECT_TRUE(harness.Run().empty());
    EXPECT_EQ(harness.global_ba().num_runs_started(), 0u);
    ASSERT_EQ(harness.global_ba().replica().num_keyframes(), kNumFrames);

    harness.SendCorrection(kMap, 3);
    const std::vector<Result> results = harness.Run();
    ASSERT_EQ(results.size(), 1u);
    EXPECT_EQ(results[0].map_id, kMap);
    EXPECT_EQ(results[0].map_epoch, 3u);
    EXPECT_TRUE(results[0].priority);
    EXPECT_EQ(results[0].frames, Range(kNumFrames));
    EXPECT_EQ(results[0].points, Range(static_cast<uint32_t>(TheScene().points.size())));
    EXPECT_EQ(harness.global_ba().num_runs_started(), 1u);
    EXPECT_EQ(harness.global_ba().num_runs_interrupted(), 0u);

    // Transactions after the correction start nothing.
    harness.SendOrdinary(kMap, 3);
    EXPECT_TRUE(harness.Run().empty());
    EXPECT_EQ(harness.global_ba().num_runs_started(), 1u);
}

// A newer correction waiting in the inbox interrupts the run of epoch N
// (nothing is published for it) and is optimized once applied.
TEST(GlobalBundleAdjusterTest, NewerCorrectionInterruptsRun) {
    Harness harness;
    harness.SendMap(kMap);
    harness.SendCorrection(kMap, 1);
    bool sent = false;
    harness.global_ba().set_run_hook([&] {
        if (!sent) {
            harness.SendCorrection(kMap, 2, 1, 4);
            sent = true;
        }
    });
    const std::vector<Result> results = harness.Run();
    harness.global_ba().set_run_hook({});

    ASSERT_EQ(results.size(), 1u);
    EXPECT_EQ(results[0].map_epoch, 2u);
    EXPECT_EQ(harness.global_ba().num_runs_started(), 2u);
    EXPECT_EQ(harness.global_ba().num_runs_interrupted(), 1u);
    EXPECT_EQ(harness.global_ba().replica().map_epoch(), 2u);
}

// Transactions without a loop edge, even at a newer epoch, leave the run be.
TEST(GlobalBundleAdjusterTest, OrdinaryTransactionDoesNotInterruptRun) {
    Harness harness;
    harness.SendMap(kMap);
    harness.SendCorrection(kMap, 1);
    harness.global_ba().set_run_hook([&] {
        harness.SendOrdinary(kMap, 1);
        harness.SendOrdinary(kMap, 2);
    });
    const std::vector<Result> results = harness.Run();
    harness.global_ba().set_run_hook({});

    ASSERT_EQ(results.size(), 1u);
    EXPECT_EQ(results[0].map_epoch, 1u);
    EXPECT_EQ(harness.global_ba().num_runs_interrupted(), 0u);
    EXPECT_EQ(harness.global_ba().replica().map_epoch(), 2u);  // applied after the run
}

// A new map clears the replica and drops the pending run, whether it comes
// before the run (in the same drain) or while it runs.
TEST(GlobalBundleAdjusterTest, NewMapClearsAndDropsRun) {
    for (const bool during_run : {false, true}) {
        SCOPED_TRACE(during_run ? "during the run" : "before the run");
        Harness harness;
        harness.SendMap(kMap);
        harness.SendCorrection(kMap, 1);
        if (during_run) {
            harness.global_ba().set_run_hook([&] { harness.SendMap(map::MapId{2}); });
        } else {
            harness.SendMap(map::MapId{2});
        }
        EXPECT_TRUE(harness.Run().empty());
        harness.global_ba().set_run_hook({});

        EXPECT_EQ(harness.global_ba().num_runs_started(), during_run ? 1u : 0u);
        EXPECT_EQ(harness.global_ba().num_runs_interrupted(), during_run ? 1u : 0u);
        EXPECT_EQ(harness.global_ba().replica().map_id(), map::MapId{2});
        EXPECT_EQ(harness.global_ba().replica().map_epoch(), 0u);
        EXPECT_EQ(harness.global_ba().replica().num_keyframes(), kNumFrames);
    }
}

// A newer map id without new_map is a new map too.
TEST(GlobalBundleAdjusterTest, NewerMapIdDropsRun) {
    Harness harness;
    harness.SendMap(kMap);
    harness.SendCorrection(kMap, 1);
    harness.SendOrdinary(map::MapId{2}, 0);
    EXPECT_TRUE(harness.Run().empty());
    EXPECT_EQ(harness.global_ba().num_runs_started(), 0u);
    EXPECT_EQ(harness.global_ba().replica().map_id(), map::MapId{2});
}

// Events of an older map (and of map 0) neither start nor interrupt a run.
TEST(GlobalBundleAdjusterTest, IgnoresOldMap) {
    Harness harness;
    harness.SendMap(map::MapId{2});
    harness.SendCorrection(kMap, 5);
    harness.SendCorrection(map::MapId{0}, 5);
    EXPECT_TRUE(harness.Run().empty());
    EXPECT_EQ(harness.global_ba().num_runs_started(), 0u);
    EXPECT_EQ(harness.global_ba().replica().map_id(), map::MapId{2});
    EXPECT_EQ(harness.global_ba().replica().map_epoch(), 0u);

    harness.SendCorrection(map::MapId{2}, 1);
    harness.global_ba().set_run_hook([&] {
        harness.SendCorrection(kMap, 9);
        harness.SendMap(kMap, 9);
    });
    const std::vector<Result> results = harness.Run();
    harness.global_ba().set_run_hook({});
    ASSERT_EQ(results.size(), 1u);
    EXPECT_EQ(results[0].map_id, map::MapId{2});
    EXPECT_EQ(results[0].map_epoch, 1u);
    EXPECT_EQ(harness.global_ba().num_runs_interrupted(), 0u);
    EXPECT_EQ(harness.global_ba().replica().map_id(), map::MapId{2});
    EXPECT_EQ(harness.global_ba().replica().map_epoch(), 1u);
}

// Review Focus 4: Stop() interrupts a running BA on the worker thread; it
// returns and nothing is published.
TEST(GlobalBundleAdjusterTest, StopInterruptsRunningBa) {
    Harness harness;
    harness.SendMap(kMap);
    harness.SendCorrection(kMap, 1);
    std::atomic<bool> in_run{false};
    GlobalBundleAdjuster<Extractor>& global_ba = harness.global_ba();
    global_ba.set_run_hook([&] {
        in_run.store(true);
        while (!global_ba.stop_requested()) {
            std::this_thread::yield();
        }
    });
    global_ba.Start();
    while (!in_run.load()) {
        std::this_thread::yield();
    }
    global_ba.Stop();
    global_ba.set_run_hook({});

    EXPECT_TRUE(harness.Take().empty());
    EXPECT_EQ(global_ba.num_runs_started(), 1u);
    EXPECT_EQ(global_ba.num_runs_interrupted(), 1u);
}

// Review Focus 5: with every result in flight the next one is dropped and
// counted, without an abort.
TEST(GlobalBundleAdjusterTest, DropsResultWhenPoolExhausted) {
    GlobalBaOptions options;
    options.max_events_in_flight = 1;
    Harness harness(options);
    harness.SendMap(kMap);
    harness.set_hold(true);
    harness.SendCorrection(kMap, 1);
    EXPECT_TRUE(harness.Run().empty());
    harness.SendCorrection(kMap, 2, 1, 4);
    EXPECT_TRUE(harness.Run().empty());
    EXPECT_EQ(harness.global_ba().num_runs_started(), 2u);
    EXPECT_EQ(harness.global_ba().num_results_dropped(), 1u);

    harness.set_hold(false);
    const std::vector<Result> results = harness.Take();
    ASSERT_EQ(results.size(), 1u);
    EXPECT_EQ(results[0].map_epoch, 1u);
}

// A run applies (and so releases) each transaction it finds at a poll: the
// Mapper's pool never waits for the run to end. 2 transactions per poll from a
// pool of 4 run out by the third poll if they are held.
TEST(GlobalBundleAdjusterTest, RunReleasesTransactionsAtEachPoll) {
    constexpr size_t kPool = 4;
    Harness harness({}, kPool);
    harness.SendMap(kMap);
    harness.SendCorrection(kMap, 2);
    size_t polls = 0;
    harness.global_ba().set_poll_hook([&] {
        ++polls;
        harness.SendOrdinary(kMap, 2);
        harness.SendOrdinary(kMap, 2);
    });
    const std::vector<Result> results = harness.Run();
    harness.global_ba().set_poll_hook({});

    ASSERT_GT(polls, kPool);  // more sent during the run than the pool holds
    EXPECT_EQ(harness.num_exhausted(), 0u);
    ASSERT_EQ(results.size(), 1u);
    EXPECT_EQ(results[0].map_epoch, 2u);
    EXPECT_EQ(harness.global_ba().num_runs_interrupted(), 0u);
}

// A newer correction that arrives after the BA started iterating (at its
// second poll) interrupts it too.
TEST(GlobalBundleAdjusterTest, NewerCorrectionInterruptsBetweenIterations) {
    Harness harness;
    harness.SendMap(kMap);
    harness.SendCorrection(kMap, 1);
    size_t polls = 0;
    harness.global_ba().set_poll_hook([&] {
        if (++polls == 2) harness.SendCorrection(kMap, 2, 1, 4);
    });
    const std::vector<Result> results = harness.Run();
    harness.global_ba().set_poll_hook({});

    ASSERT_EQ(results.size(), 1u);
    EXPECT_EQ(results[0].map_epoch, 2u);
    EXPECT_EQ(harness.global_ba().num_runs_started(), 2u);
    EXPECT_EQ(harness.global_ba().num_runs_interrupted(), 1u);
}

// RunGlobalBa's contract: it optimizes the replica as it stood at its first
// poll. A transaction applied during the run (culling a Frame and points,
// moving another Frame) changes neither the problem nor the published result,
// which carries the run's map id and epoch.
TEST(GlobalBundleAdjusterTest, ChangesDuringRunDoNotAffectResult) {
    std::vector<Result> undisturbed;
    {
        Harness harness;
        harness.SendMap(kMap);
        harness.SendCorrection(kMap, 1);
        undisturbed = harness.Run();
    }
    ASSERT_EQ(undisturbed.size(), 1u);

    Harness harness;
    harness.SendMap(kMap);
    harness.SendCorrection(kMap, 1);
    size_t polls = 0;
    harness.global_ba().set_poll_hook([&] {
        if (++polls == 1) harness.SendEdit();
    });
    const std::vector<Result> results = harness.Run();
    harness.global_ba().set_poll_hook({});

    ASSERT_GT(polls, 1u);  // the edit was applied before the run ended
    ASSERT_EQ(results.size(), 1u);
    EXPECT_EQ(results[0].map_id, kMap);
    EXPECT_EQ(results[0].map_epoch, 1u);
    EXPECT_EQ(results[0].frames, Range(kNumFrames));
    EXPECT_EQ(results[0].points, Range(static_cast<uint32_t>(TheScene().points.size())));
    ASSERT_EQ(results[0].frame_poses.size(), undisturbed[0].frame_poses.size());
    for (size_t i = 0; i < results[0].frame_poses.size(); ++i) {
        EXPECT_EQ(results[0].frame_poses[i].first, undisturbed[0].frame_poses[i].first);
        EXPECT_EQ(results[0].frame_poses[i].second.rotation(), undisturbed[0].frame_poses[i].second.rotation());
        EXPECT_EQ(results[0].frame_poses[i].second.translation(),
                  undisturbed[0].frame_poses[i].second.translation());
    }
    EXPECT_EQ(results[0].point_positions, undisturbed[0].point_positions);
    EXPECT_EQ(harness.global_ba().replica().num_keyframes(), kNumFrames - 1);
}

} // namespace
} // namespace voyahslam::global_ba
