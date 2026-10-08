#include <gtest/gtest.h>
#include "tracking/tracker_test_fixture.h"
#include <support/synthetic_sequence.h>
#include <support/test_measurements.h>
#include <support/trajectory_error.h>
#include <voyahslam/events/async_observer.h>
#include <voyahslam/events/events.h>
#include <voyahslam/events/observable.h>
#include <voyahslam/features/stereo_matcher.h>
#include <voyahslam/mapping/mapper.h>
#include <voyahslam/memory/object_pool.h>
#include <voyahslam/tracking/tracker.h>

#include <chrono>
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <string_view>
#include <thread>
#include <utility>
#include <variant>
#include <vector>

namespace voyahslam::tracking {
namespace {

// Mono accuracy (spec success criterion 1, Ruling T) is measured on a
// well-conditioned initial map: the default 1 deg minimum parallax initializes
// the synthetic sequence at frame 3 (~1.4 deg), whose map has ~12 % depth
// error, and the tracked translation inherits it. 4 deg initializes at frame 10
// (~3 % depth error).
class TrackerMonoAccuracyTest : public TrackerTest {
protected:
    TrackerMonoAccuracyTest() : TrackerTest(MonoAccuracyMapperOptions()) {}

    static mapping::MapperOptions MonoAccuracyMapperOptions() {
        mapping::MapperOptions options;
        options.initializer.reconstruction.min_parallax_deg = 4.0;
        return options;
    }
};

TEST_F(TrackerTest, ForwardsMeasurementsWhileWaitingForMap) {
    const auto captured = Feed(test::MakeMonocular(200, 0.0));

    ASSERT_EQ(captured.size(), 1u);
    EXPECT_EQ(captured[0].type, events::EventType::InitializationMeasurement);
    EXPECT_EQ(captured[0].map_id, map::MapId{0});
    EXPECT_EQ(tracker_.state(), TrackerState::WaitingForMap);
    // The Mapper took it as its reference frame: no map yet.
    EXPECT_EQ(mapper_.state(), mapping::MapperState::Initializing);
}

TEST_F(TrackerTest, StereoMatchedBeforeForwarding) {
    auto frame = test::MakeSyntheticStereoFrame();
    ASSERT_FALSE(frame.measurement.stereo_matched());
    const features::StereoMatches expected =
        features::MatchStereo<Extractor>(frame.measurement, features::StereoMatcherOptions{});
    tracker_.Track(std::move(frame.measurement));
    ASSERT_TRUE(tracker_.ProcessPending());  // the Mapper does not run

    auto* event = listener_.GetNextEvent();
    ASSERT_NE(event, nullptr);
    ASSERT_EQ(event->type, events::EventType::InitializationMeasurement);
    const auto& init = *static_cast<TInitializationMeasurement*>(event);
    ASSERT_TRUE(std::holds_alternative<TTracker::TBinocularMeasurement>(init.measurement));
    const auto& binocular = std::get<TTracker::TBinocularMeasurement>(init.measurement);
    EXPECT_TRUE(binocular.stereo_matched());
    ASSERT_EQ(binocular.depths().size(), expected.depths.size());
    EXPECT_TRUE(binocular.depths().isApprox(expected.depths));
    ASSERT_EQ(binocular.right_indices().size(), expected.right_indices.size());
    EXPECT_TRUE(binocular.right_indices() == expected.right_indices);
    EXPECT_GT((binocular.depths().array() > 0).count(), 0);
}

TEST_F(TrackerTest, NewMapTransactionStartsTracking) {
    // The Mapper publishes the new map as one transaction (Frame and points):
    // the replica applies it, and its start Frame becomes the last frame.
    EXPECT_EQ(tracker_.state(), TrackerState::WaitingForMap);
    EXPECT_EQ(tracker_.current_map_id(), map::MapId{0});
    const auto sequence = StereoSequence();
    ASSERT_EQ(InitializeStereo(sequence, 0), 0u);

    EXPECT_EQ(tracker_.state(), TrackerState::Tracking);
    EXPECT_EQ(tracker_.current_map_id(), map::MapId{1});
    const MapReplica<Extractor>& replica = tracker_.replica();
    EXPECT_EQ(replica.map_id(), map::MapId{1});
    EXPECT_EQ(replica.num_points(), mapper_.num_map_points());
    EXPECT_GE(replica.num_points(), 100u);
    ASSERT_EQ(replica.keyframe_ids().size(), 1u);
    ExpectLastMatchesAreKeyFrameObservations(replica.keyframe_ids().front());
}

TEST_F(TrackerTest, TracksAndPublishesFrameTracked) {
    const auto sequence = StereoSequence();
    ASSERT_EQ(InitializeStereo(sequence, 0), 0u);
    const map::FrameId keyframe = tracker_.replica().keyframe_ids().front();

    const auto captured = Feed(sequence.frames[1].measurement);

    ASSERT_EQ(captured.size(), 1u);
    ASSERT_EQ(captured[0].type, events::EventType::FrameTracked);
    EXPECT_EQ(captured[0].map_id, map::MapId{1});
    ASSERT_TRUE(captured[0].has_reference);
    EXPECT_EQ(captured[0].frame_id, keyframe);  // the new map's start Frame
    EXPECT_GE(captured[0].num_inliers, TrackingOptions{}.min_local_map_inliers);
    // The published observations are the last frame's matches.
    size_t num_last_matches = 0;
    for (const int64_t id : tracker_.last_matches()) num_last_matches += id >= 0 ? 1 : 0;
    EXPECT_EQ(captured[0].num_observed_points, num_last_matches);
    EXPECT_DOUBLE_EQ(captured[0].timestamp, sequence.frames[1].measurement.timestamp());
    ExpectTrackedMetric(captured, sequence.frames[1].T_cw,
                        test::MapAlignment(sequence.frames[0].T_cw), 1);
    EXPECT_EQ(tracker_.state(), TrackerState::Tracking);
}

// Tracker with a configurable motion-model gate.
class TrackerGateTest : public TrackerTest {
protected:
    explicit TrackerGateTest(size_t min_motion_model_inliers = 30)
        : TrackerTest({}, GateOptions(min_motion_model_inliers)) {}

    static TrackerOptions GateOptions(size_t min_motion_model_inliers) {
        TrackerOptions options;
        options.tracking.min_motion_model_inliers = min_motion_model_inliers;
        return options;
    }

    // Tracks frames 1 and 2 (frame 2 is the first with a velocity) and returns
    // the stages of their initial poses.
    std::vector<events::TrackingStage> TrackTwoFramesStages() {
        const auto sequence = StereoSequence();
        EXPECT_EQ(InitializeStereo(sequence, 0), 0u);
        std::vector<events::TrackingStage> stages;
        for (size_t k = 1; k <= 2; ++k) {
            const auto captured = Feed(sequence.frames[k].measurement);
            EXPECT_EQ(captured.size(), 1u);
            if (captured.size() == 1 && captured[0].type == events::EventType::FrameTracked) {
                stages.push_back(captured[0].stage);
            } else {
                ADD_FAILURE() << "frame " << k << " not tracked";
            }
        }
        return stages;
    }
};

class TrackerStrictGateTest : public TrackerGateTest {
protected:
    // No frame of the synthetic sequence reaches this many motion-model inliers.
    TrackerStrictGateTest() : TrackerGateTest(100000) {}
};

TEST_F(TrackerGateTest, MotionModelAcceptedAboveGate) {
    EXPECT_EQ(TrackingOptions{}.min_motion_model_inliers, 30u);
    EXPECT_EQ(TrackingOptions{}.min_motion_inliers, 10u);
    // Frame 1 has no velocity yet: zero motion.
    const std::vector<events::TrackingStage> expected{events::TrackingStage::ZeroMotion,
                                                      events::TrackingStage::MotionModel};
    EXPECT_EQ(TrackTwoFramesStages(), expected);
}

TEST_F(TrackerStrictGateTest, WeakMotionModelFallsBackToZeroMotion) {
    // Frame 2 is tracked with a velocity; its motion-model pose has fewer
    // inliers than the gate, so zero motion (gated at min_motion_inliers only)
    // produces the pose.
    const std::vector<events::TrackingStage> expected{events::TrackingStage::ZeroMotion,
                                                      events::TrackingStage::ZeroMotion};
    EXPECT_EQ(TrackTwoFramesStages(), expected);
}

TEST_F(TrackerTest, UrgentWhenInliersNearGate) {
    const TrackingOptions options;
    EXPECT_TRUE(options.IsUrgent(40));
    EXPECT_FALSE(options.IsUrgent(60));
    EXPECT_TRUE(options.IsUrgent(44));
    EXPECT_FALSE(options.IsUrgent(45));

    // The published event mirrors the rule, in both flags.
    const auto sequence = StereoSequence();
    ASSERT_EQ(InitializeStereo(sequence, 0), 0u);
    const auto captured = Feed(sequence.frames[1].measurement);
    ASSERT_EQ(captured.size(), 1u);
    EXPECT_EQ(captured[0].urgent, options.IsUrgent(captured[0].num_inliers));
    EXPECT_EQ(captured[0].priority, captured[0].urgent);
}

TEST_F(TrackerTest, FrameTrackedCarriesLocalMap) {
    const test::SyntheticSequence sequence = test::MakeSyntheticSequence();
    size_t k = 0;
    for (; k < sequence.frames.size() && tracker_.state() != TrackerState::Tracking; ++k) {
        Feed(sequence.frames[k].measurement);
    }
    ASSERT_EQ(tracker_.state(), TrackerState::Tracking);
    ASSERT_LT(k, sequence.frames.size());
    std::vector<map::FrameId> frames = tracker_.replica().keyframe_ids();
    ASSERT_EQ(frames.size(), 2u);

    const auto captured = Feed(sequence.frames[k].measurement);

    ASSERT_EQ(captured.size(), 1u);
    ASSERT_EQ(captured[0].type, events::EventType::FrameTracked);
    std::vector<map::FrameId> local_frames = captured[0].local_frames;
    std::sort(local_frames.begin(), local_frames.end());
    EXPECT_EQ(local_frames, frames);
    ASSERT_TRUE(captured[0].has_reference);
    EXPECT_NE(std::find(frames.begin(), frames.end(), captured[0].frame_id), frames.end());
    // Each visible point once; every observed point is visible.
    std::vector<map::MapPointId> visible = captured[0].visible_points;
    std::sort(visible.begin(), visible.end());
    EXPECT_EQ(std::adjacent_find(visible.begin(), visible.end()), visible.end());
    ASSERT_FALSE(captured[0].observed_points.empty());
    for (const map::MapPointId point : captured[0].observed_points) {
        EXPECT_TRUE(std::binary_search(visible.begin(), visible.end(), point))
            << "point " << point.value;
    }
}

TEST_F(TrackerTest, MergesOwnFrameObservationsIntoLastFrame) {
    const auto sequence = StereoSequence();
    ASSERT_EQ(InitializeStereo(sequence, 0), 0u);
    ASSERT_EQ(Feed(sequence.frames[1].measurement).size(), 1u);
    const std::vector<int64_t> before = tracker_.last_matches();
    // Two unmatched features of the last frame.
    std::vector<uint32_t> unmatched;
    for (size_t i = 0; i < before.size() && unmatched.size() < 2; ++i) {
        if (before[i] < 0) unmatched.push_back(static_cast<uint32_t>(i));
    }
    ASSERT_EQ(unmatched.size(), 2u);
    const auto new_id = static_cast<uint32_t>(tracker_.replica().num_points());

    // A Frame of the last frame's measurement: feature unmatched[0] observes
    // a new point, unmatched[1] an existing one.
    const auto inject = [&](uint32_t frame, const auto& measurement) {
        auto* created = transaction_pool_.Create();
        created->map_id = map::MapId{1};
        auto& added = created->added_frames.emplace_back();
        added.id = map::FrameId{frame};
        added.T_cw = sequence.frames[1].T_cw;
        added.measurement = measurement;
        added.observed_features = unmatched;
        added.observed_points = {map::MapPointId{new_id}, map::MapPointId{0}};
        created->new_points_start_id = map::MapPointId{new_id};
        created->ResizeNewPoints(1);
        created->new_positions.col(0) = geometry::TPoint3DF(0, 0, 5);
        created->new_descriptors.col(0).setZero();
        created->new_normals.col(0) = geometry::TPoint3DF(0, 0, 1);
        injector_.Notify(created);
        Drain();
    };

    // Another measurement's Frame: not merged.
    inject(900, sequence.frames[2].measurement);
    EXPECT_EQ(tracker_.last_matches(), before);

    inject(901, sequence.frames[1].measurement);
    std::vector<int64_t> expected = before;
    expected[unmatched[0]] = new_id;
    expected[unmatched[1]] = 0;
    EXPECT_EQ(tracker_.last_matches(), expected);
}

TEST_F(TrackerTest, RemovedAndReplacedPointsInLastFrame) {
    const auto sequence = StereoSequence();
    ASSERT_EQ(InitializeStereo(sequence, 0), 0u);
    ASSERT_EQ(Feed(sequence.frames[1].measurement).size(), 1u);
    const std::vector<int64_t> before = tracker_.last_matches();
    // Two matched points of the last frame, and an alive point it does not match.
    std::vector<int64_t> matched;
    for (const int64_t id : before) {
        if (id >= 0 && matched.size() < 2) matched.push_back(id);
    }
    ASSERT_EQ(matched.size(), 2u);
    const auto loser = map::MapPointId{static_cast<uint32_t>(matched[0])};
    const auto removed = map::MapPointId{static_cast<uint32_t>(matched[1])};
    std::optional<map::MapPointId> winner;
    for (uint32_t p = 0; p < tracker_.replica().num_points() && !winner; ++p) {
        if (tracker_.replica().IsAlive({p}) &&
            std::find(before.begin(), before.end(), int64_t{p}) == before.end()) {
            winner = map::MapPointId{p};
        }
    }
    ASSERT_TRUE(winner.has_value());

    auto* transaction = transaction_pool_.Create();
    transaction->map_id = map::MapId{1};
    transaction->replaced_points = {{loser, *winner}};
    transaction->removed_points = {removed};
    injector_.Notify(transaction);
    Drain();

    EXPECT_FALSE(tracker_.replica().IsAlive(loser));
    EXPECT_FALSE(tracker_.replica().IsAlive(removed));
    EXPECT_TRUE(tracker_.replica().CheckConsistency());
    std::vector<int64_t> expected = before;
    for (int64_t& id : expected) {
        if (id == matched[0]) id = static_cast<int64_t>(winner->value);
        if (id == matched[1]) id = -1;
    }
    EXPECT_EQ(tracker_.last_matches(), expected);

    // The next frame tracks without failing.
    const auto captured = Feed(sequence.frames[2].measurement);
    ExpectTrackedMetric(captured, sequence.frames[2].T_cw,
                        test::MapAlignment(sequence.frames[0].T_cw), 2);
    EXPECT_EQ(tracker_.state(), TrackerState::Tracking);
}

TEST_F(TrackerMonoAccuracyTest, TracksSyntheticMono) {
    // The fixture's Mapper initializes with min_parallax_deg = 4: the error
    // bounds measure the tracker, not the initial map.
    const MonoRun run = TrackSyntheticMono(/*check_translation=*/true);
    RecordProperty("init_frame", std::to_string(run.init_frame));
    RecordProperty("max_rotation_deg", std::to_string(run.max_rotation_deg));
    RecordProperty("max_translation_percent", std::to_string(100.0 * run.max_relative_translation));
}

TEST_F(TrackerTest, TracksSyntheticMonoDefaultInit) {
    // Default initialization (~1.4 deg parallax at frame 3): the initial map's
    // depths are off by ~12 %, and so is the tracked translation; only the
    // rotation is bounded, the translation error is recorded.
    const MonoRun run = TrackSyntheticMono(/*check_translation=*/false);
    RecordProperty("init_frame", std::to_string(run.init_frame));
    RecordProperty("max_rotation_deg", std::to_string(run.max_rotation_deg));
    RecordProperty("max_translation_percent", std::to_string(100.0 * run.max_relative_translation));
}

TEST_F(TrackerTest, EmptyFrameIsLost) {
    const auto sequence = StereoSequence();
    ASSERT_EQ(InitializeStereo(sequence, 0), 0u);

    const auto captured = Feed(test::MakeBinocular(0, 0.07));

    ASSERT_EQ(captured.size(), 1u);
    EXPECT_EQ(captured[0].type, events::EventType::TrackingLost);
    EXPECT_EQ(captured[0].map_id, map::MapId{1});
    EXPECT_TRUE(captured[0].final);
    EXPECT_EQ(tracker_.state(), TrackerState::Lost);
}

TEST_F(TrackerTest, TeleportedFrameIsLost) {
    // Same scene (same seed): frame 1 of these sequences is 5 m away from, or
    // rotated 180 degrees against, frame 1 of the tracked one.
    const auto sequence = StereoSequence();
    const auto teleported = StereoSequence(/*step=*/5.0);
    const auto turned = StereoSequence(/*step=*/0.0, /*yaw_step_deg=*/180.0);
    const auto expect_no_nan = [](const std::vector<Captured>& captured) {
        for (const Captured& c : captured) {
            if (c.type == events::EventType::FrameTracked) {
                EXPECT_TRUE(c.pose.ToMatrix34().allFinite());
            }
        }
    };

    ASSERT_EQ(InitializeStereo(sequence, 0), 0u);
    for (size_t k = 1; k <= 3; ++k) {
        const auto captured = Feed(sequence.frames[k].measurement);
        ASSERT_EQ(captured.size(), 1u);
        ASSERT_EQ(captured[0].type, events::EventType::FrameTracked);
        expect_no_nan(captured);
    }
    auto captured = Feed(teleported.frames[1].measurement);
    expect_no_nan(captured);
    ASSERT_EQ(captured.size(), 1u);
    EXPECT_EQ(captured[0].type, events::EventType::TrackingLost);
    EXPECT_EQ(tracker_.state(), TrackerState::Lost);

    // Map 2 from frame 4, then a frame turned around.
    ASSERT_EQ(InitializeStereo(sequence, 4), 4u);
    captured = Feed(sequence.frames[5].measurement);
    expect_no_nan(captured);
    ASSERT_EQ(captured.size(), 1u);
    ASSERT_EQ(captured[0].type, events::EventType::FrameTracked);
    captured = Feed(turned.frames[1].measurement);
    expect_no_nan(captured);
    ASSERT_EQ(captured.size(), 1u);
    EXPECT_EQ(captured[0].type, events::EventType::TrackingLost);
    EXPECT_EQ(captured[0].map_id, map::MapId{2});
    EXPECT_EQ(tracker_.state(), TrackerState::Lost);
}

TEST_F(TrackerTest, NewMapResetsTrackingState) {
    const auto sequence = StereoSequence();
    const auto truth = TruePoses(sequence);
    ASSERT_EQ(InitializeStereo(sequence, 0), 0u);
    constexpr size_t kMap1End = 5;
    for (size_t k = 1; k <= kMap1End; ++k) {
        const auto captured = Feed(sequence.frames[k].measurement);
        ASSERT_EQ(captured.size(), 1u);
        ASSERT_EQ(captured[0].type, events::EventType::FrameTracked);
    }
    // Forced loss: an empty frame.
    auto captured =
        Feed(test::MakeBinocular(0, sequence.frames[kMap1End].measurement.timestamp() + 0.01));
    ASSERT_EQ(captured.size(), 1u);
    ASSERT_EQ(captured[0].type, events::EventType::TrackingLost);

    // Re-initialization from a later frame: map 2, with the replica and the
    // last frame of map 2 only.
    constexpr size_t kMap2Frame = 12;
    ASSERT_EQ(InitializeStereo(sequence, kMap2Frame), kMap2Frame);
    EXPECT_EQ(tracker_.current_map_id(), map::MapId{2});
    EXPECT_EQ(tracker_.replica().map_id(), map::MapId{2});
    ASSERT_EQ(tracker_.replica().keyframe_ids().size(), 1u);
    ExpectLastMatchesAreKeyFrameObservations(tracker_.replica().keyframe_ids().front());

    const test::MapAlignment alignment = AlignmentOf(truth, /*monocular=*/false);
    for (size_t k = kMap2Frame + 1; k <= kMap2Frame + 6; ++k) {
        captured = Feed(sequence.frames[k].measurement);
        ExpectTrackedMetric(captured, truth[k], alignment, k);
        ASSERT_FALSE(captured.empty());
        EXPECT_EQ(captured[0].map_id, map::MapId{2});
    }
}

TEST_F(TrackerTest, LosesTrackingOnFewFeatures) {
    const auto sequence = StereoSequence();
    ASSERT_EQ(InitializeStereo(sequence, 0), 0u);

    auto captured = Feed(test::MakeMonocular(0, 3.0));
    ASSERT_EQ(captured.size(), 1u);
    ASSERT_EQ(captured[0].type, events::EventType::TrackingLost);
    EXPECT_EQ(captured[0].map_id, map::MapId{1});
    EXPECT_DOUBLE_EQ(captured[0].timestamp, 3.0);
    EXPECT_EQ(tracker_.state(), TrackerState::Lost);

    // Lost: measurements go to the Mapper for initialization again.
    captured = Feed(test::MakeMonocular(200, 4.0));
    ASSERT_EQ(captured.size(), 1u);
    EXPECT_EQ(captured[0].type, events::EventType::InitializationMeasurement);
    EXPECT_EQ(captured[0].map_id, map::MapId{0});
    EXPECT_EQ(tracker_.state(), TrackerState::Lost);
    EXPECT_EQ(mapper_.state(), mapping::MapperState::Initializing);
}

TEST_F(TrackerTest, StateIsLostWhenTrackingLostIsPublished) {
    const auto sequence = StereoSequence();
    ASSERT_EQ(InitializeStereo(sequence, 0), 0u);
    // A synchronous observer runs on the publishing thread, inside Notify, so
    // it sees the state at the exact moment TrackingLost is published.
    struct Probe {
        TTracker* tracker;
        bool seen = false;
        TrackerState state = TrackerState::WaitingForMap;
    } probe{&tracker_};
    tracker_.AddSyncObserver(
        [](void* ctx, events::Event* event) {
            auto* p = static_cast<Probe*>(ctx);
            if (event->type == events::EventType::TrackingLost) {
                p->seen = true;
                p->state = p->tracker->state();
            }
        },
        &probe);

    Feed(test::MakeMonocular(0, 3.0));

    ASSERT_TRUE(probe.seen);
    EXPECT_EQ(probe.state, TrackerState::Lost);
}

TEST_F(TrackerTest, IgnoresStaleNewMap) {
    const auto sequence = StereoSequence();
    const auto truth = TruePoses(sequence);
    ASSERT_EQ(InitializeStereo(sequence, 0), 0u);
    Feed(test::MakeMonocular(0, 0.07));  // lost
    ASSERT_EQ(InitializeStereo(sequence, 2), 2u);
    ASSERT_EQ(tracker_.current_map_id(), map::MapId{2});
    const map::FrameId keyframe = tracker_.replica().keyframe_ids().front();

    auto* stale = transaction_pool_.Create();
    stale->map_id = map::MapId{1};
    stale->new_map = true;
    stale->start_frame = map::FrameId{0};
    injector_.Notify(stale);
    Drain();

    EXPECT_EQ(tracker_.current_map_id(), map::MapId{2});
    EXPECT_EQ(tracker_.replica().map_id(), map::MapId{2});
    EXPECT_EQ(tracker_.state(), TrackerState::Tracking);

    // The position is still reported against map 2's keyframe and world.
    const auto captured = Feed(sequence.frames[3].measurement);
    ExpectTrackedMetric(captured, truth[3], test::MapAlignment(truth[2]), 3);
    ASSERT_FALSE(captured.empty());
    EXPECT_EQ(captured[0].map_id, map::MapId{2});
    EXPECT_EQ(captured[0].frame_id, keyframe);
}

TEST_F(TrackerTest, NewMapAfterLoss) {
    const auto sequence = StereoSequence();
    ASSERT_EQ(InitializeStereo(sequence, 0), 0u);
    Feed(test::MakeMonocular(0, 0.07));
    ASSERT_EQ(tracker_.state(), TrackerState::Lost);

    ASSERT_EQ(InitializeStereo(sequence, 5), 5u);

    EXPECT_EQ(tracker_.state(), TrackerState::Tracking);
    EXPECT_EQ(tracker_.current_map_id(), map::MapId{2});
}

TEST_F(TrackerTest, ThreadProcessesTrackedMeasurements) {
    // The worker forwards measurements while waiting for a map, then tracks
    // once the (synchronously driven) Mapper has published one.
    const auto sequence = StereoSequence();
    tracker_.Start();
    tracker_.Track(sequence.frames[0].measurement);
    const auto deadline = Clock::now() + std::chrono::seconds(10);
    std::vector<Captured> captured;
    while (captured.empty() && Clock::now() < deadline) {
        captured = TakeCaptured();
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    ASSERT_EQ(captured.size(), 1u);
    EXPECT_EQ(captured[0].type, events::EventType::InitializationMeasurement);

    while (mapper_.ProcessPending()) {
    }
    while (tracker_.state() != TrackerState::Tracking && Clock::now() < deadline) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    ASSERT_EQ(tracker_.state(), TrackerState::Tracking);

    for (size_t k = 1; k <= 3; ++k) tracker_.Track(sequence.frames[k].measurement);
    captured.clear();
    while (captured.size() < 3 && Clock::now() < deadline) {
        for (const Captured& c : TakeCaptured()) captured.push_back(c);
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    tracker_.Stop();

    ASSERT_EQ(captured.size(), 3u);
    for (const Captured& c : captured) EXPECT_EQ(c.type, events::EventType::FrameTracked);
}

// Reference-Frame tracking and the loss protocol (spec sections 2, 3).

// k = 2, L = 1: two words of weight 1 under the root. With levels_up = 4
// every FeatureVector files all features under the root, so SearchByBoW
// compares every pair of features (the ratio test decides).
std::shared_ptr<const TVocabulary> MakeRootVocabulary() {
    using TDescriptors = typename Extractor::TDescriptors;
    TDescriptors descriptors = TDescriptors::Zero(Extractor::DescriptorSize, 3);
    descriptors(0, 2) = 0xFF;
    return std::make_shared<const TVocabulary>(
        TVocabulary::FromTree(2, 1, {0, 0, 0}, std::move(descriptors), {0.0, 1.0, 1.0}));
}

// `min_map_frames` is RelocalizationOptions::min_map_frames_to_relocalize; a
// fresh stereo map has one Frame.
TrackerOptions LossOptions(size_t min_map_frames) {
    TrackerOptions options;
    options.relocalization.min_map_frames_to_relocalize = min_map_frames;
    return options;
}

class TrackerBowTest : public TrackerTest {
protected:
    explicit TrackerBowTest(size_t min_map_frames = 0,
                            std::shared_ptr<const TVocabulary> vocabulary = MakeRootVocabulary())
        : TrackerTest({}, LossOptions(min_map_frames), vocabulary) {}

    // Tracks frames 1..3 of `sequence` (a velocity, a reference Frame), then
    // the frame `jumped`: frame 4's timestamp, yawed 15 degrees from where the
    // motion model puts it. Returns what the jumped frame published.
    std::vector<Captured> TrackThenJump(const test::SyntheticStereoSequence& sequence,
                                        const test::SyntheticStereoSequence& jumped) {
        EXPECT_EQ(InitializeStereo(sequence, 0), 0u);
        for (size_t k = 1; k <= 3; ++k) {
            const auto captured = Feed(sequence.frames[k].measurement);
            EXPECT_EQ(captured.size(), 1u);
            if (!captured.empty()) EXPECT_EQ(captured[0].type, events::EventType::FrameTracked);
        }
        return Feed(jumped.frames[4].measurement);
    }

    // Loses tracking with an empty frame at `timestamp`.
    std::vector<Captured> LoseAt(double timestamp) { return Feed(test::MakeBinocular(0, timestamp)); }
};

// Same scene (same seed), 4 degrees of yaw per frame instead of 0.2: frame 4
// is ~15 degrees (~100 px) from the motion model's prediction, beyond the
// zero-motion window (4 x 7 px, retried at x2).
test::SyntheticStereoSequence JumpedSequence() { return StereoSequence(0.05, 4.0); }

class TrackerNoVocabularyTest : public TrackerBowTest {
protected:
    TrackerNoVocabularyTest() : TrackerBowTest(0, nullptr) {}
};

class TrackerSmallMapTest : public TrackerBowTest {
protected:
    // A map with exactly min_map_frames_to_relocalize Frames is too small.
    TrackerSmallMapTest() : TrackerBowTest(1) {}
};

TEST(TrackerOptionsTest, RelocalizationDefaults) {
    const RelocalizationOptions options;
    EXPECT_DOUBLE_EQ(options.recently_lost_seconds, 5.0);
    EXPECT_EQ(options.min_map_frames_to_relocalize, 10u);
    EXPECT_DOUBLE_EQ(TrackingOptions{}.reference_frame_nn_ratio, 0.7);
    EXPECT_EQ(TrackingOptions{}.min_reference_frame_matches, 15u);
    EXPECT_EQ(TrackingOptions{}.min_reference_frame_inliers, 10u);
}

TEST_F(TrackerBowTest, ReferenceFrameRecoversJump) {
    const auto sequence = StereoSequence();
    const auto jumped = JumpedSequence();
    const auto captured = TrackThenJump(sequence, jumped);

    ASSERT_EQ(captured.size(), 1u);
    ASSERT_EQ(captured[0].type, events::EventType::FrameTracked);
    EXPECT_EQ(captured[0].stage, events::TrackingStage::ReferenceFrame);
    EXPECT_EQ(captured[0].map_id, map::MapId{1});
    EXPECT_GE(captured[0].num_inliers, TrackingOptions{}.min_local_map_inliers);
    EXPECT_EQ(tracker_.state(), TrackerState::Tracking);
    ExpectTrackedMetric(captured, jumped.frames[4].T_cw, test::MapAlignment(sequence.frames[0].T_cw), 4,
                        events::TrackingStage::ReferenceFrame);

    // Tracking goes on from the recovered pose.
    const auto next = Feed(jumped.frames[5].measurement);
    ASSERT_EQ(next.size(), 1u);
    EXPECT_EQ(next[0].type, events::EventType::FrameTracked);
}

TEST_F(TrackerNoVocabularyTest, NoVocabularyBehavesAsBefore) {
    // The same jump, no vocabulary: no reference-Frame tracking, no
    // RecentlyLost (although the map is large enough), straight to Lost.
    const auto captured = TrackThenJump(StereoSequence(), JumpedSequence());

    ASSERT_EQ(captured.size(), 1u);
    EXPECT_EQ(captured[0].type, events::EventType::TrackingLost);
    EXPECT_TRUE(captured[0].final);
    EXPECT_EQ(tracker_.state(), TrackerState::Lost);
    EXPECT_EQ(mapper_.state(), mapping::MapperState::Initializing);

    // Lost: measurements go to the Mapper for initialization again.
    const auto next = Feed(test::MakeBinocular(200, 1.0));
    ASSERT_EQ(next.size(), 1u);
    EXPECT_EQ(next[0].type, events::EventType::InitializationMeasurement);
}

TEST_F(TrackerBowTest, FailureEntersRecentlyLostWithVocabulary) {
    const auto sequence = StereoSequence();
    ASSERT_EQ(InitializeStereo(sequence, 0), 0u);
    const size_t num_frames = mapper_.num_keyframes();

    auto captured = LoseAt(1.0);

    ASSERT_EQ(captured.size(), 1u);
    EXPECT_EQ(captured[0].type, events::EventType::TrackingLost);
    EXPECT_FALSE(captured[0].final);
    EXPECT_EQ(captured[0].map_id, map::MapId{1});
    EXPECT_DOUBLE_EQ(captured[0].timestamp, 1.0);
    EXPECT_EQ(tracker_.state(), TrackerState::RecentlyLost);
    // The Mapper keeps its map.
    EXPECT_EQ(mapper_.state(), mapping::MapperState::Mapping);
    EXPECT_EQ(mapper_.current_map_id(), map::MapId{1});
    EXPECT_EQ(mapper_.num_keyframes(), num_frames);

    // While RecentlyLost nothing is published (no FrameTracked, no
    // InitializationMeasurement), not even for a trackable frame: only
    // relocalization (not implemented yet) leaves RecentlyLost.
    EXPECT_TRUE(LoseAt(2.0).empty());
    EXPECT_TRUE(Feed(sequence.frames[1].measurement).empty());
    EXPECT_EQ(tracker_.state(), TrackerState::RecentlyLost);
    EXPECT_EQ(mapper_.state(), mapping::MapperState::Mapping);
    EXPECT_EQ(mapper_.num_keyframes(), num_frames);
}

TEST_F(TrackerBowTest, StateIsRecentlyLostWhenTrackingLostIsPublished) {
    const auto sequence = StereoSequence();
    ASSERT_EQ(InitializeStereo(sequence, 0), 0u);
    struct Probe {
        TTracker* tracker;
        bool seen = false;
        TrackerState state = TrackerState::WaitingForMap;
    } probe{&tracker_};
    tracker_.AddSyncObserver(
        [](void* ctx, events::Event* event) {
            auto* p = static_cast<Probe*>(ctx);
            if (event->type == events::EventType::TrackingLost) {
                p->seen = true;
                p->state = p->tracker->state();
            }
        },
        &probe);

    LoseAt(1.0);

    ASSERT_TRUE(probe.seen);
    EXPECT_EQ(probe.state, TrackerState::RecentlyLost);
}

TEST_F(TrackerBowTest, RecentlyLostTimesOutToLost) {
    const auto sequence = StereoSequence();
    ASSERT_EQ(InitializeStereo(sequence, 0), 0u);
    constexpr double kLostAt = 1.0;
    ASSERT_EQ(LoseAt(kLostAt).size(), 1u);
    ASSERT_EQ(tracker_.state(), TrackerState::RecentlyLost);

    // Measurement timestamps, not wall time: just under 5 s, nothing.
    for (const double dt : {1.0, 2.5, 4.99}) {
        EXPECT_TRUE(LoseAt(kLostAt + dt).empty()) << dt;
        EXPECT_EQ(tracker_.state(), TrackerState::RecentlyLost) << dt;
    }

    auto captured = LoseAt(kLostAt + 5.0);
    ASSERT_EQ(captured.size(), 1u);
    EXPECT_EQ(captured[0].type, events::EventType::TrackingLost);
    EXPECT_TRUE(captured[0].final);
    EXPECT_EQ(captured[0].map_id, map::MapId{1});
    EXPECT_DOUBLE_EQ(captured[0].timestamp, kLostAt + 5.0);
    EXPECT_EQ(tracker_.state(), TrackerState::Lost);
    // The final loss clears the Mapper's map.
    EXPECT_EQ(mapper_.state(), mapping::MapperState::Initializing);
    EXPECT_EQ(mapper_.num_keyframes(), 0u);

    // Lost: re-initialization as before, into map 2.
    captured = Feed(test::MakeBinocular(200, kLostAt + 6.0));
    ASSERT_EQ(captured.size(), 1u);
    EXPECT_EQ(captured[0].type, events::EventType::InitializationMeasurement);
    ASSERT_EQ(InitializeStereo(sequence, 5), 5u);
    EXPECT_EQ(tracker_.current_map_id(), map::MapId{2});
}

TEST_F(TrackerSmallMapTest, SmallMapGoesStraightToLost) {
    const auto sequence = StereoSequence();
    ASSERT_EQ(InitializeStereo(sequence, 0), 0u);
    ASSERT_EQ(tracker_.replica().keyframe_ids().size(), 1u);

    const auto captured = LoseAt(1.0);

    ASSERT_EQ(captured.size(), 1u);
    EXPECT_EQ(captured[0].type, events::EventType::TrackingLost);
    EXPECT_TRUE(captured[0].final);
    EXPECT_EQ(tracker_.state(), TrackerState::Lost);
    EXPECT_EQ(mapper_.state(), mapping::MapperState::Initializing);
}

class TrackerDefaultBowTest : public TrackerBowTest {
protected:
    TrackerDefaultBowTest() : TrackerBowTest(RelocalizationOptions{}.min_map_frames_to_relocalize) {}
};

TEST_F(TrackerDefaultBowTest, SmallMapGoesStraightToLostByDefault) {
    // Default options: a fresh stereo map (1 Frame) is far below 10 Frames.
    const auto sequence = StereoSequence();
    ASSERT_EQ(InitializeStereo(sequence, 0), 0u);
    const auto captured = LoseAt(1.0);
    ASSERT_EQ(captured.size(), 1u);
    EXPECT_TRUE(captured[0].final);
    EXPECT_EQ(tracker_.state(), TrackerState::Lost);
}

} // namespace
} // namespace voyahslam::tracking
