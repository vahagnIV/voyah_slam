#include <gtest/gtest.h>
#include "tracking/tracker_test_fixture.h"
#include <support/synthetic_sequence.h>
#include <support/test_measurements.h>
#include <support/trajectory_error.h>
#include <voyahslam/errors.h>
#include <voyahslam/events/async_observer.h>
#include <voyahslam/events/events.h>
#include <voyahslam/events/observable.h>
#include <voyahslam/features/stereo_matcher.h>
#include <voyahslam/map/absorbed_point_ids.h>
#include <voyahslam/mapping/mapper.h>
#include <voyahslam/tracking/tracker.h>

#include <chrono>
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <numbers>
#include <optional>
#include <string>
#include <string_view>
#include <thread>
#include <unordered_map>
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

    const events::EventPtr event = listener_.GetNextEvent();
    ASSERT_NE(event, nullptr);
    ASSERT_EQ(event->type, events::EventType::InitializationMeasurement);
    const auto& init = static_cast<const TInitializationMeasurement&>(*event);
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
    const MapReplica<Extractor>& replica = TrackerTestAccess::MapReplica(tracker_);
    EXPECT_EQ(replica.map_id(), map::MapId{1});
    EXPECT_EQ(replica.num_points(), mapper_.num_map_points());
    EXPECT_GE(replica.num_points(), 100u);
    ASSERT_EQ(replica.keyframe_ids().size(), 1u);
    ExpectLastMatchesAreKeyFrameObservations(replica.keyframe_ids().front());
}

TEST_F(TrackerTest, TracksAndPublishesFrameTracked) {
    const auto sequence = StereoSequence();
    ASSERT_EQ(InitializeStereo(sequence, 0), 0u);
    const map::FrameId keyframe = TrackerTestAccess::MapReplica(tracker_).keyframe_ids().front();

    const auto captured = Feed(sequence.frames[1].measurement);

    ASSERT_EQ(captured.size(), 1u);
    ASSERT_EQ(captured[0].type, events::EventType::FrameTracked);
    EXPECT_EQ(captured[0].map_id, map::MapId{1});
    ASSERT_TRUE(captured[0].has_reference);
    EXPECT_EQ(captured[0].frame_id, keyframe);  // the new map's start Frame
    EXPECT_GE(captured[0].num_inliers, TrackingOptions{}.min_local_map_inliers);
    // The published observations are the last frame's matches.
    size_t num_last_matches = 0;
    for (const int64_t id : TrackerTestAccess::LastMatches(tracker_)) num_last_matches += id >= 0 ? 1 : 0;
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
    std::vector<map::FrameId> frames = TrackerTestAccess::MapReplica(tracker_).keyframe_ids();
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
    const std::vector<int64_t> before = TrackerTestAccess::LastMatches(tracker_);
    // Two unmatched features of the last frame.
    std::vector<uint32_t> unmatched;
    for (size_t i = 0; i < before.size() && unmatched.size() < 2; ++i) {
        if (before[i] < 0) unmatched.push_back(static_cast<uint32_t>(i));
    }
    ASSERT_EQ(unmatched.size(), 2u);
    const auto new_id = static_cast<uint32_t>(TrackerTestAccess::MapReplica(tracker_).num_points());

    // A Frame of the last frame's measurement: feature unmatched[0] observes
    // a new point, unmatched[1] an existing one.
    const auto inject = [&](uint32_t frame, const auto& measurement) {
        auto created = std::make_shared<events::MapTransaction<Extractor>>();
        created->map_id = map::MapId{1};
        created->active_map = map::MapId{1};  // as the Mapper
        auto& added = created->added_frames.emplace_back();
        added.id = map::FrameId{frame};
        added.T_cw = sequence.frames[1].T_cw;
        added.measurement = test::Share(measurement);
        added.observed_features = unmatched;
        added.observed_points = {map::MapPointId{new_id}, map::MapPointId{0}};
        created->new_points_start_id = map::MapPointId{new_id};
        created->ResizeNewPoints(1);
        created->new_positions.col(0) = geometry::TPoint3DF(0, 0, 5);
        created->new_descriptors.col(0).setZero();
        created->new_normals.col(0) = geometry::TPoint3DF(0, 0, 1);
        injector_.Notify(std::move(created));
        Drain();
    };

    // Another measurement's Frame: not merged.
    inject(900, sequence.frames[2].measurement);
    EXPECT_EQ(TrackerTestAccess::LastMatches(tracker_), before);

    inject(901, sequence.frames[1].measurement);
    std::vector<int64_t> expected = before;
    expected[unmatched[0]] = new_id;
    expected[unmatched[1]] = 0;
    EXPECT_EQ(TrackerTestAccess::LastMatches(tracker_), expected);
}

TEST_F(TrackerTest, RemovedAndReplacedPointsInLastFrame) {
    const auto sequence = StereoSequence();
    ASSERT_EQ(InitializeStereo(sequence, 0), 0u);
    ASSERT_EQ(Feed(sequence.frames[1].measurement).size(), 1u);
    const std::vector<int64_t> before = TrackerTestAccess::LastMatches(tracker_);
    // Two matched points of the last frame, and an alive point it does not match.
    std::vector<int64_t> matched;
    for (const int64_t id : before) {
        if (id >= 0 && matched.size() < 2) matched.push_back(id);
    }
    ASSERT_EQ(matched.size(), 2u);
    const auto loser = map::MapPointId{static_cast<uint32_t>(matched[0])};
    const auto removed = map::MapPointId{static_cast<uint32_t>(matched[1])};
    std::optional<map::MapPointId> winner;
    for (uint32_t p = 0; p < TrackerTestAccess::MapReplica(tracker_).num_points() && !winner; ++p) {
        if (TrackerTestAccess::MapReplica(tracker_).IsAlive({p}) &&
            std::find(before.begin(), before.end(), int64_t{p}) == before.end()) {
            winner = map::MapPointId{p};
        }
    }
    ASSERT_TRUE(winner.has_value());

    auto transaction = std::make_shared<events::MapTransaction<Extractor>>();
    transaction->map_id = map::MapId{1};
    transaction->active_map = map::MapId{1};  // as the Mapper
    transaction->replaced_points = {{loser, *winner}};
    transaction->removed_points = {removed};
    injector_.Notify(std::move(transaction));
    Drain();

    EXPECT_FALSE(TrackerTestAccess::MapReplica(tracker_).IsAlive(loser));
    EXPECT_FALSE(TrackerTestAccess::MapReplica(tracker_).IsAlive(removed));
    EXPECT_TRUE(TrackerTestAccess::MapReplica(tracker_).CheckConsistency());
    std::vector<int64_t> expected = before;
    for (int64_t& id : expected) {
        if (id == matched[0]) id = static_cast<int64_t>(winner->value);
        if (id == matched[1]) id = -1;
    }
    EXPECT_EQ(TrackerTestAccess::LastMatches(tracker_), expected);

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
    EXPECT_EQ(TrackerTestAccess::MapReplica(tracker_).map_id(), map::MapId{2});
    ASSERT_EQ(TrackerTestAccess::MapReplica(tracker_).keyframe_ids().size(), 1u);
    ExpectLastMatchesAreKeyFrameObservations(TrackerTestAccess::MapReplica(tracker_).keyframe_ids().front());

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
    const events::Subscription probe_subscription = tracker_.AddSyncObserver(
        [](void* ctx, const events::Event& event) {
            auto* p = static_cast<Probe*>(ctx);
            if (event.type == events::EventType::TrackingLost) {
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
    const map::FrameId keyframe = TrackerTestAccess::MapReplica(tracker_).keyframe_ids().front();

    auto stale = std::make_shared<events::MapTransaction<Extractor>>();
    stale->map_id = map::MapId{1};
    stale->new_map = true;
    stale->start_frame = map::FrameId{0};
    injector_.Notify(std::move(stale));
    Drain();

    EXPECT_EQ(tracker_.current_map_id(), map::MapId{2});
    EXPECT_EQ(TrackerTestAccess::MapReplica(tracker_).map_id(), map::MapId{2});
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

TEST_F(TrackerTest, TrackBeforeStartQueues) {
    ASSERT_EQ(tracker_.run_state(), events::RunState::Created);
    EXPECT_EQ(tracker_.pending_measurements(), 0u);
    tracker_.Track(test::MakeMonocular(200, 0.0));
    EXPECT_EQ(tracker_.pending_measurements(), 1u);
    EXPECT_TRUE(tracker_.ProcessPending());  // the queued measurement is tracked once driven
    EXPECT_EQ(tracker_.pending_measurements(), 0u);
}

TEST_F(TrackerTest, TrackAfterStopThrows) {
    tracker_.Start();
    tracker_.Stop();
    EXPECT_THROW(tracker_.Track(test::MakeMonocular(200, 0.0)), UsageError);
    EXPECT_THROW(tracker_.Track(test::MakeSyntheticStereoFrame().measurement), UsageError);
    EXPECT_EQ(tracker_.pending_measurements(), 0u);
    EXPECT_THROW(tracker_.Start(), UsageError);  // Stopped is terminal
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
    // InitializationMeasurement), not even for a frame that could start a new
    // map (another scene): only relocalization leaves RecentlyLost.
    EXPECT_TRUE(LoseAt(2.0).empty());
    test::SyntheticStereoSequenceOptions elsewhere;
    elsewhere.trajectory.num_points = 600;
    elsewhere.trajectory.seed = 2;
    EXPECT_TRUE(Feed(test::MakeSyntheticStereoSequence(elsewhere).frames[1].measurement).empty());
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
    const events::Subscription probe_subscription = tracker_.AddSyncObserver(
        [](void* ctx, const events::Event& event) {
            auto* p = static_cast<Probe*>(ctx);
            if (event.type == events::EventType::TrackingLost) {
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
    ASSERT_EQ(TrackerTestAccess::MapReplica(tracker_).keyframe_ids().size(), 1u);

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

// Loop corrections (loop-closing spec section 3, Tracker).

// The root vocabulary and relocalization on any map, so RecentlyLost and
// relocalization are reachable on the small synthetic stereo map.
TrackerOptions LoopOptions() {
    TrackerOptions options;
    options.relocalization.min_map_frames_to_relocalize = 0;
    return options;
}

// The Mapper's map does not see the injected corrections: each test checks
// the Tracker's first frame after one, which is tracked before any Mapper
// transaction about it.
class TrackerLoopTest : public TrackerTest {
protected:
    TrackerLoopTest() : TrackerTest({}, LoopOptions(), MakeRootVocabulary()) {}

    // 15 degrees of yaw and 0.4 m: ~100 px at fx = 380, beyond the
    // zero-motion window (4 x 7 px, retried at x2).
    static geometry::RigidTransformation WorldMove() {
        const double angle = 15.0 * std::numbers::pi / 180.0;
        const geometry::TMatrix33F rotation =
            Eigen::AngleAxis<precision_t>(angle, geometry::TVector3DF::UnitY()).toRotationMatrix();
        return {rotation, geometry::TVector3DF(0.3, -0.1, 0.25)};
    }

    // A loop correction of the replica, as the Mapper publishes one: the world
    // moves by `S` (p_new = S p_old), so every Frame's T_cw becomes
    // T_cw * S^-1 and every alive point S p, its normal rotated; the epoch is
    // `epoch`. Frames in `removed_frames` are culled in the same transaction.
    void InjectCorrection(const geometry::RigidTransformation& S, uint32_t epoch,
                          const std::vector<map::FrameId>& removed_frames = {}) {
        const MapReplica<Extractor>& replica = TrackerTestAccess::MapReplica(tracker_);
        auto correction = std::make_shared<events::MapTransaction<Extractor>>();
        correction->map_id = replica.map_id();
        correction->active_map = replica.map_id();  // as the Mapper
        correction->map_epoch = epoch;
        for (const map::FrameId id : replica.keyframe_ids()) {
            correction->updated_poses.emplace_back(id, replica.keyframe(id)->T_cw * S.Inverse());
        }
        std::vector<map::MapPointId> alive;
        for (uint32_t p = 0; p < replica.num_points(); ++p) {
            if (replica.IsAlive(map::MapPointId{p})) alive.push_back(map::MapPointId{p});
        }
        correction->ResizeUpdatedPoints(static_cast<uint32_t>(alive.size()));
        for (size_t i = 0; i < alive.size(); ++i) {
            const auto col = static_cast<Eigen::Index>(i);
            correction->updated_ids[i] = alive[i];
            correction->updated_positions.col(col) = S.Transform(replica.Position(alive[i]));
            correction->updated_normals.col(col) = S.rotation() * replica.Normal(alive[i]);
            correction->updated_descriptors.col(col) = replica.Descriptor(alive[i]);
        }
        correction->removed_frames = removed_frames;
        injector_.Notify(std::move(correction));
        Drain();
        ASSERT_EQ(TrackerTestAccess::MapReplica(tracker_).map_epoch(), epoch);
        ASSERT_TRUE(TrackerTestAccess::MapReplica(tracker_).CheckConsistency());
    }

    // Initializes on frame 0 and tracks frames 1..last; returns the last
    // FrameTracked.
    Captured TrackUpTo(const test::SyntheticStereoSequence& sequence, size_t last) {
        EXPECT_EQ(InitializeStereo(sequence, 0), 0u);
        Captured tracked{};
        for (size_t k = 1; k <= last; ++k) {
            const auto captured = Feed(sequence.frames[k].measurement);
            EXPECT_EQ(captured.size(), 1u) << "frame " << k;
            if (captured.size() == 1) {
                EXPECT_EQ(captured[0].type, events::EventType::FrameTracked) << "frame " << k;
                tracked = captured[0];
            }
        }
        return tracked;
    }

    // The corrected map's alignment to ground truth: its world is S applied
    // to the camera of frame 0.
    static test::MapAlignment CorrectedAlignment(const test::SyntheticStereoSequence& sequence,
                                                 const geometry::RigidTransformation& S) {
        return test::MapAlignment(S * sequence.frames[0].T_cw);
    }
};

TEST_F(TrackerLoopTest, LastFrameFollowsReferenceAfterCorrection) {
    const auto sequence = StereoSequence();
    TrackUpTo(sequence, 3);
    const geometry::RigidTransformation S = WorldMove();
    InjectCorrection(S, 1);

    // The motion model predicts from the shifted last frame: the next frame
    // is tracked at once, in the corrected world.
    const auto captured = Feed(sequence.frames[4].measurement);
    ExpectTrackedMetric(captured, sequence.frames[4].T_cw, CorrectedAlignment(sequence, S), 4,
                        events::TrackingStage::MotionModel);
    EXPECT_EQ(tracker_.state(), TrackerState::Tracking);
}

TEST_F(TrackerLoopTest, FollowsAnotherFrameWhenReferenceCulled) {
    // The reference Frame is culled by the correction: the last frame follows
    // another Frame observing its points.
    const auto sequence = StereoSequence();
    size_t last = 1;
    Captured tracked = TrackUpTo(sequence, last);
    while (TrackerTestAccess::MapReplica(tracker_).num_keyframes() < 2 && last + 1 < sequence.frames.size()) {
        const auto captured = Feed(sequence.frames[++last].measurement);
        ASSERT_EQ(captured.size(), 1u) << "frame " << last;
        ASSERT_EQ(captured[0].type, events::EventType::FrameTracked) << "frame " << last;
        tracked = captured[0];
    }
    ASSERT_GE(TrackerTestAccess::MapReplica(tracker_).num_keyframes(), 2u);
    ASSERT_TRUE(tracked.has_reference);
    const geometry::RigidTransformation S = WorldMove();
    InjectCorrection(S, 1, {tracked.frame_id});
    ASSERT_EQ(TrackerTestAccess::MapReplica(tracker_).keyframe(tracked.frame_id), nullptr);

    const size_t next = last + 1;
    const auto captured = Feed(sequence.frames[next].measurement);
    ExpectTrackedMetric(captured, sequence.frames[next].T_cw, CorrectedAlignment(sequence, S), next,
                        events::TrackingStage::MotionModel);
}

TEST_F(TrackerLoopTest, FrameTrackedCarriesEpoch) {
    const auto sequence = StereoSequence();
    EXPECT_EQ(TrackUpTo(sequence, 1).map_epoch, 0u);
    InjectCorrection(geometry::RigidTransformation::Identity(), 1);

    const auto captured = Feed(sequence.frames[2].measurement);
    ASSERT_EQ(captured.size(), 1u);
    ASSERT_EQ(captured[0].type, events::EventType::FrameTracked);
    EXPECT_EQ(captured[0].map_epoch, 1u);
}

TEST_F(TrackerLoopTest, CorrectionWhileRecentlyLost) {
    // Review Focus 5: no last frame to shift; relocalization finds the next
    // frame in the corrected replica.
    const auto sequence = StereoSequence();
    TrackUpTo(sequence, 3);
    const auto lost = Feed(test::MakeBinocular(0, 4 * kFrameInterval));
    ASSERT_EQ(lost.size(), 1u);
    ASSERT_EQ(lost[0].type, events::EventType::TrackingLost);
    ASSERT_EQ(tracker_.state(), TrackerState::RecentlyLost);
    const geometry::RigidTransformation S = WorldMove();
    InjectCorrection(S, 1);
    EXPECT_EQ(tracker_.state(), TrackerState::RecentlyLost);
    EXPECT_TRUE(TrackerTestAccess::LastMatches(tracker_).empty());

    const auto captured = Feed(sequence.frames[5].measurement);
    ExpectTrackedMetric(captured, sequence.frames[5].T_cw, CorrectedAlignment(sequence, S), 5,
                        events::TrackingStage::Relocalization);
    EXPECT_EQ(captured[0].map_epoch, 1u);
    EXPECT_EQ(tracker_.state(), TrackerState::Tracking);
}

// Map merging (map-merging spec section 4, Tracker).

constexpr map::MapId kAbsorbedMap{1};    // the Mapper's map, tracked
constexpr map::MapId kTargetMap{5};      // injected; the Mapper never reaches this id here
constexpr map::FrameId kTargetFrame{1000};

// The Tracker tracks the Mapper's map 1 (M). A target map N is injected as an
// inactive map, then the merge of M into N, as the Mapper publishes it
// (mapper_merge.cpp): one transaction of N at N's epoch + 1, absorbed_map M,
// removed_maps {M}, active_map N. N is the start Frame seen again: Frame
// kTargetFrame (the start Frame's measurement) observing copies of the start
// Frame's points, in a world moved by S (p_N = S p_M). The merge moves every
// Frame and point of M by S, renumbers M's points by the id rule
// (map::AbsorbedPointIds), fuses M's points of even id that have a copy into
// it, and removes one point of the last frame. The Mapper does not see the
// injected maps: each test checks the Tracker's first frame after the merge.
class TrackerMergeTest : public TrackerLoopTest {
protected:
    // Applies the injected transactions: both components, or the Tracker only
    // (the Mapper has not read what the Tracker published).
    void Settle() {
        if (!tracker_only_) {
            Drain();
            return;
        }
        while (tracker_.ProcessPending()) {
        }
    }

    // Injects N: a new map that leaves M active.
    void InjectTargetMap(const geometry::RigidTransformation& S) {
        const MapReplica<Extractor>& absorbed = TrackerTestAccess::MapReplica(tracker_);
        ASSERT_EQ(absorbed.map_id(), kAbsorbedMap);
        const auto* start = absorbed.keyframe(absorbed.keyframe_ids().front());
        ASSERT_NE(start, nullptr);
        auto target = std::make_shared<events::MapTransaction<Extractor>>();
        target->map_id = kTargetMap;
        target->new_map = true;
        target->start_frame = kTargetFrame;
        target->active_map = kAbsorbedMap;  // N is not active
        events::NewFrame<Extractor> frame;
        frame.id = kTargetFrame;
        frame.T_cw = start->T_cw * S.Inverse();
        frame.measurement = start->shared_measurement;
        target->ResizeNewPoints(static_cast<uint32_t>(start->observations.size()));
        for (size_t i = 0; i < start->observations.size(); ++i) {
            const auto& [feature, point] = start->observations[i];
            const auto col = static_cast<Eigen::Index>(i);
            target->new_positions.col(col) = S.Transform(absorbed.Position(point));
            target->new_normals.col(col) = S.rotation() * absorbed.Normal(point);
            target->new_descriptors.col(col) = absorbed.Descriptor(point);
            frame.observed_features.push_back(feature);
            frame.observed_points.push_back(map::MapPointId{static_cast<uint32_t>(i)});
            copies_[point.value] = map::MapPointId{static_cast<uint32_t>(i)};
        }
        target->added_frames.push_back(std::move(frame));
        injector_.Notify(std::move(target));
        Settle();
        ASSERT_EQ(TrackerTestAccess::Atlas(tracker_).map_ids(), (std::vector<map::MapId>{kAbsorbedMap, kTargetMap}));
        ASSERT_EQ(TrackerTestAccess::Atlas(tracker_).active_map(), kAbsorbedMap);
        ASSERT_EQ(tracker_.current_map_id(), kAbsorbedMap);
        ASSERT_TRUE(TrackerTestAccess::Atlas(tracker_).map(kTargetMap)->CheckConsistency());
    }

    // Injects the merge of M into N; `removed` (an M id) is removed by it.
    // Fills expected_: per M id, its id in N (nullopt if gone). `active` is
    // the transaction's active_map: N, as the Mapper sends it.
    void InjectMerge(const geometry::RigidTransformation& S, std::optional<map::MapPointId> removed,
                     std::optional<map::MapId> active = kTargetMap) {
        const MapReplica<Extractor>& absorbed = *TrackerTestAccess::Atlas(tracker_).map(kAbsorbedMap);
        const MapReplica<Extractor>& target = *TrackerTestAccess::Atlas(tracker_).map(kTargetMap);
        const uint32_t target_epoch = target.map_epoch();
        const std::vector<std::optional<map::MapPointId>> new_ids = map::AbsorbedPointIds(
            static_cast<uint32_t>(absorbed.num_points()), map::MapPointId{static_cast<uint32_t>(target.num_points())},
            [&](map::MapPointId point) { return absorbed.IsAlive(point); });
        auto merge = std::make_shared<events::MapTransaction<Extractor>>();
        merge->map_id = kTargetMap;
        merge->absorbed_map = kAbsorbedMap;
        merge->removed_maps.push_back(kAbsorbedMap);
        merge->active_map = active;
        merge->map_epoch = target_epoch + 1;
        for (const map::FrameId id : absorbed.keyframe_ids()) {
            merge->updated_poses.emplace_back(id, absorbed.keyframe(id)->T_cw * S.Inverse());
        }
        std::vector<map::MapPointId> alive;
        for (uint32_t p = 0; p < absorbed.num_points(); ++p) {
            if (absorbed.IsAlive(map::MapPointId{p})) alive.push_back(map::MapPointId{p});
        }
        merge->ResizeUpdatedPoints(static_cast<uint32_t>(alive.size()));
        expected_.assign(absorbed.num_points(), std::nullopt);
        for (size_t i = 0; i < alive.size(); ++i) {
            const map::MapPointId old = alive[i];
            const map::MapPointId id = *new_ids[old.value];
            const auto col = static_cast<Eigen::Index>(i);
            merge->updated_ids[i] = id;
            merge->updated_positions.col(col) = S.Transform(absorbed.Position(old));
            merge->updated_normals.col(col) = S.rotation() * absorbed.Normal(old);
            merge->updated_descriptors.col(col) = absorbed.Descriptor(old);
            expected_[old.value] = id;
            if (removed && old == *removed) {
                merge->removed_points.push_back(id);
                expected_[old.value].reset();
                continue;
            }
            if (!IsFused(old.value)) continue;
            const auto copy = copies_.find(old.value);
            // Fused into its copy: the copy takes the point's observations.
            merge->replaced_points.push_back({id, copy->second});
            for (const map::PointObservation& observation : absorbed.covisibility().Observations(old)) {
                merge->added_observations.push_back({observation.frame, observation.feature, copy->second});
            }
            expected_[old.value] = copy->second;
        }
        injector_.Notify(std::move(merge));
        Settle();
        ASSERT_EQ(TrackerTestAccess::Atlas(tracker_).map_ids(), (std::vector<map::MapId>{kTargetMap}));
        ASSERT_EQ(TrackerTestAccess::Atlas(tracker_).active_map(), active);
        ASSERT_EQ(TrackerTestAccess::Atlas(tracker_).map(kTargetMap)->map_epoch(), target_epoch + 1);
        ASSERT_TRUE(TrackerTestAccess::Atlas(tracker_).map(kTargetMap)->CheckConsistency());
    }

    // Tracks frames 0..last in M, injects N and merges M into it.
    void TrackThenMerge(const test::SyntheticStereoSequence& sequence, size_t last,
                        const geometry::RigidTransformation& S) {
        TrackUpTo(sequence, last);
        InjectTargetMap(S);
        ASSERT_FALSE(HasFatalFailure());
        InjectMerge(S, FirstUnfusedMatch());
    }

    // Whether the merge fuses M's point `old` into its copy.
    [[nodiscard]] bool IsFused(uint32_t old) const { return copies_.contains(old) && old % 2 == 0; }

    // The first point of the last frame the merge does not fuse; none if
    // there is none.
    [[nodiscard]] std::optional<map::MapPointId> FirstUnfusedMatch() const {
        for (const int64_t match : TrackerTestAccess::LastMatches(tracker_)) {
            if (match >= 0 && !IsFused(static_cast<uint32_t>(match))) {
                return map::MapPointId{static_cast<uint32_t>(match)};
            }
        }
        return std::nullopt;
    }

    bool tracker_only_ = false;
    std::unordered_map<uint32_t, map::MapPointId> copies_;  // M id -> its copy in N
    std::vector<std::optional<map::MapPointId>> expected_;  // M id -> N id after the merge
};

TEST_F(TrackerMergeTest, TracksAcrossMerge) {
    // The motion model predicts from the shifted last frame: the next frame
    // is tracked at once, in N's world, with no loss.
    const auto sequence = StereoSequence();
    const geometry::RigidTransformation S = WorldMove();
    TrackThenMerge(sequence, 3, S);
    ASSERT_FALSE(HasFatalFailure());
    EXPECT_EQ(tracker_.state(), TrackerState::Tracking);
    EXPECT_EQ(tracker_.current_map_id(), kTargetMap);
    EXPECT_TRUE(TakeCaptured().empty());  // no loss announced

    const auto captured = Feed(sequence.frames[4].measurement);
    ExpectTrackedMetric(captured, sequence.frames[4].T_cw, CorrectedAlignment(sequence, S), 4,
                        events::TrackingStage::MotionModel);
    EXPECT_EQ(tracker_.state(), TrackerState::Tracking);
}

TEST_F(TrackerMergeTest, TranslatesPointReferences) {
    // Every point of the last frame takes its id in N: renumbered by the id
    // rule, or its fusion winner; the removed one becomes unmatched.
    const auto sequence = StereoSequence();
    const geometry::RigidTransformation S = WorldMove();
    TrackUpTo(sequence, 3);
    const std::vector<int64_t> before = TrackerTestAccess::LastMatches(tracker_);
    InjectTargetMap(S);
    ASSERT_FALSE(HasFatalFailure());
    const std::optional<map::MapPointId> removed = FirstUnfusedMatch();
    ASSERT_TRUE(removed.has_value());
    InjectMerge(S, removed);
    ASSERT_FALSE(HasFatalFailure());

    const std::vector<int64_t>& after = TrackerTestAccess::LastMatches(tracker_);
    ASSERT_EQ(after.size(), before.size());
    size_t renumbered = 0;
    size_t fused = 0;
    size_t unmatched = 0;
    for (size_t feature = 0; feature < before.size(); ++feature) {
        if (before[feature] < 0) {
            EXPECT_EQ(after[feature], -1) << "feature " << feature;
            continue;
        }
        const auto old = static_cast<uint32_t>(before[feature]);
        ASSERT_LT(old, expected_.size());
        const std::optional<map::MapPointId>& expected = expected_[old];
        EXPECT_EQ(after[feature], expected ? static_cast<int64_t>(expected->value) : -1) << "feature " << feature;
        if (!expected) {
            ++unmatched;
        } else if (IsFused(old)) {
            ++fused;
        } else {
            ++renumbered;
            EXPECT_GE(expected->value, copies_.size());  // after N's own points
        }
        if (expected) EXPECT_TRUE(TrackerTestAccess::MapReplica(tracker_).IsAlive(*expected)) << "feature " << feature;
    }
    EXPECT_EQ(unmatched, 1u);
    EXPECT_GT(fused, 0u);
    EXPECT_GT(renumbered, 0u);
}

TEST_F(TrackerMergeTest, FrameTrackedCarriesTargetMap) {
    // M's epoch is 2 before the merge, N's 0: the next FrameTracked carries N
    // and N's epoch after the merge, 1, and N's point ids.
    const auto sequence = StereoSequence();
    TrackUpTo(sequence, 3);
    InjectCorrection(geometry::RigidTransformation::Identity(), 2);
    const geometry::RigidTransformation S = WorldMove();
    InjectTargetMap(S);
    ASSERT_FALSE(HasFatalFailure());
    InjectMerge(S, FirstUnfusedMatch());
    ASSERT_FALSE(HasFatalFailure());

    const auto captured = Feed(sequence.frames[4].measurement);
    ASSERT_EQ(captured.size(), 1u);
    ASSERT_EQ(captured[0].type, events::EventType::FrameTracked);
    EXPECT_EQ(captured[0].map_id, kTargetMap);
    EXPECT_EQ(captured[0].map_epoch, 1u);
    ASSERT_FALSE(captured[0].observed_points.empty());
    for (const map::MapPointId point : captured[0].observed_points) {
        EXPECT_TRUE(TrackerTestAccess::MapReplica(tracker_).IsAlive(point)) << point.value;
    }
    for (const map::FrameId frame : captured[0].local_frames) {
        EXPECT_NE(TrackerTestAccess::MapReplica(tracker_).keyframe(frame), nullptr) << frame.value;
    }
}

TEST_F(TrackerMergeTest, MergeWhileRecentlyLost) {
    // Review Focus 4: no last frame to shift; the Tracker switches to N, stays
    // RecentlyLost, and relocalizes the next frame in N.
    const auto sequence = StereoSequence();
    TrackUpTo(sequence, 3);
    const auto lost = Feed(test::MakeBinocular(0, 4 * kFrameInterval));
    ASSERT_EQ(lost.size(), 1u);
    ASSERT_EQ(lost[0].type, events::EventType::TrackingLost);
    ASSERT_EQ(tracker_.state(), TrackerState::RecentlyLost);
    const geometry::RigidTransformation S = WorldMove();
    InjectTargetMap(S);
    ASSERT_FALSE(HasFatalFailure());
    InjectMerge(S, std::nullopt);
    ASSERT_FALSE(HasFatalFailure());
    EXPECT_EQ(tracker_.state(), TrackerState::RecentlyLost);
    EXPECT_EQ(tracker_.current_map_id(), kTargetMap);
    EXPECT_TRUE(TrackerTestAccess::LastMatches(tracker_).empty());
    EXPECT_TRUE(TakeCaptured().empty());  // no loss announced

    const auto captured = Feed(sequence.frames[5].measurement);
    ExpectTrackedMetric(captured, sequence.frames[5].T_cw, CorrectedAlignment(sequence, S), 5,
                        events::TrackingStage::Relocalization);
    EXPECT_EQ(captured[0].map_id, kTargetMap);
    EXPECT_EQ(captured[0].map_epoch, 1u);
    for (const map::MapPointId point : captured[0].observed_points) {
        EXPECT_TRUE(TrackerTestAccess::MapReplica(tracker_).IsAlive(point)) << point.value;
    }
    EXPECT_EQ(tracker_.state(), TrackerState::Tracking);
}

TEST_F(TrackerMergeTest, LostTrackerReannouncesLossAfterMerge) {
    // The merge of M overtook the Tracker's final TrackingLost(M) in the
    // Mapper, which then ignores that loss (M is removed): the Tracker, Lost,
    // announces its final loss again for the map it now follows.
    const auto sequence = StereoSequence();
    TrackUpTo(sequence, 3);
    constexpr double kLostAt = 4 * kFrameInterval;
    ASSERT_EQ(Feed(test::MakeBinocular(0, kLostAt)).size(), 1u);
    ASSERT_EQ(tracker_.state(), TrackerState::RecentlyLost);
    // The Mapper does not read the final loss before the merge.
    tracker_only_ = true;
    const double final_at = kLostAt + RelocalizationOptions{}.recently_lost_seconds;
    tracker_.Track(test::MakeBinocular(0, final_at));
    Settle();
    const auto lost = TakeCaptured();
    ASSERT_EQ(lost.size(), 1u);
    ASSERT_EQ(lost[0].type, events::EventType::TrackingLost);
    ASSERT_TRUE(lost[0].final);
    ASSERT_EQ(lost[0].map_id, kAbsorbedMap);
    ASSERT_EQ(tracker_.state(), TrackerState::Lost);

    const geometry::RigidTransformation S = WorldMove();
    InjectTargetMap(S);
    ASSERT_FALSE(HasFatalFailure());
    EXPECT_TRUE(TakeCaptured().empty());  // not a merge
    InjectMerge(S, std::nullopt);
    ASSERT_FALSE(HasFatalFailure());

    const auto captured = TakeCaptured();
    ASSERT_EQ(captured.size(), 1u);
    EXPECT_EQ(captured[0].type, events::EventType::TrackingLost);
    EXPECT_TRUE(captured[0].final);
    EXPECT_EQ(captured[0].map_id, kTargetMap);
    EXPECT_DOUBLE_EQ(captured[0].timestamp, final_at);
    EXPECT_EQ(tracker_.state(), TrackerState::Lost);
    EXPECT_EQ(tracker_.current_map_id(), kTargetMap);
}

TEST_F(TrackerMergeTest, MergeLeavingNoActiveMapResets) {
    // A merge of M whose active_map is not N (here none; the Mapper never
    // sends this shape): nothing is followed, and nothing tracked against M
    // is kept: the last frame's M ids would index into N.
    const auto sequence = StereoSequence();
    TrackUpTo(sequence, 3);
    ASSERT_FALSE(TrackerTestAccess::LastMatches(tracker_).empty());
    const geometry::RigidTransformation S = WorldMove();
    InjectTargetMap(S);
    ASSERT_FALSE(HasFatalFailure());
    InjectMerge(S, std::nullopt, /*active=*/std::nullopt);
    ASSERT_FALSE(HasFatalFailure());
    EXPECT_TRUE(TrackerTestAccess::LastMatches(tracker_).empty());
    EXPECT_NE(tracker_.current_map_id(), kAbsorbedMap);  // M is gone
    EXPECT_TRUE(TakeCaptured().empty());
}

TEST_F(TrackerMergeTest, MergeOfAnotherMapLeavesTrackerAlone) {
    // The inactive N is absorbed into the active M (the reverse direction,
    // which the Tracker does not follow: M stays active and keeps its ids and
    // world). The Tracker keeps its state, map and last frame, and tracks the
    // next frame by the motion model in M's world, as before.
    const auto sequence = StereoSequence();
    TrackUpTo(sequence, 3);
    const std::vector<int64_t> before = TrackerTestAccess::LastMatches(tracker_);
    ASSERT_FALSE(before.empty());
    const geometry::RigidTransformation S = WorldMove();
    InjectTargetMap(S);
    ASSERT_FALSE(HasFatalFailure());

    const MapReplica<Extractor>& active = *TrackerTestAccess::Atlas(tracker_).map(kAbsorbedMap);
    const MapReplica<Extractor>& absorbed = *TrackerTestAccess::Atlas(tracker_).map(kTargetMap);
    const uint32_t active_epoch = active.map_epoch();
    const auto new_ids = map::AbsorbedPointIds(
        static_cast<uint32_t>(absorbed.num_points()), map::MapPointId{static_cast<uint32_t>(active.num_points())},
        [&](map::MapPointId point) { return absorbed.IsAlive(point); });
    auto merge = std::make_shared<events::MapTransaction<Extractor>>();
    merge->map_id = kAbsorbedMap;
    merge->absorbed_map = kTargetMap;
    merge->removed_maps.push_back(kTargetMap);
    merge->active_map = kAbsorbedMap;
    merge->map_epoch = active_epoch + 1;
    // N's world into M's (p_M = S^-1 p_N); M's own Frames and points stay.
    for (const map::FrameId id : absorbed.keyframe_ids()) {
        merge->updated_poses.emplace_back(id, absorbed.keyframe(id)->T_cw * S);
    }
    std::vector<map::MapPointId> alive;
    for (uint32_t p = 0; p < absorbed.num_points(); ++p) {
        if (absorbed.IsAlive(map::MapPointId{p})) alive.push_back(map::MapPointId{p});
    }
    merge->ResizeUpdatedPoints(static_cast<uint32_t>(alive.size()));
    const geometry::RigidTransformation S_inverse = S.Inverse();
    for (size_t i = 0; i < alive.size(); ++i) {
        const auto col = static_cast<Eigen::Index>(i);
        merge->updated_ids[i] = *new_ids[alive[i].value];
        merge->updated_positions.col(col) = S_inverse.Transform(absorbed.Position(alive[i]));
        merge->updated_normals.col(col) = S_inverse.rotation() * absorbed.Normal(alive[i]);
        merge->updated_descriptors.col(col) = absorbed.Descriptor(alive[i]);
    }
    injector_.Notify(std::move(merge));
    Settle();
    ASSERT_EQ(TrackerTestAccess::Atlas(tracker_).map_ids(), (std::vector<map::MapId>{kAbsorbedMap}));
    ASSERT_EQ(TrackerTestAccess::Atlas(tracker_).active_map(), kAbsorbedMap);
    ASSERT_NE(TrackerTestAccess::MapReplica(tracker_).keyframe(kTargetFrame), nullptr);
    ASSERT_TRUE(TrackerTestAccess::MapReplica(tracker_).CheckConsistency());

    EXPECT_EQ(tracker_.state(), TrackerState::Tracking);
    EXPECT_EQ(tracker_.current_map_id(), kAbsorbedMap);
    EXPECT_EQ(TrackerTestAccess::LastMatches(tracker_), before);
    EXPECT_TRUE(TakeCaptured().empty());
    const auto captured = Feed(sequence.frames[4].measurement);
    ExpectTrackedMetric(captured, sequence.frames[4].T_cw,
                        CorrectedAlignment(sequence, geometry::RigidTransformation::Identity()), 4,
                        events::TrackingStage::MotionModel);
    EXPECT_EQ(captured[0].map_id, kAbsorbedMap);
    EXPECT_EQ(captured[0].map_epoch, active_epoch + 1);
}

} // namespace
} // namespace voyahslam::tracking
