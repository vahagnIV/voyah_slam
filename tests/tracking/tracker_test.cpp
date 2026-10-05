#include <gtest/gtest.h>
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
#include <optional>
#include <string>
#include <string_view>
#include <thread>
#include <utility>
#include <variant>
#include <vector>

namespace voyahslam::tracking {
namespace {

using Extractor = test::TestExtractor;
using TTracker = Tracker<Extractor>;
using TMapper = mapping::Mapper<Extractor>;
using TInitializationMeasurement = events::InitializationMeasurement<Extractor>;
using Clock = std::chrono::steady_clock;

constexpr double kFrameInterval = 0.05;  // SyntheticSequenceOptions::frame_interval

// What the Tracker published.
struct Captured {
    events::EventType type;
    map::MapId map_id;
    map::FrameId frame_id;       // PositionUpdated
    geometry::RigidTransformation pose;  // PositionUpdated
    double timestamp = 0.0;      // PositionUpdated, TrackingLost
};

size_t FrameIndexOf(double timestamp) {
    return static_cast<size_t>(std::lround(timestamp / kFrameInterval));
}

// 600 scene points: a stereo initialization needs more than 500 left features.
test::SyntheticStereoSequence StereoSequence(double step = 0.05, double yaw_step_deg = 0.2) {
    test::SyntheticStereoSequenceOptions options;
    options.trajectory.num_points = 600;
    options.trajectory.step = step;
    options.trajectory.yaw_step_deg = yaw_step_deg;
    return test::MakeSyntheticStereoSequence(options);
}

// A real Mapper and a real Tracker driven synchronously, without threads: the
// Mapper's events go to the Tracker's inbox, the Tracker's to the Mapper's.
// `injector_` publishes hand-built events into the Tracker's inbox; `capture_`
// records what the Tracker publishes.
class TrackerTest : public ::testing::Test {
protected:
    // Mono accuracy (spec success criterion 1, Ruling T) is measured on a
    // well-conditioned initial map: the default 1 deg minimum parallax
    // initializes the synthetic sequence at frame 3 (~1.4 deg), whose map has
    // ~12 % depth error, and the tracked translation inherits it. 4 deg
    // initializes at frame 10 (~3 % depth error).
    static mapping::MapperOptions MapperOptionsForCurrentTest() {
        mapping::MapperOptions options;
        const auto* info = ::testing::UnitTest::GetInstance()->current_test_info();
        if (info != nullptr && std::string_view(info->name()) == "TracksSyntheticMono") {
            options.initializer.reconstruction.min_parallax_deg = 4.0;
        }
        return options;
    }

    TrackerTest() {
        mapper_.AddObserver(tracker_.inbox());
        tracker_.AddObserver(mapper_.inbox());
        injector_.ConnectAsync(tracker_.inbox());
        tracker_.AddObserver(capture_);
    }

    // Each inbox holds events of the other component's pools: empty both
    // before either is destroyed.
    void TearDown() override {
        tracker_.Stop();
        Drain();
    }

    // Runs both components until neither has anything left to process.
    void Drain() {
        bool processed = true;
        while (processed) {
            processed = false;
            while (tracker_.ProcessPending()) processed = true;
            while (mapper_.ProcessPending()) processed = true;
        }
    }

    template <typename TMeasurement>
    std::vector<Captured> Feed(TMeasurement measurement) {
        tracker_.Track(std::move(measurement));
        Drain();
        return TakeCaptured();
    }

    std::vector<Captured> TakeCaptured() {
        std::vector<Captured> captured;
        while (events::Event* event = listener_.GetNextEvent()) {
            Captured c{event->type, event->map_id, map::FrameId{0}};
            if (event->type == events::EventType::PositionUpdated) {
                const auto& position = *static_cast<events::PositionUpdated*>(event);
                c.frame_id = position.frame_id;
                c.pose = position.pose;
                c.timestamp = position.timestamp;
            } else if (event->type == events::EventType::TrackingLost) {
                c.timestamp = static_cast<events::TrackingLost*>(event)->timestamp;
            }
            captured.push_back(c);
        }
        return captured;
    }

    // The replica keyframes' true poses, by timestamp, give the alignment of
    // the current map to ground truth `T_cw_of_frame` (indexed by frame).
    test::MapAlignment AlignmentOf(const std::vector<geometry::RigidTransformation>& T_cw_of_frame,
                                   bool monocular) const {
        const MapReplica<Extractor>& replica = tracker_.replica();
        const std::vector<map::FrameId> ids = replica.keyframe_ids();
        const auto truth_of = [&](map::FrameId id) {
            const double timestamp = std::visit([](const auto& m) { return m.timestamp(); },
                                                replica.keyframe(id)->measurement);
            return T_cw_of_frame[FrameIndexOf(timestamp)];
        };
        if (!monocular) {
            EXPECT_EQ(ids.size(), 1u);
            return test::MapAlignment(truth_of(ids.front()));
        }
        EXPECT_EQ(ids.size(), 2u);
        EXPECT_TRUE(replica.keyframe(ids[0])->T_cw.ToMatrix34().isApprox(
            geometry::RigidTransformation::Identity().ToMatrix34()));
        return test::MapAlignment(truth_of(ids[0]), truth_of(ids[1]), replica.keyframe(ids[1])->T_cw);
    }

    // Expects the replica's last-frame matches to be exactly the observations
    // of the current map's keyframe `frame`.
    void ExpectLastMatchesAreKeyFrameObservations(map::FrameId frame) const {
        const auto* keyframe = tracker_.replica().keyframe(frame);
        ASSERT_NE(keyframe, nullptr);
        const std::vector<int64_t>& matches = tracker_.last_matches();
        size_t matched = 0;
        for (const int64_t id : matches) matched += id >= 0 ? 1 : 0;
        EXPECT_EQ(matched, keyframe->observations.size());
        for (const auto& [feature, point] : keyframe->observations) {
            ASSERT_LT(feature, matches.size());
            EXPECT_EQ(matches[feature], static_cast<int64_t>(point.value));
        }
    }

    // Feeds stereo frames from `begin` until one creates a map; returns its index.
    size_t InitializeStereo(const test::SyntheticStereoSequence& sequence, size_t begin) {
        for (size_t k = begin; k < sequence.frames.size(); ++k) {
            Feed(sequence.frames[k].measurement);
            if (tracker_.state() == TrackerState::Tracking) return k;
        }
        ADD_FAILURE() << "no stereo map from frame " << begin;
        return sequence.frames.size();
    }

    // Expects exactly one PositionUpdated within the bounds of a metric map.
    void ExpectTrackedMetric(const std::vector<Captured>& captured,
                             const geometry::RigidTransformation& truth,
                             const test::MapAlignment& alignment, size_t frame) {
        ASSERT_EQ(captured.size(), 1u) << "frame " << frame;
        ASSERT_EQ(captured[0].type, events::EventType::PositionUpdated) << "frame " << frame;
        EXPECT_TRUE(captured[0].pose.ToMatrix34().allFinite());
        EXPECT_LT(alignment.RotationErrorDeg(captured[0].pose, truth), 0.5) << "frame " << frame;
        EXPECT_LT(alignment.CentreError(captured[0].pose, truth), 0.02) << "frame " << frame;
    }

    struct MonoRun {
        size_t init_frame = 0;
        double max_rotation_deg = 0.0;
        double max_relative_translation = 0.0;  // of the distance travelled since frame 0
    };

    // Frames 0..29 of the mono sequence: every frame after the initialization
    // is tracked (none lost), rotation < 0.5 deg and, with check_translation,
    // centre error < 2 % of the distance travelled since frame 0 (scale from
    // the second keyframe's baseline).
    MonoRun TrackSyntheticMono(bool check_translation) {
        const test::SyntheticSequence sequence = test::MakeSyntheticSequence();
        EXPECT_EQ(sequence.frames.size(), 30u);
        std::vector<geometry::RigidTransformation> truth;
        for (const auto& frame : sequence.frames) truth.push_back(frame.T_cw);

        MonoRun run;
        std::optional<size_t> init_frame;
        std::optional<test::MapAlignment> alignment;
        size_t tracked = 0;
        for (size_t k = 0; k < sequence.frames.size(); ++k) {
            const auto captured = Feed(sequence.frames[k].measurement);
            if (!init_frame) {
                EXPECT_EQ(captured.size(), 1u);
                if (captured.size() == 1) {
                    EXPECT_EQ(captured[0].type, events::EventType::InitializationMeasurement);
                }
                if (tracker_.state() == TrackerState::Tracking) {
                    init_frame = k;
                    alignment = AlignmentOf(truth, /*monocular=*/true);
                }
                continue;
            }
            EXPECT_EQ(captured.size(), 1u) << "frame " << k;
            if (captured.size() != 1 || captured[0].type != events::EventType::PositionUpdated) {
                ADD_FAILURE() << "frame " << k << " not tracked";
                continue;
            }
            EXPECT_EQ(captured[0].map_id, map::MapId{1});
            const double rotation = alignment->RotationErrorDeg(captured[0].pose, truth[k]);
            const double travelled =
                (test::MapAlignment::Centre(truth[k]) - test::MapAlignment::Centre(truth[0])).norm();
            const double relative = alignment->CentreError(captured[0].pose, truth[k]) / travelled;
            EXPECT_LT(rotation, 0.5) << "frame " << k;
            if (check_translation) {
                EXPECT_LT(relative, 0.02) << "frame " << k;
            }
            run.max_rotation_deg = std::max(run.max_rotation_deg, rotation);
            run.max_relative_translation = std::max(run.max_relative_translation, relative);
            ++tracked;
        }
        EXPECT_TRUE(init_frame.has_value());
        if (!init_frame) return run;
        run.init_frame = *init_frame;
        EXPECT_EQ(tracked, sequence.frames.size() - 1 - *init_frame);  // zero lost frames
        EXPECT_GT(tracked, 0u);
        EXPECT_EQ(tracker_.state(), TrackerState::Tracking);
        return run;
    }

    memory::ObjectPool<events::MapCreated, memory::ThreadSafePolicy> map_created_pool_{16};
    // Declaration order: the components outlive every queue that can hold
    // their events (capture_), and each other's inboxes are drained in TearDown.
    TMapper mapper_{MapperOptionsForCurrentTest()};
    TTracker tracker_;
    events::Observable injector_;
    events::AsyncObserver capture_;
    events::Listener listener_ = capture_.GetListener();
};

std::vector<geometry::RigidTransformation> TruePoses(const test::SyntheticStereoSequence& sequence) {
    std::vector<geometry::RigidTransformation> poses;
    for (const auto& frame : sequence.frames) poses.push_back(frame.T_cw);
    return poses;
}

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

TEST_F(TrackerTest, InitialMapEventsBeforeMapCreatedAreKept) {
    // The Mapper publishes MapPointsAdded and KeyFrameCreated before MapCreated;
    // all of them reach the replica, and the keyframe becomes the last frame.
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

TEST_F(TrackerTest, TracksAndPublishesPosition) {
    const auto sequence = StereoSequence();
    ASSERT_EQ(InitializeStereo(sequence, 0), 0u);
    const map::FrameId keyframe = tracker_.replica().keyframe_ids().front();

    const auto captured = Feed(sequence.frames[1].measurement);

    ASSERT_EQ(captured.size(), 1u);
    ASSERT_EQ(captured[0].type, events::EventType::PositionUpdated);
    EXPECT_EQ(captured[0].map_id, map::MapId{1});
    EXPECT_EQ(captured[0].frame_id, keyframe);  // the MapCreated keyframe
    EXPECT_DOUBLE_EQ(captured[0].timestamp, sequence.frames[1].measurement.timestamp());
    ExpectTrackedMetric(captured, sequence.frames[1].T_cw,
                        test::MapAlignment(sequence.frames[0].T_cw), 1);
    EXPECT_EQ(tracker_.state(), TrackerState::Tracking);
}

TEST_F(TrackerTest, TracksSyntheticMono) {
    // The Mapper of this test initializes with min_parallax_deg = 4 (see
    // MapperOptionsForCurrentTest): the error bounds measure the tracker, not
    // the initial map.
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

TEST_F(TrackerTest, TracksSyntheticStereo) {
    const auto sequence = StereoSequence();
    const auto truth = TruePoses(sequence);
    ASSERT_EQ(InitializeStereo(sequence, 0), 0u);
    const test::MapAlignment alignment = AlignmentOf(truth, /*monocular=*/false);

    double max_rotation = 0.0;
    double max_translation = 0.0;
    for (size_t k = 1; k < sequence.frames.size(); ++k) {
        const auto captured = Feed(sequence.frames[k].measurement);
        ExpectTrackedMetric(captured, truth[k], alignment, k);
        if (captured.size() == 1 && captured[0].type == events::EventType::PositionUpdated) {
            max_rotation = std::max(max_rotation, alignment.RotationErrorDeg(captured[0].pose, truth[k]));
            max_translation = std::max(max_translation, alignment.CentreError(captured[0].pose, truth[k]));
        }
    }
    EXPECT_EQ(tracker_.state(), TrackerState::Tracking);
    RecordProperty("max_rotation_deg", std::to_string(max_rotation));
    RecordProperty("max_translation_m", std::to_string(max_translation));
}

TEST_F(TrackerTest, EmptyFrameIsLost) {
    const auto sequence = StereoSequence();
    ASSERT_EQ(InitializeStereo(sequence, 0), 0u);

    const auto captured = Feed(test::MakeBinocular(0, 0.07));

    ASSERT_EQ(captured.size(), 1u);
    EXPECT_EQ(captured[0].type, events::EventType::TrackingLost);
    EXPECT_EQ(captured[0].map_id, map::MapId{1});
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
            if (c.type == events::EventType::PositionUpdated) {
                EXPECT_TRUE(c.pose.ToMatrix34().allFinite());
            }
        }
    };

    ASSERT_EQ(InitializeStereo(sequence, 0), 0u);
    for (size_t k = 1; k <= 3; ++k) {
        const auto captured = Feed(sequence.frames[k].measurement);
        ASSERT_EQ(captured.size(), 1u);
        ASSERT_EQ(captured[0].type, events::EventType::PositionUpdated);
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
    ASSERT_EQ(captured[0].type, events::EventType::PositionUpdated);
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
        ASSERT_EQ(captured[0].type, events::EventType::PositionUpdated);
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

TEST_F(TrackerTest, IgnoresStaleMapCreated) {
    const auto sequence = StereoSequence();
    const auto truth = TruePoses(sequence);
    ASSERT_EQ(InitializeStereo(sequence, 0), 0u);
    Feed(test::MakeMonocular(0, 0.07));  // lost
    ASSERT_EQ(InitializeStereo(sequence, 2), 2u);
    ASSERT_EQ(tracker_.current_map_id(), map::MapId{2});
    const map::FrameId keyframe = tracker_.replica().keyframe_ids().front();

    auto* stale = map_created_pool_.Create();
    stale->map_id = map::MapId{1};
    stale->frame_id = map::FrameId{0};
    stale->pose = geometry::RigidTransformation::Identity();
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
    for (const Captured& c : captured) EXPECT_EQ(c.type, events::EventType::PositionUpdated);
}

} // namespace
} // namespace voyahslam::tracking
