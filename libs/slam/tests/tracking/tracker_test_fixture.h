#ifndef VOYAHSLAM_TESTS_TRACKING_TRACKER_TEST_FIXTURE_H
#define VOYAHSLAM_TESTS_TRACKING_TRACKER_TEST_FIXTURE_H

// TrackerTest fixture, shared by the unit tests (tracking/tracker_test.cpp)
// and the integration tests (integration/tracker_test.cpp).

#include <gtest/gtest.h>
#include <support/synthetic_sequence.h>
#include <support/test_measurements.h>
#include <support/trajectory_error.h>
#include <voyahslam/bow/vocabulary.h>
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


using Extractor = test::TestExtractor;
using TTracker = Tracker<Extractor>;
using TMapper = mapping::Mapper<Extractor>;
using TInitializationMeasurement = events::InitializationMeasurement<Extractor>;
using TFrameTracked = events::FrameTracked<Extractor>;
using TVocabulary = bow::Vocabulary<Extractor>;
using Clock = std::chrono::steady_clock;

inline constexpr double kFrameInterval = 0.05;  // SyntheticSequenceOptions::frame_interval

// What the Tracker published.
struct Captured {
    events::EventType type;
    map::MapId map_id;
    map::FrameId frame_id;       // FrameTracked: the reference frame
    geometry::RigidTransformation pose;  // FrameTracked
    double timestamp = 0.0;      // FrameTracked, TrackingLost
    bool final = false;          // TrackingLost
    // FrameTracked.
    double measurement_timestamp = 0.0;
    size_t num_observed_features = 0;
    size_t num_observed_points = 0;
    bool has_reference = false;
    events::TrackingStage stage = events::TrackingStage::MotionModel;
    uint32_t num_matches = 0;
    uint32_t num_inliers = 0;
    bool urgent = false;
    bool priority = false;
    std::vector<map::MapPointId> observed_points;
    std::vector<map::MapPointId> visible_points;
    std::vector<map::FrameId> local_frames;
};

inline size_t FrameIndexOf(double timestamp) {
    return static_cast<size_t>(std::lround(timestamp / kFrameInterval));
}

// 600 scene points: a stereo initialization needs more than 500 left features.
inline test::SyntheticStereoSequence StereoSequence(double step = 0.05, double yaw_step_deg = 0.2) {
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
    // `vocabulary` (may be null) is shared by the Mapper and the Tracker.
    explicit TrackerTest(mapping::MapperOptions mapper_options = {},
                         TrackerOptions tracker_options = {},
                         const std::shared_ptr<const TVocabulary>& vocabulary = nullptr)
        : mapper_(std::move(mapper_options), vocabulary),
          tracker_(std::move(tracker_options), vocabulary) {
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

    // Tracks `measurement`; the replica must stay consistent.
    template <typename TMeasurement>
    std::vector<Captured> Feed(TMeasurement measurement) {
        tracker_.Track(std::move(measurement));
        Drain();
        EXPECT_TRUE(tracker_.replica().CheckConsistency());
        return TakeCaptured();
    }

    std::vector<Captured> TakeCaptured() {
        std::vector<Captured> captured;
        while (events::Event* event = listener_.GetNextEvent()) {
            Captured c{event->type, event->map_id, map::FrameId{0}};
            if (event->type == events::EventType::FrameTracked) {
                const auto& tracked = *static_cast<TFrameTracked*>(event);
                c.has_reference = tracked.reference_frame_id.has_value();
                c.frame_id = tracked.reference_frame_id.value_or(map::FrameId{0});
                c.pose = tracked.pose;
                c.timestamp = tracked.timestamp;
                c.measurement_timestamp = std::visit([](const auto& m) { return m.timestamp(); },
                                                     tracked.measurement);
                c.num_observed_features = tracked.observed_features.size();
                c.num_observed_points = tracked.observed_points.size();
                c.stage = tracked.stage;
                c.num_matches = tracked.num_matches;
                c.num_inliers = tracked.num_inliers;
                c.urgent = tracked.urgent;
                c.priority = tracked.priority;
                c.observed_points = tracked.observed_points;
                c.visible_points = tracked.visible_points;
                c.local_frames = tracked.local_frames;
            } else if (event->type == events::EventType::TrackingLost) {
                const auto& lost = *static_cast<events::TrackingLost*>(event);
                c.timestamp = lost.timestamp;
                c.final = lost.final;
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

    // Expects exactly one FrameTracked within the bounds of a metric map,
    // with its inliers (and its stage, if given).
    void ExpectTrackedMetric(const std::vector<Captured>& captured,
                             const geometry::RigidTransformation& truth,
                             const test::MapAlignment& alignment, size_t frame,
                             std::optional<events::TrackingStage> stage = std::nullopt) {
        ASSERT_EQ(captured.size(), 1u) << "frame " << frame;
        ASSERT_EQ(captured[0].type, events::EventType::FrameTracked) << "frame " << frame;
        ExpectTrackedThroughLocalMap(captured[0], frame, stage);
        EXPECT_TRUE(captured[0].pose.ToMatrix34().allFinite());
        EXPECT_LT(alignment.RotationErrorDeg(captured[0].pose, truth), 0.5) << "frame " << frame;
        EXPECT_LT(alignment.CentreError(captured[0].pose, truth), 0.02) << "frame " << frame;
    }

    // A successful frame's final step is the local map, refining the initial
    // pose of `stage` (by default a last-frame pose: motion model or zero
    // motion); its observations are exactly its inliers.
    static void ExpectTrackedThroughLocalMap(const Captured& tracked, size_t frame,
                                             std::optional<events::TrackingStage> stage = std::nullopt) {
        if (stage) {
            EXPECT_EQ(tracked.stage, *stage) << "frame " << frame;
        } else {
            EXPECT_TRUE(tracked.stage == events::TrackingStage::MotionModel ||
                        tracked.stage == events::TrackingStage::ZeroMotion)
                << "frame " << frame;
        }
        EXPECT_EQ(tracked.num_observed_points, tracked.num_inliers) << "frame " << frame;
        EXPECT_EQ(tracked.num_observed_features, tracked.num_observed_points) << "frame " << frame;
        EXPECT_GE(tracked.num_matches, tracked.num_inliers) << "frame " << frame;
        EXPECT_DOUBLE_EQ(tracked.measurement_timestamp, tracked.timestamp) << "frame " << frame;
        EXPECT_TRUE(tracked.has_reference) << "frame " << frame;
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
            if (captured.size() != 1 || captured[0].type != events::EventType::FrameTracked) {
                ADD_FAILURE() << "frame " << k << " not tracked";
                continue;
            }
            EXPECT_EQ(captured[0].map_id, map::MapId{1});
            ExpectTrackedThroughLocalMap(captured[0], k);
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

    memory::ObjectPool<events::MapTransaction<Extractor>, memory::ThreadSafePolicy> transaction_pool_{16};
    // Declaration order: the components outlive every queue that can hold
    // their events (capture_), and each other's inboxes are drained in TearDown.
    TMapper mapper_;
    TTracker tracker_;
    events::Observable injector_;
    events::AsyncObserver capture_;
    events::Listener listener_ = capture_.GetListener();
};

inline std::vector<geometry::RigidTransformation> TruePoses(const test::SyntheticStereoSequence& sequence) {
    std::vector<geometry::RigidTransformation> poses;
    for (const auto& frame : sequence.frames) poses.push_back(frame.T_cw);
    return poses;
}

} // namespace voyahslam::tracking

#endif // VOYAHSLAM_TESTS_TRACKING_TRACKER_TEST_FIXTURE_H
