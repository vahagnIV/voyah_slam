#include <gtest/gtest.h>
#include "tracking/tracker_test_fixture.h"
#include <support/synthetic_sequence.h>
#include <support/test_measurements.h>
#include <support/trajectory_error.h>
#include <voyahslam/bow/frame_database.h>
#include <voyahslam/features/stereo_matcher.h>
#include <voyahslam/map/covisibility_index.h>
#include <voyahslam/tracking/relocalizer.h>
#include <voyahslam/tracking/tracker.h>

#include <cstddef>
#include <cstdint>
#include <vector>

namespace voyahslam::tracking {
namespace {

constexpr size_t kMappedFrames = 20;   // frames 0..19 build the map
constexpr size_t kBlackout = 10;       // frames 20..29 arrive empty
constexpr size_t kResumeFrame = kMappedFrames + kBlackout;

test::SyntheticStereoSequence LongStereoSequence(uint32_t seed = 1) {
    test::SyntheticStereoSequenceOptions options;
    options.trajectory.num_points = 600;
    options.trajectory.num_frames = kResumeFrame + 5;
    options.trajectory.seed = seed;
    return test::MakeSyntheticStereoSequence(options);
}

// The synthetic stereo sequence makes few Frames (3 by frame 20), so any map
// may relocalize; otherwise the default relocalization options.
TrackerOptions RelocalizingOptions() {
    TrackerOptions options;
    options.relocalization.min_map_frames_to_relocalize = 0;
    return options;
}

// The root vocabulary of the Tracker tests: SearchByBoW compares every
// feature pair.
class RelocalizationTest : public TrackerTest {
protected:
    RelocalizationTest() : TrackerTest({}, RelocalizingOptions(), MakeRootVocabulary()) {}

    // Tracks frames 0..kMappedFrames-1 of `sequence`, then loses tracking
    // with kBlackout empty measurements at the next frames' timestamps.
    void MapThenBlackOut(const test::SyntheticStereoSequence& sequence) {
        ASSERT_EQ(InitializeStereo(sequence, 0), 0u);
        for (size_t k = 1; k < kMappedFrames; ++k) {
            const auto captured = Feed(sequence.frames[k].measurement);
            ASSERT_EQ(captured.size(), 1u) << "frame " << k;
            ASSERT_EQ(captured[0].type, events::EventType::FrameTracked) << "frame " << k;
        }
        ASSERT_GE(tracker_.replica().num_keyframes(), 2u);
        for (size_t k = kMappedFrames; k < kResumeFrame; ++k) {
            const auto captured = Feed(test::MakeBinocular(0, FrameTimestamp(k)));
            if (k == kMappedFrames) {
                ASSERT_EQ(captured.size(), 1u);
                EXPECT_EQ(captured[0].type, events::EventType::TrackingLost);
                EXPECT_FALSE(captured[0].final);
            } else {
                EXPECT_TRUE(captured.empty()) << "frame " << k;
            }
            ASSERT_EQ(tracker_.state(), TrackerState::RecentlyLost);
        }
    }

    static double FrameTimestamp(size_t k) { return static_cast<double>(k) * kFrameInterval; }

    // `frame`'s measurement, stereo-matched as the Tracker does on dequeue.
    static features::TMeasurement<Extractor> Matched(const test::SyntheticStereoSequenceFrame& frame) {
        auto measurement = frame.measurement;
        measurement.SetStereoMatches(features::MatchStereo<Extractor>(measurement, TrackerOptions{}.stereo_matcher));
        return measurement;
    }

    // Relocalize() on the Tracker's replica with the Tracker's defaults.
    RelocalizationResult RelocalizeDirectly(const test::SyntheticStereoSequenceFrame& frame) const {
        const TrackerOptions options = RelocalizingOptions();
        const PoseRefinement refinement{options.pose_optimization, options.sigma_px, options.level_scale};
        return Relocalize(tracker_.replica(), Matched(frame), *MakeRootVocabulary(), options.relocalization,
                          refinement, options.matcher);
    }

    void ExpectRelocalized(const std::vector<Captured>& captured, const test::SyntheticStereoSequence& sequence,
                           size_t frame) {
        ASSERT_EQ(captured.size(), 1u);
        ASSERT_EQ(captured[0].type, events::EventType::FrameTracked);
        EXPECT_EQ(captured[0].map_id, map::MapId{1});
        EXPECT_EQ(tracker_.current_map_id(), map::MapId{1});
        EXPECT_EQ(tracker_.state(), TrackerState::Tracking);
        ExpectTrackedThroughLocalMap(captured[0], frame, events::TrackingStage::Relocalization);
        const test::MapAlignment alignment(sequence.frames[0].T_cw);
        EXPECT_LT(alignment.RotationErrorDeg(captured[0].pose, sequence.frames[frame].T_cw), 0.5);
        EXPECT_LT(alignment.CentreError(captured[0].pose, sequence.frames[frame].T_cw), 0.01);
    }
};

TEST(RelocalizationOptionsTest, SpecDefaults) {
    const RelocalizationOptions options;
    EXPECT_DOUBLE_EQ(options.min_common_words_ratio, 0.8);
    EXPECT_EQ(options.group_neighbours, 10u);
    EXPECT_DOUBLE_EQ(options.min_group_score_ratio, 0.75);
    EXPECT_DOUBLE_EQ(options.nn_ratio, 0.75);
    EXPECT_EQ(options.min_bow_matches, 15u);
    EXPECT_EQ(options.ransac_max_iterations, 300);
    EXPECT_DOUBLE_EQ(options.ransac_confidence, 0.99);
    EXPECT_EQ(options.ransac_min_inliers, 10u);
    EXPECT_EQ(options.ransac_iterations_per_round, 5);
    EXPECT_EQ(options.min_refined_inliers, 10u);
    EXPECT_EQ(options.min_inliers, 50u);
    EXPECT_EQ(options.min_second_pass_inliers, 30u);
    EXPECT_DOUBLE_EQ(options.first_pass_window, 10.0);
    EXPECT_DOUBLE_EQ(options.first_pass_max_distance, 100.0);
    EXPECT_DOUBLE_EQ(options.second_pass_window, 3.0);
    EXPECT_DOUBLE_EQ(options.second_pass_max_distance, 64.0);
}

TEST_F(RelocalizationTest, RelocalizesIntoSameMap) {
    const auto sequence = LongStereoSequence();
    MapThenBlackOut(sequence);
    const size_t num_frames = mapper_.num_keyframes();

    const auto captured = Feed(sequence.frames[kResumeFrame].measurement);

    ExpectRelocalized(captured, sequence, kResumeFrame);
    // The Mapper kept its map throughout.
    EXPECT_EQ(mapper_.current_map_id(), map::MapId{1});
    EXPECT_GE(mapper_.num_keyframes(), num_frames);

    // Tracking goes on from the relocalized pose.
    const auto next = Feed(sequence.frames[kResumeFrame + 1].measurement);
    ASSERT_EQ(next.size(), 1u);
    EXPECT_EQ(next[0].type, events::EventType::FrameTracked);
    EXPECT_EQ(next[0].map_id, map::MapId{1});
}

TEST_F(RelocalizationTest, UnmappedPlaceDoesNotRelocalize) {
    const auto sequence = LongStereoSequence();
    MapThenBlackOut(sequence);
    // Another scene: other points, other descriptors.
    const auto elsewhere = LongStereoSequence(/*seed=*/2);

    for (size_t k = kResumeFrame; k < kResumeFrame + 5; ++k) {
        EXPECT_FALSE(RelocalizeDirectly(elsewhere.frames[k]).success) << "frame " << k;
        EXPECT_TRUE(Feed(elsewhere.frames[k].measurement).empty()) << "frame " << k;
        EXPECT_EQ(tracker_.state(), TrackerState::RecentlyLost) << "frame " << k;
    }
    EXPECT_EQ(mapper_.current_map_id(), map::MapId{1});
}

TEST_F(RelocalizationTest, SkipsFrameRemovedMidRecovery) {
    const auto sequence = LongStereoSequence();
    MapThenBlackOut(sequence);
    const RelocalizationResult first = RelocalizeDirectly(sequence.frames[kResumeFrame]);
    ASSERT_TRUE(first.success);
    ASSERT_NE(tracker_.replica().keyframe(first.frame), nullptr);

    // The Mapper removes the Frame relocalization would match while the
    // Tracker is RecentlyLost.
    auto* transaction = transaction_pool_.Create();
    transaction->map_id = map::MapId{1};
    transaction->removed_frames = {first.frame};
    injector_.Notify(transaction);
    Drain();
    ASSERT_EQ(tracker_.replica().keyframe(first.frame), nullptr);
    ASSERT_FALSE(tracker_.replica().database().Contains(first.frame));

    const RelocalizationResult second = RelocalizeDirectly(sequence.frames[kResumeFrame]);
    ASSERT_TRUE(second.success);
    EXPECT_NE(second.frame, first.frame);
    EXPECT_NE(tracker_.replica().keyframe(second.frame), nullptr);

    const auto captured = Feed(sequence.frames[kResumeFrame].measurement);
    ExpectRelocalized(captured, sequence, kResumeFrame);
    EXPECT_EQ(captured[0].frame_id, second.frame);  // the reference is the matched Frame
    for (const map::FrameId frame : captured[0].local_frames) EXPECT_NE(frame, first.frame);
}

TEST_F(RelocalizationTest, EmptyMeasurementsAdvanceTimeout) {
    const auto sequence = LongStereoSequence();
    MapThenBlackOut(sequence);
    const double lost_at = FrameTimestamp(kMappedFrames);

    // Empty measurements are relocalized (and fail) without stopping the clock.
    EXPECT_FALSE(RelocalizeDirectly({test::MakeBinocular(0, lost_at + 1.0)}).success);
    for (const double dt : {1.0, 4.99}) {
        EXPECT_TRUE(Feed(test::MakeBinocular(0, lost_at + dt)).empty()) << dt;
        EXPECT_EQ(tracker_.state(), TrackerState::RecentlyLost) << dt;
    }
    const auto captured = Feed(test::MakeBinocular(0, lost_at + 5.0));
    ASSERT_EQ(captured.size(), 1u);
    EXPECT_EQ(captured[0].type, events::EventType::TrackingLost);
    EXPECT_TRUE(captured[0].final);
    EXPECT_EQ(tracker_.state(), TrackerState::Lost);
}

TEST(RelocalizationCandidatesTest, CandidateGroupingPrefersCovisibleCluster) {
    // Cluster X: Frames 1, 2, 3, all covisible (group score 0.65, best 1).
    // Cluster Y: Frames 10, 11 (group score 0.55, best 10): kept, after X,
    // although Frame 10 is the best single hit. Frame 20 alone (0.3) is below
    // 0.75 x 0.65; its covisible Frame 21 is not a hit and adds nothing.
    map::CovisibilityIndex covisibility;
    const auto observe = [&](uint32_t frame, uint32_t point) {
        covisibility.Add(map::FrameId{frame}, point, map::MapPointId{point});
    };
    for (const uint32_t frame : {1u, 2u, 3u}) observe(frame, 0);
    for (const uint32_t frame : {10u, 11u}) observe(frame, 5);
    for (const uint32_t frame : {20u, 21u}) observe(frame, 9);
    const std::vector<bow::DatabaseHit> hits = {
        {map::FrameId{10}, 5, 0.45}, {map::FrameId{20}, 5, 0.3}, {map::FrameId{1}, 5, 0.25},
        {map::FrameId{2}, 5, 0.2},   {map::FrameId{3}, 5, 0.2},  {map::FrameId{11}, 5, 0.1},
    };

    const std::vector<map::FrameId> candidates = RelocalizationCandidates(hits, covisibility, 10, 0.75);

    EXPECT_EQ(candidates, (std::vector<map::FrameId>{map::FrameId{1}, map::FrameId{10}}));
}

TEST(RelocalizationCandidatesTest, NeighboursBeyondTheLimitDoNotCount) {
    // Frames 1 and 2 share one point; each shares two with a Frame that is
    // not a hit (4 and 6), its top covisible Frame. With one neighbour 1 and
    // 2 do not count each other, so Frame 5 alone (0.35) is the best group
    // and 1's (0.3) falls below 0.9 x 0.35.
    map::CovisibilityIndex covisibility;
    const auto share = [&](uint32_t a, uint32_t b, uint32_t point) {
        covisibility.Add(map::FrameId{a}, point, map::MapPointId{point});
        covisibility.Add(map::FrameId{b}, point, map::MapPointId{point});
    };
    share(1, 4, 0);
    share(1, 4, 1);
    share(2, 6, 3);
    share(2, 6, 4);
    share(1, 2, 2);
    const std::vector<bow::DatabaseHit> hits = {
        {map::FrameId{5}, 5, 0.35}, {map::FrameId{1}, 5, 0.3}, {map::FrameId{2}, 5, 0.1}};

    EXPECT_EQ(RelocalizationCandidates(hits, covisibility, 1, 0.9),
              (std::vector<map::FrameId>{map::FrameId{5}}));
    // With two neighbours, 1's group (0.4) wins.
    EXPECT_EQ(RelocalizationCandidates(hits, covisibility, 2, 0.9),
              (std::vector<map::FrameId>{map::FrameId{1}}));
}

} // namespace
} // namespace voyahslam::tracking
