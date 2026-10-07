#include <gtest/gtest.h>
#include "tracking/tracker_test_fixture.h"

#include <algorithm>
#include <cstddef>
#include <string>
#include <vector>

namespace voyahslam::tracking {
namespace {

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
        if (captured.size() == 1 && captured[0].type == events::EventType::FrameTracked) {
            max_rotation = std::max(max_rotation, alignment.RotationErrorDeg(captured[0].pose, truth[k]));
            max_translation = std::max(max_translation, alignment.CentreError(captured[0].pose, truth[k]));
        }
    }
    EXPECT_EQ(tracker_.state(), TrackerState::Tracking);
    RecordProperty("max_rotation_deg", std::to_string(max_rotation));
    RecordProperty("max_translation_m", std::to_string(max_translation));
}

} // namespace
} // namespace voyahslam::tracking
