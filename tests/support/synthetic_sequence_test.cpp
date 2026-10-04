#include <gtest/gtest.h>

#include "support/synthetic_sequence.h"

namespace voyahslam::test {
namespace {

TEST(SyntheticSequenceTest, ProjectionsMatchPoses) {
    SyntheticSequenceOptions options;
    options.pixel_noise = 0.0;
    const SyntheticSequence sequence = MakeSyntheticSequence(options);
    const geometry::TMatrix33F K = SyntheticIntrinsics();
    ASSERT_EQ(sequence.frames.size(), options.num_frames);
    size_t checked = 0;
    for (const SyntheticFrame& frame : sequence.frames) {
        ASSERT_EQ(static_cast<size_t>(frame.measurement.points().cols()),
                  frame.point_of_feature.size());
        for (size_t j = 0; j < frame.point_of_feature.size(); ++j) {
            const Eigen::Index p = frame.point_of_feature[j];
            if (p < 0) {
                continue;
            }
            const geometry::TPoint2DF expected =
                (K * frame.T_cw.Transform(geometry::TPoint3DF(sequence.points_w.col(p)))).hnormalized();
            EXPECT_NEAR((frame.measurement.points().col(j) - expected).norm(), 0.0, 1e-9);
            ++checked;
        }
    }
    EXPECT_GT(checked, 0U);
}

TEST(SyntheticSequenceTest, DescriptorsStayClose) {
    SyntheticSequenceOptions options;
    const SyntheticSequence sequence = MakeSyntheticSequence(options);
    const SyntheticFrame& f0 = sequence.frames[0];
    const SyntheticFrame& f5 = sequence.frames[5];

    size_t same = 0;
    size_t far_pairs = 0;
    size_t far_total = 0;
    for (size_t i = 0; i < f0.point_of_feature.size(); ++i) {
        for (size_t j = 0; j < f5.point_of_feature.size(); ++j) {
            const int distance = TestExtractor::Distance(f0.measurement.descriptors().col(i),
                                                         f5.measurement.descriptors().col(j));
            if (f0.point_of_feature[i] >= 0 && f0.point_of_feature[i] == f5.point_of_feature[j]) {
                EXPECT_LE(distance, 2 * options.bit_flips);
                ++same;
            } else if ((f0.point_of_feature[i] < 0) != (f5.point_of_feature[j] < 0)) {
                // Random extra against a scene point.
                ++far_total;
                far_pairs += distance > 50 ? 1 : 0;
            }
        }
    }
    EXPECT_GT(same, 0U);
    ASSERT_GT(far_total, 0U);
    EXPECT_GE(static_cast<double>(far_pairs) / static_cast<double>(far_total), 0.95);
}

TEST(SyntheticSequenceTest, Deterministic) {
    const SyntheticSequence a = MakeSyntheticSequence();
    const SyntheticSequence b = MakeSyntheticSequence();
    EXPECT_EQ(a.points_w, b.points_w);
    ASSERT_EQ(a.frames.size(), b.frames.size());
    EXPECT_EQ(a.frames[3].measurement.points(), b.frames[3].measurement.points());
    EXPECT_EQ(a.frames[3].point_of_feature, b.frames[3].point_of_feature);
}

}  // namespace
}  // namespace voyahslam::test
