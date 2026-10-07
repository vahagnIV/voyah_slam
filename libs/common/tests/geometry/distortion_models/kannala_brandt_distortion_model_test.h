#ifndef VOYAHSLAM_TEST_GEOMETRY_KANNALA_BRANDT_DISTORTION_MODEL_TEST_H
#define VOYAHSLAM_TEST_GEOMETRY_KANNALA_BRANDT_DISTORTION_MODEL_TEST_H

#include <gtest/gtest.h>
#include "voyahslam/geometry/distortion_models/kannala_brandt_distortion_model.h"
#include "voyahslam/defs.h"

namespace voyahslam::test::features
{
class KannalaBrandtDistortionModelTest : public ::testing::Test
{
protected:
    void SetUp() override;

    // Test data
    std::vector<precision_t> default_coefficients_;
    std::vector<precision_t> fisheye_coefficients_;
    geometry::THomogenousPointsF test_points_;
    geometry::THomogenousPointsF test_points_copy_;

    // Helper methods
    void CreateTestPoints();
    void VerifyPointsValid(const geometry::THomogenousPointsF& points) const;
    [[nodiscard]] precision_t ComputeAngleFromPoint(precision_t x, precision_t y) const;
    [[nodiscard]] precision_t ComputeRadialDistance(precision_t x, precision_t y) const;
};
} // namespace voyahslam::test::features

#endif // VOYAHSLAM_TEST_GEOMETRY_KANNALA_BRANDT_DISTORTION_MODEL_TEST_H
