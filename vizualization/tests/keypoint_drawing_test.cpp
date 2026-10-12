#include <voyahviz/keypoint_drawing.h>

#include <gtest/gtest.h>

#include <memory>

#include <voyahslam/geometry/distortion_models/kannala_brandt_distortion_model.h>

namespace voyahviz {
namespace {

using namespace voyahslam;

using TFeatures = features::ImageFeatures<uint8_t, 32>;
using TMeasurement = features::MonocularMeasurement<uint8_t, 32>;

TFeatures MakeFeatures(const geometry::TPoints2DF& points) {
    const auto n = points.cols();
    geometry::TPoints2DF copy = points;
    geometry::TRowVectorF strengths = geometry::TRowVectorF::Ones(n);
    geometry::TRowVectorF angles = geometry::TRowVectorF::Zero(n);
    TFeatures::TDescriptors descriptors = TFeatures::TDescriptors::Zero(32, n);
    return TFeatures(std::move(copy), std::move(strengths), std::move(angles), std::move(descriptors),
                     features::TLevels::Zero(n));
}

images::TImageU8Gray BlackImage(unsigned width, unsigned height) {
    images::TImageU8Gray image(width, height);
    for (unsigned y = 0; y < height; ++y)
        for (unsigned x = 0; x < width; ++x) image.Set(y, x, 0, 0);
    return image;
}

TEST(KeypointDrawingTest, DrawsOnColorCopyAtKeypointPositions) {
    images::TImageU8Gray gray = BlackImage(60, 50);
    geometry::TPoints2DF points(2, 1);
    points << 20, 30;

    KeypointStyle style;
    style.color = Color{255, 0, 0};  // red
    style.radius = 4;
    TColorImage canvas = DrawKeypoints(gray, points, nullptr, style);

    ASSERT_EQ(canvas.Width(), 60u);
    ASSERT_EQ(canvas.Height(), 50u);
    EXPECT_EQ(gray.At(30, 24, 0), 0);  // input untouched
    // On the circle (radius 4 right of the centre): red, stored as B, G, R.
    EXPECT_GT(canvas.At(30, 24, 2), 100);
    EXPECT_EQ(canvas.At(30, 24, 0), 0);
    // Far away stays black.
    EXPECT_EQ(canvas.At(5, 5, 2), 0);
}

TEST(KeypointDrawingTest, DrawsIntoExistingCanvas) {
    TColorImage canvas = ToColor(BlackImage(60, 50));
    geometry::TPoints2DF points(2, 1);
    points << 20, 30;
    KeypointStyle style;
    style.radius = 4;
    DrawKeypoints(canvas, points, nullptr, style);
    EXPECT_GT(canvas.At(30, 24, 1), 100);  // default colour is green
}

TEST(KeypointDrawingTest, DrawsOrientationTick) {
    images::TImageU8Gray gray = BlackImage(60, 50);
    geometry::TPoints2DF points(2, 1);
    points << 20, 30;
    geometry::TRowVectorF angles(1);
    angles << 0.0;  // pointing along +x

    KeypointStyle style;
    style.radius = 2;
    style.orientation_length = 10;
    TColorImage canvas = DrawKeypoints(gray, points, &angles, style);
    EXPECT_GT(canvas.At(30, 27, 1), 100);  // on the tick, outside the circle
}

TEST(KeypointDrawingTest, ProjectToImageRecoversRawFisheyePixels) {
    // TUM-VI cam0 Kannala-Brandt calibration.
    const auto model = std::make_shared<geometry::distortion_models::KannalaBrandtDistortionModel>(
        0.010171079892421483, -0.010816440029919381, 0.005942781769412756, -0.001662284667857643);
    geometry::TMatrix33F K;
    K << 380.81, 0, 510.29, 0, 380.81, 514.33, 0, 0, 1;
    model->PrecomputeInitialGuessLookup(K, 1024, 1024);

    geometry::TPoints2DF raw(2, 3);
    raw << 510.0, 900.0, 100.0,
           514.0, 300.0, 850.0;
    TMeasurement measurement(MakeFeatures(raw), K, model, 0.0);
    ASSERT_EQ(measurement.points().cols(), 3);
    // points() are undistorted and far from the raw pixels near the border...
    EXPECT_GT((measurement.points().col(1) - raw.col(1)).norm(), 10.0);
    // ... ProjectToImage maps them back.
    geometry::TPoints2DF projected = ProjectToImage(measurement, model.get());
    EXPECT_TRUE(projected.isApprox(raw, 1e-6)) << projected;
}

TEST(KeypointDrawingTest, ProjectToImageWithoutDistortionReturnsPoints) {
    geometry::TMatrix33F K;
    K << 100, 0, 50, 0, 100, 40, 0, 0, 1;
    geometry::TPoints2DF raw(2, 2);
    raw << 10, 70, 20, 60;
    TMeasurement measurement(MakeFeatures(raw), K, nullptr, 0.0);
    EXPECT_TRUE(ProjectToImage(measurement, nullptr).isApprox(raw, 1e-9));
}

} // namespace
} // namespace voyahviz
