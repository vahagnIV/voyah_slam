#include <voyahviz/keypoint_drawing.h>

#include <gtest/gtest.h>

#include <voyahslam/features/distortion_models/kannala_brandt_distortion_model.h>

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
    return TFeatures(std::move(copy), std::move(strengths), std::move(angles), std::move(descriptors));
}

TEST(KeypointDrawingTest, DrawsOnBgrCopyAtKeypointPositions) {
    cv::Mat gray(50, 60, CV_8UC1, cv::Scalar(0));
    geometry::TPoints2DF points(2, 1);
    points << 20, 30;

    KeypointStyle style;
    style.color = cv::Scalar(0, 0, 255);
    style.radius = 4;
    cv::Mat canvas = DrawKeypoints(gray, points, nullptr, style);

    ASSERT_EQ(canvas.type(), CV_8UC3);
    ASSERT_EQ(canvas.size(), gray.size());
    EXPECT_EQ(cv::countNonZero(gray), 0);  // input untouched
    // A pixel on the circle (radius 4 to the right of the centre) is red.
    cv::Vec3b on_circle = canvas.at<cv::Vec3b>(30, 24);
    EXPECT_GT(on_circle[2], 100);
    EXPECT_EQ(on_circle[0], 0);
    // Far away stays black.
    EXPECT_EQ(canvas.at<cv::Vec3b>(5, 5), cv::Vec3b(0, 0, 0));
}

TEST(KeypointDrawingTest, DrawsOrientationTick) {
    cv::Mat gray(50, 60, CV_8UC1, cv::Scalar(0));
    geometry::TPoints2DF points(2, 1);
    points << 20, 30;
    geometry::TRowVectorF angles(1);
    angles << 0.0;  // pointing along +x

    KeypointStyle style;
    style.radius = 2;
    style.orientation_length = 10;
    cv::Mat canvas = DrawKeypoints(gray, points, &angles, style);
    EXPECT_GT(canvas.at<cv::Vec3b>(30, 27)[1], 100);  // on the tick, outside the circle
}

TEST(KeypointDrawingTest, ProjectToImageRecoversRawFisheyePixels) {
    // TUM-VI cam0 Kannala-Brandt calibration.
    features::distortion_models::KannalaBrandtDistortionModel model(
        0.010171079892421483, -0.010816440029919381, 0.005942781769412756, -0.001662284667857643);
    geometry::TMatrix33F K;
    K << 380.81, 0, 510.29, 0, 380.81, 514.33, 0, 0, 1;
    model.PrecomputeInitialGuessLookup(K, 1024, 1024);

    geometry::TPoints2DF raw(2, 3);
    raw << 510.0, 900.0, 100.0,
           514.0, 300.0, 850.0;
    TMeasurement measurement(MakeFeatures(raw), K, &model, 0.0);
    ASSERT_EQ(measurement.points().cols(), 3);
    // points() are undistorted and far from the raw pixels near the border...
    EXPECT_GT((measurement.points().col(1) - raw.col(1)).norm(), 10.0);
    // ... ProjectToImage maps them back.
    geometry::TPoints2DF projected = ProjectToImage(measurement, &model);
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
