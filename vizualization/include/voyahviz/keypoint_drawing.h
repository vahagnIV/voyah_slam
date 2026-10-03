#ifndef VOYAHVIZ_KEYPOINT_DRAWING_H
#define VOYAHVIZ_KEYPOINT_DRAWING_H

#include <voyahslam/defs.h>
#include <voyahslam/features/distortion_models/distortion_model.h>
#include <voyahslam/features/image_features.h>
#include <voyahslam/features/monocular_measurement.h>
#include <voyahslam/images/image.h>
#include <voyahviz/color_image.h>

namespace voyahviz {

struct KeypointStyle {
    Color color{0, 255, 0};
    int radius = 3;
    int thickness = 1;
    // Draws a tick from the centre in the keypoint's orientation (radians).
    bool draw_orientation = true;
    int orientation_length = 8;
};

// Draws a circle at every column of `points` (pixel coordinates of the
// canvas) and, if `angles` is given, an orientation tick.
void DrawKeypoints(TColorImage& out_canvas, const voyahslam::geometry::TPoints2DF& points,
                   const voyahslam::geometry::TRowVectorF* angles = nullptr, const KeypointStyle& style = {});

// Same, on a color copy of a gray image.
[[nodiscard]] TColorImage DrawKeypoints(const voyahslam::images::TImageU8Gray& image,
                                        const voyahslam::geometry::TPoints2DF& points,
                                        const voyahslam::geometry::TRowVectorF* angles = nullptr,
                                        const KeypointStyle& style = {});

// Draws raw extractor output (points in distorted image coordinates).
template <typename TScalar, unsigned int Length>
[[nodiscard]] TColorImage DrawFeatures(const voyahslam::images::TImageU8Gray& image,
                                       const voyahslam::features::ImageFeatures<TScalar, Length>& features,
                                       const KeypointStyle& style = {}) {
    return DrawKeypoints(image, features.points(), &features.angles(), style);
}

// A measurement's points() are *undistorted* pixel coordinates and don't line
// up with the raw image — use ProjectToImage() / DrawMeasurement() instead.
template <typename TScalar, unsigned int Length>
TColorImage DrawFeatures(const voyahslam::images::TImageU8Gray&,
                         const voyahslam::features::MonocularMeasurement<TScalar, Length>&,
                         const KeypointStyle& = {}) = delete;

// Maps a measurement's features back to raw (distorted) pixel coordinates:
// pixel = K · Distort(normalized point). `distortion_model` must be the one the
// measurement was built with (nullptr if none).
template <typename TScalar, unsigned int Length>
[[nodiscard]] voyahslam::geometry::TPoints2DF ProjectToImage(
    const voyahslam::features::MonocularMeasurement<TScalar, Length>& measurement,
    const voyahslam::features::distortion_models::DistortionModel* distortion_model) {
    voyahslam::geometry::THomogenousPointsF points = measurement.normalized_points();
    if (distortion_model != nullptr) {
        distortion_model->Distort(points);
    }
    return (measurement.K() * points).colwise().hnormalized();
}

// Draws a measurement's features on the raw image it was extracted from.
template <typename TScalar, unsigned int Length>
[[nodiscard]] TColorImage DrawMeasurement(
    const voyahslam::images::TImageU8Gray& image,
    const voyahslam::features::MonocularMeasurement<TScalar, Length>& measurement,
    const voyahslam::features::distortion_models::DistortionModel* distortion_model,
    const KeypointStyle& style = {}) {
    return DrawKeypoints(image, ProjectToImage(measurement, distortion_model), &measurement.angles(), style);
}

} // namespace voyahviz

#endif // VOYAHVIZ_KEYPOINT_DRAWING_H
