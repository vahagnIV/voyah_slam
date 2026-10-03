#include <voyahviz/keypoint_drawing.h>

#include <cmath>
#include <opencv2/imgproc.hpp>

#include "opencv_wrapping.h"

namespace voyahviz {

void DrawKeypoints(TColorImage& out_canvas, const voyahslam::geometry::TPoints2DF& points,
                   const voyahslam::geometry::TRowVectorF* angles, const KeypointStyle& style) {
    cv::Mat canvas = internal::WrapAsCvMat(out_canvas);  // draws into out_canvas's buffer
    const cv::Scalar color(style.color.blue, style.color.green, style.color.red);
    for (Eigen::Index i = 0; i < points.cols(); ++i) {
        cv::Point2d center(points(0, i), points(1, i));
        cv::circle(canvas, center, style.radius, color, style.thickness, cv::LINE_AA);
        if (style.draw_orientation && angles != nullptr) {
            double angle = (*angles)(i);
            cv::Point2d tip = center + cv::Point2d(std::cos(angle), std::sin(angle)) * style.orientation_length;
            cv::line(canvas, center, tip, color, style.thickness, cv::LINE_AA);
        }
    }
}

TColorImage DrawKeypoints(const voyahslam::images::TImageU8Gray& image,
                          const voyahslam::geometry::TPoints2DF& points,
                          const voyahslam::geometry::TRowVectorF* angles, const KeypointStyle& style) {
    TColorImage canvas = ToColor(image);
    DrawKeypoints(canvas, points, angles, style);
    return canvas;
}

} // namespace voyahviz
