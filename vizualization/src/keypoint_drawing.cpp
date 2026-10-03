#include <voyahviz/keypoint_drawing.h>

#include <cmath>
#include <opencv2/imgproc.hpp>

namespace voyahviz {

cv::Mat DrawKeypoints(const cv::Mat& image, const voyahslam::geometry::TPoints2DF& points,
                      const voyahslam::geometry::TRowVectorF* angles, const KeypointStyle& style) {
    cv::Mat canvas;
    if (image.channels() == 1) {
        cv::cvtColor(image, canvas, cv::COLOR_GRAY2BGR);
    } else {
        canvas = image.clone();
    }
    for (Eigen::Index i = 0; i < points.cols(); ++i) {
        cv::Point2d center(points(0, i), points(1, i));
        cv::circle(canvas, center, style.radius, style.color, style.thickness, cv::LINE_AA);
        if (style.draw_orientation && angles != nullptr) {
            double angle = (*angles)(i);
            cv::Point2d tip = center + cv::Point2d(std::cos(angle), std::sin(angle)) * style.orientation_length;
            cv::line(canvas, center, tip, style.color, style.thickness, cv::LINE_AA);
        }
    }
    return canvas;
}

} // namespace voyahviz
