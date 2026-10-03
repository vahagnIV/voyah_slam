#include <voyahviz/image_window.h>

#include <algorithm>
#include <opencv2/highgui.hpp>
#include <stdexcept>
#include <utility>

#include "opencv_wrapping.h"

namespace voyahviz {

ImageWindow::ImageWindow(std::string name) : name_(std::move(name)) {
    if (GuiBackend().empty()) {
        throw std::runtime_error("voyahviz::ImageWindow: no GUI backend available "
                                 "(OpenCV must be built with GTK or Qt)");
    }
    cv::namedWindow(name_, cv::WINDOW_NORMAL | cv::WINDOW_KEEPRATIO);
}

ImageWindow::~ImageWindow() {
    try {
        cv::destroyWindow(name_);
    } catch (const cv::Exception&) {
        // The window may already have been closed by the user.
    }
}

void ImageWindow::Show(const voyahslam::images::TImageU8Gray& image) {
    cv::imshow(name_, internal::WrapAsCvMat(image));
}

void ImageWindow::Show(const TColorImage& image) { cv::imshow(name_, internal::WrapAsCvMat(image)); }

int ImageWindow::WaitKey(int delay_ms) { return cv::waitKey(std::max(delay_ms, 1)); }

std::string ImageWindow::GuiBackend() { return cv::currentUIFramework(); }

} // namespace voyahviz
