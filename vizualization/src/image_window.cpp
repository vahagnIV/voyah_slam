#include <voyahviz/image_window.h>

#include <algorithm>
#include <opencv2/highgui.hpp>
#include <stdexcept>
#include <utility>

namespace voyahviz {

ImageWindow::ImageWindow(std::string name) : name_(std::move(name)) {
    if (GuiBackend().empty()) {
        throw std::runtime_error("voyahviz::ImageWindow: OpenCV was built without a GUI backend "
                                 "(rebuild with GTK or Qt to show windows)");
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

void ImageWindow::Show(const cv::Mat& image) { cv::imshow(name_, image); }

int ImageWindow::WaitKey(int delay_ms) { return cv::waitKey(std::max(delay_ms, 1)); }

std::string ImageWindow::GuiBackend() { return cv::currentUIFramework(); }

} // namespace voyahviz
