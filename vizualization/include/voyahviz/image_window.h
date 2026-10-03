#ifndef VOYAHVIZ_IMAGE_WINDOW_H
#define VOYAHVIZ_IMAGE_WINDOW_H

#include <opencv2/core.hpp>
#include <string>

namespace voyahviz {

// A named OpenCV HighGUI window. HighGUI is not thread-safe: construct the
// window and call Show() / WaitKey() from one thread (in the example apps,
// the main thread). Requires OpenCV built with a GUI backend (GTK, Qt, ...);
// the constructor throws std::runtime_error otherwise.
class ImageWindow {
public:
    explicit ImageWindow(std::string name);
    ~ImageWindow();
    ImageWindow(const ImageWindow&) = delete;
    ImageWindow& operator=(const ImageWindow&) = delete;

    void Show(const cv::Mat& image);
    // Processes GUI events for up to `delay_ms` (>= 1); returns the pressed
    // key code or -1. Nothing is drawn on screen until this is called.
    int WaitKey(int delay_ms = 1);

    // Name of OpenCV's GUI backend ("GTK2", "QT6", ...); empty if none.
    [[nodiscard]] static std::string GuiBackend();

private:
    std::string name_;
};

} // namespace voyahviz

#endif // VOYAHVIZ_IMAGE_WINDOW_H
