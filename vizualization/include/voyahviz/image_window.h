#ifndef VOYAHVIZ_IMAGE_WINDOW_H
#define VOYAHVIZ_IMAGE_WINDOW_H

#include <string>
#include <voyahslam/images/image.h>
#include <voyahviz/color_image.h>

namespace voyahviz {

// A named on-screen window. Not thread-safe: construct it and call Show() /
// WaitKey() from one thread (in the example apps, the main thread). Throws
// std::runtime_error if no GUI backend is available.
class ImageWindow {
public:
    explicit ImageWindow(std::string name);
    ~ImageWindow();
    ImageWindow(const ImageWindow&) = delete;
    ImageWindow& operator=(const ImageWindow&) = delete;

    void Show(const voyahslam::images::TImageU8Gray& image);
    void Show(const TColorImage& image);
    // Processes GUI events for up to `delay_ms` (>= 1); returns the pressed
    // key code or -1. Nothing is drawn on screen until this is called.
    int WaitKey(int delay_ms = 1);

    // Name of the GUI backend ("GTK2", "QT6", ...); empty if none.
    [[nodiscard]] static std::string GuiBackend();

private:
    std::string name_;
};

} // namespace voyahviz

#endif // VOYAHVIZ_IMAGE_WINDOW_H
