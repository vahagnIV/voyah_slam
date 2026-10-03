#ifndef VOYAHSLAM_EXAMPLES_FULL_SLAM_FULL_SLAM_COMMON_H
#define VOYAHSLAM_EXAMPLES_FULL_SLAM_FULL_SLAM_COMMON_H

#include <algorithm>
#include <chrono>
#include <cstddef>
#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <limits>
#include <string>

namespace voyahslam::full_slam {

// ORB settings (ORB-SLAM3 defaults for TUM-VI); will move to configuration.
constexpr size_t kNumFeatures = 1500;
constexpr double kScaleFactor = 1.2;
constexpr size_t kNumLevels = 8;
constexpr unsigned kInitThresholdFast = 20;
constexpr unsigned kMinThresholdFast = 7;

struct Options {
    std::filesystem::path dataset;
    double rate = 1.0;  // playback speed; 0 = as fast as possible
    size_t max_frames = std::numeric_limits<size_t>::max();
    bool show = false;  // display frames with keypoints (voyahviz)
};

// True if the key returned by voyahviz::ImageWindow::WaitKey() asks to quit.
inline bool IsQuitKey(int key) { return key == 'q' || key == 27; }

// Usage: <program> <dataset_root> [--rate=R] [--max-frames=N] [--show]
inline Options ParseOptions(int argc, char** argv) {
    auto usage = [argv]() {
        std::cerr << "Usage: " << argv[0] << " <dataset_root> [--rate=R] [--max-frames=N] [--show]\n"
                  << "  --rate=R        playback speed (1 = real time, 0 = as fast as possible)\n"
                  << "  --max-frames=N  stop after N frames\n"
                  << "  --show          show frames with keypoints (q / Esc to quit)\n";
        std::exit(EXIT_FAILURE);
    };
    Options options;
    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg.rfind("--rate=", 0) == 0) {
            options.rate = std::stod(arg.substr(7));
        } else if (arg.rfind("--max-frames=", 0) == 0) {
            options.max_frames = std::stoul(arg.substr(13));
        } else if (arg == "--show") {
            options.show = true;
        } else if (arg.rfind("--", 0) == 0 || !options.dataset.empty()) {
            usage();
        } else {
            options.dataset = arg;
        }
    }
    if (options.dataset.empty()) usage();
    return options;
}

// Collects how late frames were delivered and how long loading and feature
// extraction took.
class PlaybackStats {
public:
    void Add(std::chrono::duration<double> lateness, std::chrono::duration<double> load,
             std::chrono::duration<double> extract, size_t features) {
        ++frames_;
        total_features_ += features;
        if (lateness.count() > 0.0) ++late_frames_;
        max_lateness_ = std::max(max_lateness_, lateness.count());
        total_load_ += load.count();
        total_extract_ += extract.count();
        max_frame_ = std::max(max_frame_, load.count() + extract.count());
    }

    void Print(std::ostream& out) const {
        double n = frames_ ? static_cast<double>(frames_) : 1.0;
        out << "frames: " << frames_ << ", late: " << late_frames_
            << ", max lateness: " << max_lateness_ * 1e3 << " ms\n"
            << "mean load: " << total_load_ / n * 1e3 << " ms"
            << ", mean extract: " << total_extract_ / n * 1e3 << " ms"
            << ", max load+extract: " << max_frame_ * 1e3 << " ms\n"
            << "mean features per frame: " << static_cast<double>(total_features_) / n << "\n";
    }

private:
    size_t frames_ = 0;
    size_t late_frames_ = 0;
    double max_lateness_ = 0.0;
    double total_load_ = 0.0;
    double total_extract_ = 0.0;
    double max_frame_ = 0.0;
    size_t total_features_ = 0;
};

} // namespace voyahslam::full_slam

#endif // VOYAHSLAM_EXAMPLES_FULL_SLAM_FULL_SLAM_COMMON_H
