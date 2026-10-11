#ifndef VOYAHSLAM_ALGS_TESTS_FEATURES_CORRIDOR1_FRAMES_H
#define VOYAHSLAM_ALGS_TESTS_FEATURES_CORRIDOR1_FRAMES_H

// Real TUM-VI corridor1 frames (1024x1024, 16-bit PNG read as 8-bit, as the
// dataset reader does) for the FAST and ORB regression tests.
//
// The dataset lives outside the repository: $VOYAHSLAM_CORRIDOR1, else
// ~/Downloads/dataset-corridor1_1024_16. Tests skip when the directory is absent;
// a present dataset with too few frames fails.

#include <voyahslam/images/image.h>
#include <voyahslam/io/read_image.h>

#include <algorithm>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <string>
#include <vector>

namespace voyahslam::test {

inline std::filesystem::path Corridor1Cam0Dir() {
    std::filesystem::path root;
    if (const char* env = std::getenv("VOYAHSLAM_CORRIDOR1"); env && *env) {
        root = env;
    } else if (const char* home = std::getenv("HOME"); home && *home) {
        root = std::filesystem::path(home) / "Downloads" / "dataset-corridor1_1024_16";
    }
    return root / "mav0" / "cam0" / "data";
}

// Sorted frame file names (empty when the dataset is missing).
inline std::vector<std::filesystem::path> Corridor1Frames() {
    std::vector<std::filesystem::path> frames;
    const auto dir = Corridor1Cam0Dir();
    std::error_code error;
    if (!std::filesystem::is_directory(dir, error)) return frames;
    for (const auto& entry : std::filesystem::directory_iterator(dir)) {
        if (entry.path().extension() == ".png") frames.push_back(entry.path());
    }
    std::sort(frames.begin(), frames.end());
    return frames;
}

// Whether the dataset directory exists (tests skip only when it does not).
inline bool Corridor1Present() {
    std::error_code error;
    return std::filesystem::is_directory(Corridor1Cam0Dir(), error);
}

// A spread of frames across the sequence: indices 0, 1000, 2000, ...
inline std::vector<std::filesystem::path> Corridor1SampleFrames(size_t count) {
    const auto all = Corridor1Frames();
    std::vector<std::filesystem::path> sample;
    for (size_t i = 0; i < count && i * 1000 < all.size(); ++i) {
        sample.push_back(all[i * 1000]);
    }
    return sample;
}

inline images::TWorkingImage ReadFrame(const std::filesystem::path& path) {
    std::ifstream file(path, std::ios::binary);
    return io::ReadImageFromStream(file);
}

}  // namespace voyahslam::test

#endif  // VOYAHSLAM_ALGS_TESTS_FEATURES_CORRIDOR1_FRAMES_H
