#ifndef VOYAHSLAM_EXAMPLES_FULL_SLAM_FULL_SLAM_COMMON_H
#define VOYAHSLAM_EXAMPLES_FULL_SLAM_FULL_SLAM_COMMON_H

#include <algorithm>
#include <chrono>
#include <cstddef>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <Eigen/Geometry>
#include <voyahslam/bow/vocabulary.h>
#include <voyahslam/events/async_observer.h>
#include <voyahslam/events/frame_tracked.h>
#include <voyahslam/mapping/map_follower.h>
#include <voyahslam/mapping/map_snapshot.h>
#include <voyahslam/mapping/mapper.h>
#include <voyahslam/tracking/tracker.h>

namespace voyahslam::full_slam {

// ORB settings (ORB-SLAM3 defaults for TUM-VI); will move to configuration.
constexpr size_t kNumFeatures = 1500;
constexpr double kScaleFactor = 1.2;
constexpr size_t kNumLevels = 8;
constexpr unsigned kInitThresholdFast = 20;
constexpr unsigned kMinThresholdFast = 7;

// Frames first..last (inclusive, 0-based).
struct FrameRange {
    size_t first = 0;
    size_t last = 0;

    [[nodiscard]] bool Contains(size_t frame) const { return frame >= first && frame <= last; }
};

struct Options {
    std::filesystem::path dataset;
    double rate = 1.0;  // playback speed; 0 = as fast as possible
    size_t max_frames = std::numeric_limits<size_t>::max();
    bool show = false;  // display frames with keypoints (voyahviz)
    bool viewer = false;  // open the 3D map viewer (Pangolin)
    std::filesystem::path trajectory_out;  // empty: no trajectory file
    std::filesystem::path vocabulary;  // binary bag-of-words vocabulary; empty: none
    // Loop closing (opt-in; needs the vocabulary, which serves relocalization
    // either way).
    bool loop_closing = false;
    // Global BA after each loop correction (on with loop closing; the flag
    // compares the essential graph alone with it followed by global BA).
    bool global_ba = true;
    // Frames fed as measurements without features, at their own timestamps
    // (a simulated camera blackout); empty: none.
    std::optional<FrameRange> drop_frames;
};

// True if the key returned by voyahviz::ImageWindow::WaitKey() asks to quit.
inline bool IsQuitKey(int key) { return key == 'q' || key == 27; }

// `a:b` with a <= b; nullopt otherwise.
inline std::optional<FrameRange> ParseFrameRange(const std::string& text) {
    const size_t colon = text.find(':');
    if (colon == std::string::npos || colon == 0 || colon + 1 == text.size()) return std::nullopt;
    const std::string first = text.substr(0, colon);
    const std::string last = text.substr(colon + 1);
    const auto is_number = [](const std::string& s) {
        return std::all_of(s.begin(), s.end(), [](char c) { return c >= '0' && c <= '9'; });
    };
    if (!is_number(first) || !is_number(last)) return std::nullopt;
    FrameRange range{.first = std::stoul(first), .last = std::stoul(last)};
    if (range.first > range.last) return std::nullopt;
    return range;
}

// Usage: <program> <dataset_root> [--rate=R] [--max-frames=N] [--show] [--viewer]
//                  [--trajectory_out <file>] [--vocabulary <file.bin>] [--loop_closing]
//                  [--no_global_ba] [--drop_frames a:b]
inline Options ParseOptions(int argc, char** argv) {
    auto usage = [argv]() {
        std::cerr << "Usage: " << argv[0] << " <dataset_root> [--rate=R] [--max-frames=N] [--show] [--viewer]"
                  << " [--trajectory_out <file>] [--vocabulary <file.bin>] [--loop_closing]"
                  << " [--no_global_ba] [--drop_frames a:b]\n"
                  << "  --rate=R                  playback speed (1 = real time, 0 = as fast as possible)\n"
                  << "  --max-frames=N            stop after N frames\n"
                  << "  --show                    show frames with keypoints (q / Esc to quit)\n"
                  << "  --viewer                  open the 3D map viewer (Pangolin)\n"
                  << "  --trajectory_out <file>   write tracked poses (TUM format, camera-to-world)\n"
                  << "  --vocabulary <file.bin>   bag-of-words vocabulary (binary, see convert_vocabulary)\n"
                  << "  --loop_closing            enable loop closing (needs --vocabulary; off by default)\n"
                  << "  --no_global_ba            no global BA after a loop correction (essential graph only)\n"
                  << "  --drop_frames a:b         feed frames a..b (inclusive, 0-based) without features\n";
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
        } else if (arg == "--viewer") {
            options.viewer = true;
        } else if (arg == "--trajectory_out" && i + 1 < argc) {
            options.trajectory_out = argv[++i];
        } else if (arg.rfind("--trajectory_out=", 0) == 0) {
            options.trajectory_out = arg.substr(17);
        } else if (arg == "--vocabulary" && i + 1 < argc) {
            options.vocabulary = argv[++i];
        } else if (arg.rfind("--vocabulary=", 0) == 0) {
            options.vocabulary = arg.substr(13);
        } else if (arg == "--loop_closing") {
            options.loop_closing = true;
        } else if (arg == "--no_global_ba") {
            options.global_ba = false;
        } else if (arg == "--drop_frames" || arg.rfind("--drop_frames=", 0) == 0) {
            if (arg == "--drop_frames" && i + 1 >= argc) usage();
            options.drop_frames = ParseFrameRange(arg == "--drop_frames" ? argv[++i] : arg.substr(14));
            if (!options.drop_frames) usage();
        } else if (arg.rfind("--", 0) == 0 || !options.dataset.empty()) {
            usage();
        } else {
            options.dataset = arg;
        }
    }
    if (options.dataset.empty()) usage();
    return options;
}

// The vocabulary at `path` (binary format); nullptr when `path` is empty.
// Exits with status 1 if it cannot be loaded (the loader logs why).
template <typename TFeatureExtractor>
std::shared_ptr<const bow::Vocabulary<TFeatureExtractor>> LoadVocabulary(
    const std::filesystem::path& path) {
    if (path.empty()) return nullptr;
    std::optional<bow::Vocabulary<TFeatureExtractor>> vocabulary =
        bow::Vocabulary<TFeatureExtractor>::LoadBinary(path.string());
    if (!vocabulary) {
        std::cerr << "Error: cannot load vocabulary " << path << "\n";
        std::exit(EXIT_FAILURE);
    }
    std::cout << "Vocabulary: " << path << ", " << vocabulary->num_words() << " words, k "
              << vocabulary->k() << ", L " << vocabulary->L() << "\n";
    return std::make_shared<const bow::Vocabulary<TFeatureExtractor>>(std::move(*vocabulary));
}

inline const char* StateName(tracking::TrackerState state) {
    switch (state) {
        case tracking::TrackerState::WaitingForMap: return "WaitingForMap";
        case tracking::TrackerState::Tracking: return "Tracking";
        case tracking::TrackerState::RecentlyLost: return "RecentlyLost";
        case tracking::TrackerState::Lost: return "Lost";
    }
    return "?";
}

inline const char* StateName(mapping::MapperState state) {
    switch (state) {
        case mapping::MapperState::Initializing: return "Initializing";
        case mapping::MapperState::Mapping: return "Mapping";
    }
    return "?";
}

// The Atlas after the run, from a MapFollower's snapshot: its live maps with
// their Frames, the active one, and the merges applied (Mapper::num_merges(),
// readable from any thread). The snapshot is plain data, so this runs on the
// caller's thread without touching the Mapper's map.
inline void PrintAtlas(const mapping::MapSnapshot& snapshot, std::size_t num_merges, std::ostream& out) {
    out << "atlas:";
    for (const mapping::MapSnapshot::Map& map : snapshot.maps) {
        out << " map " << map.id.value << " (" << map.frame_ids.size() << " Frames"
            << (snapshot.active_map == map.id ? ", active" : "") << ")";
    }
    out << "; merges: " << num_merges << "\n";
}

// Writes every tracked frame's pose as one TUM line (`timestamp tx ty tz qx qy
// qz qw`, camera-to-world). A `# map <id>` line starts each map's section, so
// a re-initialization starts a new one. Reads the Tracker's events through its
// own inbox, polled on the caller's thread.
template <typename TFeatureExtractor>
class TrajectoryWriter {
public:
    explicit TrajectoryWriter(const std::filesystem::path& path) : out_(path), inbox_("TrajectoryWriter") {
        if (!out_) throw std::runtime_error("cannot write " + path.string());
        out_ << std::fixed;
    }

    // Before the Tracker starts.
    events::AsyncObserver& inbox() { return inbox_; }

    // Writes every pending FrameTracked; each event is released after its turn.
    void Poll() {
        while (events::EventPtr event = listener_.GetNextEvent()) {
            if (event->type != events::EventType::FrameTracked) continue;
            const auto& tracked = *static_cast<const events::FrameTracked<TFeatureExtractor>*>(event.get());
            if (!section_ || *section_ != event->map_id.value) {
                section_ = event->map_id.value;
                out_ << "# map " << *section_ << "\n";
            }
            const geometry::RigidTransformation T_wc = tracked.pose.Inverse();
            const Eigen::Quaterniond q(T_wc.rotation().template cast<double>());
            const Eigen::Vector3d t = T_wc.translation().template cast<double>();
            out_ << std::setprecision(9) << tracked.timestamp << ' ' << t.x() << ' ' << t.y() << ' '
                 << t.z() << ' ' << q.x() << ' ' << q.y() << ' ' << q.z() << ' ' << q.w() << "\n";
            ++num_poses_;
        }
    }

    [[nodiscard]] size_t num_poses() const { return num_poses_; }

private:
    std::ofstream out_;
    events::AsyncObserver inbox_;
    events::Listener listener_ = inbox_.GetListener();
    std::optional<uint32_t> section_;
    size_t num_poses_ = 0;
};

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
