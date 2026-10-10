#include <chrono>
#include <exception>
#include <iostream>
#include <utility>

#include <dataset_reader/euroc_dataset_reader.h>
#include <dataset_reader/playback_clock.h>
#include <optional>
#include <voyahviz/image_window.h>
#include <voyahviz/keypoint_drawing.h>
#include <voyahviz/map_viewer.h>
#include <voyahslam/logging/logger.h>
#include <voyahslam/features/orb_feature_extractor.h>
#include <voyahslam/slam/slam.h>

#include "full_slam_common.h"

using namespace voyahslam;

using TFeatureExtractor = features::OrbFeatureExtractor;
using TSlam = slam::Slam<TFeatureExtractor>;
using TBinocularMeasurement = TSlam::TTracker::TBinocularMeasurement;

int main(int argc, char** argv) {
    // Debug logs (e.g. initialization and bundle adjustment) are shown by the examples.
    voyahslam::Logger::SetLevel(spdlog::level::debug);
    full_slam::Options options = full_slam::ParseOptions(argc, argv);
    // Before anything starts: a bad file exits with status 1.
    const auto vocabulary = full_slam::LoadVocabulary<TFeatureExtractor>(options.vocabulary);
    try {
        dataset_reader::EurocDatasetReader reader(options.dataset);
        if (!reader.has_stereo()) {
            std::cerr << "Error: dataset has no second camera\n";
            return EXIT_FAILURE;
        }
        const features::StereoRig& rig = reader.stereo_rig();
        std::cout << "Dataset: " << options.dataset << ", " << reader.size()
                  << " stereo frames, baseline " << rig.baseline << " m\n";

        std::cout << "Loop closing: " << (options.loop_closing && vocabulary ? "on" : "off")
                  << (options.loop_closing && !vocabulary ? " (--loop_closing needs --vocabulary)" : "") << "\n";
        if (options.drop_frames) {
            std::cout << "Dropping frames " << options.drop_frames->first << ".."
                      << options.drop_frames->last << " (no features)\n";
        }

        TFeatureExtractor extractor(full_slam::kScaleFactor, full_slam::kNumLevels,
                                    full_slam::kInitThresholdFast, full_slam::kMinThresholdFast);
        // 3D viewer: consumes Tracker and Mapper events on its own thread (later
        // a remote process behind a serializing observer). Declared before slam
        // so that slam is destroyed first and nothing is published into a
        // destroyed inbox.
        voyahviz::MapViewer<TFeatureExtractor> viewer(
            voyahviz::MapViewerOptions{.show_window = options.viewer});
        // Created after the viewer: cv::namedWindow (gtk_init) may start threads,
        // and the viewer's constructor may setenv (so it must run first).
        std::optional<voyahviz::ImageWindow> window;
        if (options.show) window.emplace("binocular_full_slam");
        // Declared before slam for the same reason as the viewer.
        std::optional<full_slam::TrajectoryWriter<TFeatureExtractor>> trajectory;
        if (!options.trajectory_out.empty()) trajectory.emplace(options.trajectory_out);
        TSlam slam({}, {}, vocabulary, loop_closing::LoopCloserOptions{.enabled = options.loop_closing});
        // Also when an exception unwinds: stop the producers, then the viewer
        // and the trajectory (their final drains recycle events into slam's
        // pools), all before ~Slam destroys those pools. Stop() and Poll()
        // are idempotent, so the explicit calls below on the normal path are
        // fine.
        struct StopAll {
            TSlam& slam;
            voyahviz::MapViewer<TFeatureExtractor>& viewer;
            std::optional<full_slam::TrajectoryWriter<TFeatureExtractor>>& trajectory;
            ~StopAll() {
                slam.Stop();
                viewer.Stop();
                if (trajectory) trajectory->Poll();
            }
        } stop_all{slam, viewer, trajectory};
        // Wire before Start().
        slam.tracker().AddObserver(viewer.inbox());
        slam.mapper().AddObserver(viewer.inbox());
        if (trajectory) slam.tracker().AddObserver(trajectory->inbox());
        viewer.Start();
        slam.Start();

        dataset_reader::PlaybackClock clock(options.rate);
        full_slam::PlaybackStats stats;
        size_t num_frames = std::min(reader.size(), options.max_frames);
        bool quit = false;
        for (size_t i = 0; i < num_frames; ++i) {
            auto lateness = clock.WaitUntil(reader.timestamp(i));
            auto start = std::chrono::steady_clock::now();

            dataset_reader::BinocularFrame frame = reader.LoadBinocular(i);
            auto loaded = std::chrono::steady_clock::now();
            // A dropped frame keeps its timestamp but has no features.
            const bool dropped = options.drop_frames && options.drop_frames->Contains(i);
            auto left = dropped ? TFeatureExtractor::TFeatures{}
                                : extractor.DetectAndExtract(frame.left, full_slam::kNumFeatures);
            auto right = dropped ? TFeatureExtractor::TFeatures{}
                                 : extractor.DetectAndExtract(frame.right, full_slam::kNumFeatures);
            TBinocularMeasurement measurement(std::move(left), std::move(right), rig,
                                              frame.timestamp);
            auto extracted = std::chrono::steady_clock::now();
            size_t num_features = static_cast<size_t>(measurement.left().points().cols() +
                                                      measurement.right().points().cols());
            if (window) {
                window->Show(voyahviz::HorizontalConcat(
                    voyahviz::DrawMeasurement(frame.left, measurement.left(), rig.distortion_left),
                    voyahviz::DrawMeasurement(frame.right, measurement.right(), rig.distortion_right)));
            }
            slam.tracker().Track(std::move(measurement));
            if (trajectory) trajectory->Poll();

            stats.Add(lateness, loaded - start, extracted - loaded, num_features);
            if (window && full_slam::IsQuitKey(window->WaitKey(1))) {
                quit = true;
                break;
            }
            if (viewer.window_closed()) break;
            if ((i + 1) % 100 == 0) std::cout << "frame " << i + 1 << "/" << num_frames << "\n";
        }

        // The map is only worth looking at once playback is done: keep the
        // viewer alive until its window is closed (not after a quit key).
        if (options.viewer && !quit && !viewer.window_closed()) {
            std::cout << "Playback finished — close the 3D viewer to exit\n";
            viewer.WaitForWindowClose();
        }

        // Stop the producers before the viewer so nothing arrives after its final drain.
        slam.Stop();
        viewer.Stop();
        stats.Print(std::cout);
        if (trajectory) {
            trajectory->Poll();
            std::cout << "trajectory: " << trajectory->num_poses() << " poses written to "
                      << options.trajectory_out << "\n";
        }
        std::cout << "tracker state: " << full_slam::StateName(slam.tracker().state()) << "\n"
                  << "mapper state: " << full_slam::StateName(slam.mapper().state()) << "\n"
                  << "map: " << slam.mapper().num_keyframes() << " Frames, " << slam.mapper().num_map_points()
                  << " points\n";
        std::cout << "viewer: " << viewer.events_processed() << " events processed\n";
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << "\n";
        return EXIT_FAILURE;
    }
    return EXIT_SUCCESS;
}
