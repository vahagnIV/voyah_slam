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
using TMonocularMeasurement = TSlam::TTracker::TMonocularMeasurement;

int main(int argc, char** argv) {
    // Debug logs (e.g. initialization and bundle adjustment) are shown by the examples.
    voyahslam::Logger::SetLevel(spdlog::level::debug);
    full_slam::Options options = full_slam::ParseOptions(argc, argv);
    try {
        dataset_reader::EurocDatasetReader reader(options.dataset);
        const dataset_reader::CameraCalibration& camera = reader.camera(0);
        std::cout << "Dataset: " << options.dataset << ", " << reader.size() << " frames, "
                  << camera.width << "x" << camera.height << "\n";

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
        if (options.show) window.emplace("monocular_full_slam");
        TSlam slam;
        // Also when an exception unwinds: stop the producers, then the viewer
        // (its final drain recycles events into slam's pools), all before
        // ~Slam destroys those pools. Both Stop()s are idempotent, so the
        // explicit stops below on the normal path are fine.
        struct StopAll {
            TSlam& slam;
            voyahviz::MapViewer<TFeatureExtractor>& viewer;
            ~StopAll() {
                slam.Stop();
                viewer.Stop();
            }
        } stop_all{slam, viewer};
        // Wire before Start().
        slam.tracker().AddObserver(viewer.inbox());
        slam.mapper().AddObserver(viewer.inbox());
        viewer.Start();
        slam.Start();

        dataset_reader::PlaybackClock clock(options.rate);
        full_slam::PlaybackStats stats;
        size_t num_frames = std::min(reader.size(), options.max_frames);
        bool quit = false;
        for (size_t i = 0; i < num_frames; ++i) {
            auto lateness = clock.WaitUntil(reader.timestamp(i));
            auto start = std::chrono::steady_clock::now();

            dataset_reader::MonocularFrame frame = reader.LoadMonocular(i);
            auto loaded = std::chrono::steady_clock::now();
            auto features = extractor.DetectAndExtract(frame.image, full_slam::kNumFeatures);
            TMonocularMeasurement measurement(std::move(features), camera.K, camera.distortion,
                                              frame.timestamp);
            auto extracted = std::chrono::steady_clock::now();
            size_t num_features = static_cast<size_t>(measurement.points().cols());
            if (window) {
                window->Show(voyahviz::DrawMeasurement(frame.image, measurement,
                                                       camera.distortion));
            }
            slam.tracker().Track(std::move(measurement));

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
        std::cout << "tracker state: " << full_slam::StateName(slam.tracker().state()) << "\n"
                  << "mapper state: " << full_slam::StateName(slam.mapper().state()) << "\n";
        std::cout << "viewer: " << viewer.events_processed() << " events processed\n";
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << "\n";
        return EXIT_FAILURE;
    }
    return EXIT_SUCCESS;
}
