#include <chrono>
#include <exception>
#include <iostream>
#include <utility>

#include <dataset_reader/euroc_dataset_reader.h>
#include <dataset_reader/playback_clock.h>
#include <opencv2/core.hpp>
#include <optional>
#include <voyahviz/image_conversion.h>
#include <voyahviz/image_window.h>
#include <voyahviz/keypoint_drawing.h>
#include <voyahslam/features/orb_feature_extractor.h>
#include <voyahslam/slam/slam.h>

#include "full_slam_common.h"

using namespace voyahslam;

using TFeatureExtractor = features::OrbFeatureExtractor;
using TSlam = slam::Slam<TFeatureExtractor>;
using TBinocularMeasurement = TSlam::TTracker::TBinocularMeasurement;

int main(int argc, char** argv) {
    full_slam::Options options = full_slam::ParseOptions(argc, argv);
    try {
        dataset_reader::EurocDatasetReader reader(options.dataset);
        if (!reader.has_stereo()) {
            std::cerr << "Error: dataset has no second camera\n";
            return EXIT_FAILURE;
        }
        const features::StereoRig& rig = reader.stereo_rig();
        std::cout << "Dataset: " << options.dataset << ", " << reader.size()
                  << " stereo frames, baseline " << rig.baseline << " m\n";

        TFeatureExtractor extractor(full_slam::kScaleFactor, full_slam::kNumLevels,
                                    full_slam::kInitThresholdFast, full_slam::kMinThresholdFast);
        std::optional<voyahviz::ImageWindow> window;
        if (options.show) window.emplace("binocular_full_slam");

        TSlam slam;
        slam.Start();

        dataset_reader::PlaybackClock clock(options.rate);
        full_slam::PlaybackStats stats;
        size_t num_frames = std::min(reader.size(), options.max_frames);
        for (size_t i = 0; i < num_frames; ++i) {
            auto lateness = clock.WaitUntil(reader.timestamp(i));
            auto start = std::chrono::steady_clock::now();

            dataset_reader::BinocularFrame frame = reader.LoadBinocular(i);
            auto loaded = std::chrono::steady_clock::now();
            auto left = extractor.DetectAndExtract(frame.left, full_slam::kNumFeatures);
            auto right = extractor.DetectAndExtract(frame.right, full_slam::kNumFeatures);
            TBinocularMeasurement measurement(std::move(left), std::move(right), rig,
                                              frame.timestamp);
            auto extracted = std::chrono::steady_clock::now();
            if (window) {
                cv::Mat side_by_side;
                cv::hconcat(voyahviz::DrawMeasurement(voyahviz::ToCvMat(frame.left), measurement.left(),
                                                      rig.distortion_left),
                            voyahviz::DrawMeasurement(voyahviz::ToCvMat(frame.right), measurement.right(),
                                                      rig.distortion_right),
                            side_by_side);
                window->Show(side_by_side);
            }
            slam.tracker().Track(std::move(measurement));

            stats.Add(lateness, loaded - start, extracted - loaded);
            if (window && full_slam::IsQuitKey(window->WaitKey(1))) break;
            if ((i + 1) % 100 == 0) std::cout << "frame " << i + 1 << "/" << num_frames << "\n";
        }

        slam.Stop();
        stats.Print(std::cout);
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << "\n";
        return EXIT_FAILURE;
    }
    return EXIT_SUCCESS;
}
