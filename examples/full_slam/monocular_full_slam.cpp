#include <chrono>
#include <exception>
#include <iostream>
#include <utility>

#include <dataset_reader/euroc_dataset_reader.h>
#include <dataset_reader/playback_clock.h>
#include <voyahslam/features/orb_feature_extractor.h>
#include <voyahslam/slam/slam.h>

#include "full_slam_common.h"

using namespace voyahslam;

using TFeatureExtractor = features::OrbFeatureExtractor;
using TSlam = slam::Slam<TFeatureExtractor>;
using TMonocularMeasurement = TSlam::TTracker::TMonocularMeasurement;

int main(int argc, char** argv) {
    full_slam::Options options = full_slam::ParseOptions(argc, argv);
    try {
        dataset_reader::EurocDatasetReader reader(options.dataset);
        const dataset_reader::CameraCalibration& camera = reader.camera(0);
        std::cout << "Dataset: " << options.dataset << ", " << reader.size() << " frames, "
                  << camera.width << "x" << camera.height << "\n";

        TFeatureExtractor extractor(full_slam::kScaleFactor, full_slam::kNumLevels,
                                    full_slam::kInitThresholdFast, full_slam::kMinThresholdFast);
        TSlam slam;
        slam.Start();

        dataset_reader::PlaybackClock clock(options.rate);
        full_slam::PlaybackStats stats;
        size_t num_frames = std::min(reader.size(), options.max_frames);
        for (size_t i = 0; i < num_frames; ++i) {
            auto lateness = clock.WaitUntil(reader.timestamp(i));
            auto start = std::chrono::steady_clock::now();

            dataset_reader::MonocularFrame frame = reader.LoadMonocular(i);
            auto loaded = std::chrono::steady_clock::now();
            auto features = extractor.DetectAndExtract(frame.image, full_slam::kNumFeatures);
            TMonocularMeasurement measurement(std::move(features), camera.K, camera.distortion,
                                              frame.timestamp);
            auto extracted = std::chrono::steady_clock::now();
            slam.tracker().Track(std::move(measurement));

            stats.Add(lateness, loaded - start, extracted - loaded);
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
