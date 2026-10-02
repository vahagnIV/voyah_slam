#include <iostream>
#include <voyahslam/features/orb_feature_extractor.h>
#include <voyahslam/slam/slam.h>
#include <voyahslam/voyahslam.h>

int main() {
    std::cout << "VoyahSLAM v" << voyahslam::version_major() << "." << voyahslam::version_minor()
              << " — full SLAM example" << std::endl;

    voyahslam::slam::Slam<voyahslam::features::OrbFeatureExtractor> slam;
    slam.Start();
    std::cout << "SLAM started" << std::endl;

    slam.Stop();
    std::cout << "SLAM stopped" << std::endl;
    return 0;
}
