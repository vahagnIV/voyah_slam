#ifndef VOYAHSLAM_EXAMPLES_DATASET_READER_EUROC_DATASET_READER_H
#define VOYAHSLAM_EXAMPLES_DATASET_READER_EUROC_DATASET_READER_H

#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <memory>
#include <string>
#include <vector>
#include <voyahslam/defs.h>
#include <voyahslam/features/distortion_models/distortion_model.h>
#include <voyahslam/features/stereo_rig.h>
#include <voyahslam/images/image.h>

namespace voyahslam::dataset_reader {

// Calibration of one camera of the dataset.
struct CameraCalibration {
    geometry::TMatrix33F K;
    // Owned by the reader; nullptr means the images are already undistorted.
    const features::distortion_models::DistortionModel* distortion = nullptr;
    unsigned width = 0;
    unsigned height = 0;
    // Transforms points from the IMU (body) frame into this camera's frame.
    geometry::TMatrix44F T_cam_imu;
};

struct MonocularFrame {
    double timestamp;  // seconds
    images::TWorkingImage image;
};

struct BinocularFrame {
    double timestamp;  // seconds
    images::TWorkingImage left;
    images::TWorkingImage right;
};

// Reads EuRoC-layout datasets (EuRoC MAV, TUM-VI):
//   <root>/mav0/cam{0,1}/data.csv, <root>/mav0/cam{0,1}/data/<timestamp>.png
// Calibration comes from <root>/dso/camchain.yaml (Kalibr, as shipped with
// TUM-VI) if present, otherwise from <root>/mav0/cam{0,1}/sensor.yaml (EuRoC).
// Only the "equidistant" (Kannala-Brandt) distortion model is supported so far.
//
// The reader does no timing and starts no threads: frames are loaded on
// demand, by index, on the caller's thread. Images are returned as 8-bit
// grayscale (images::TWorkingImage); 16-bit PNGs are reduced to 8 bits.
class EurocDatasetReader {
public:
    explicit EurocDatasetReader(const std::filesystem::path& root);
    ~EurocDatasetReader();
    EurocDatasetReader(const EurocDatasetReader&) = delete;
    EurocDatasetReader& operator=(const EurocDatasetReader&) = delete;

    // Number of frames (of cam0).
    [[nodiscard]] size_t size() const { return entries_.size(); }
    [[nodiscard]] bool has_stereo() const { return cameras_.size() > 1; }
    [[nodiscard]] double timestamp(size_t idx) const;
    [[nodiscard]] const CameraCalibration& camera(size_t camera_idx) const;
    // cam0 = left, cam1 = right. Distortion pointers stay valid while the
    // reader is alive. Requires has_stereo().
    [[nodiscard]] const features::StereoRig& stereo_rig() const;

    [[nodiscard]] MonocularFrame LoadMonocular(size_t idx) const;
    // Requires has_stereo().
    [[nodiscard]] BinocularFrame LoadBinocular(size_t idx) const;

private:
    struct Entry {
        int64_t timestamp_ns;
        std::string file_name;
    };

    std::filesystem::path mav_dir_;
    std::vector<Entry> entries_;  // cam0; cam1 is verified to match
    std::vector<CameraCalibration> cameras_;
    std::vector<std::unique_ptr<features::distortion_models::DistortionModel>> distortion_models_;
    features::StereoRig stereo_rig_;

    void LoadCalibration(const std::filesystem::path& root);
    void BuildStereoRig();
    [[nodiscard]] images::TWorkingImage LoadImage(size_t camera_idx, size_t idx) const;
};

} // namespace voyahslam::dataset_reader

#endif // VOYAHSLAM_EXAMPLES_DATASET_READER_EUROC_DATASET_READER_H
