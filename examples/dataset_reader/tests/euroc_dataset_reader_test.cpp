#include <dataset_reader/euroc_dataset_reader.h>

#include <gtest/gtest.h>

#define STB_IMAGE_WRITE_IMPLEMENTATION
#include <stb_image_write.h>

#include <filesystem>
#include <fstream>
#include <string>
#include <vector>

namespace voyahslam::dataset_reader {
namespace {

namespace fs = std::filesystem;

constexpr int kWidth = 8;
constexpr int kHeight = 6;

// Builds a minimal TUM-VI / EuRoC style dataset in a temporary directory.
class SyntheticDataset {
public:
    SyntheticDataset() {
        root_ = fs::temp_directory_path() /
                ("voyahslam_dataset_test_" + std::to_string(reinterpret_cast<uintptr_t>(this)));
        fs::remove_all(root_);
    }
    ~SyntheticDataset() { fs::remove_all(root_); }

    void AddCamera(int camera, const std::vector<int64_t>& timestamps, uint8_t value) {
        fs::path dir = root_ / "mav0" / ("cam" + std::to_string(camera));
        fs::create_directories(dir / "data");
        std::ofstream csv(dir / "data.csv");
        csv << "#timestamp [ns],filename\n";
        std::vector<uint8_t> pixels(kWidth * kHeight, value);
        for (int64_t t : timestamps) {
            std::string name = std::to_string(t) + ".png";
            csv << t << "," << name << "\r\n";
            stbi_write_png((dir / "data" / name).c_str(), kWidth, kHeight, 1, pixels.data(), kWidth);
        }
    }

    void WriteFile(const fs::path& relative, const std::string& content) {
        fs::create_directories((root_ / relative).parent_path());
        std::ofstream(root_ / relative) << content;
    }

    void WriteKalibrStereo() {
        WriteFile("dso/camchain.yaml", R"(cam0:
  T_cam_imu:
  - [1.0, 0.0, 0.0, 0.0]
  - [0.0, 1.0, 0.0, 0.0]
  - [0.0, 0.0, 1.0, 0.0]
  - [0.0, 0.0, 0.0, 1.0]
  camera_model: pinhole
  distortion_coeffs: [0.01, -0.01, 0.005, -0.001]
  distortion_model: equidistant
  intrinsics: [380.0, 381.0, 4.0, 3.0]
  resolution: [8, 6]
cam1:
  T_cam_imu:
  - [1.0, 0.0, 0.0, -0.1]
  - [0.0, 1.0, 0.0, 0.0]
  - [0.0, 0.0, 1.0, 0.0]
  - [0.0, 0.0, 0.0, 1.0]
  camera_model: pinhole
  distortion_coeffs: [0.02, -0.01, 0.005, -0.001]
  distortion_model: equidistant
  intrinsics: [379.0, 379.5, 4.5, 3.5]
  resolution: [8, 6]
)");
    }

    [[nodiscard]] const fs::path& root() const { return root_; }

private:
    fs::path root_;
};

TEST(EurocDatasetReaderTest, ReadsKalibrStereoCalibration) {
    SyntheticDataset dataset;
    dataset.AddCamera(0, {1000000000, 1050000000}, 255);
    dataset.AddCamera(1, {1000000000, 1050000000}, 0);
    dataset.WriteKalibrStereo();

    EurocDatasetReader reader(dataset.root());
    ASSERT_EQ(reader.size(), 2u);
    ASSERT_TRUE(reader.has_stereo());

    const CameraCalibration& left = reader.camera(0);
    EXPECT_DOUBLE_EQ(left.K(0, 0), 380.0);
    EXPECT_DOUBLE_EQ(left.K(1, 1), 381.0);
    EXPECT_DOUBLE_EQ(left.K(0, 2), 4.0);
    EXPECT_DOUBLE_EQ(left.K(1, 2), 3.0);
    EXPECT_EQ(left.width, 8u);
    EXPECT_EQ(left.height, 6u);
    EXPECT_NE(left.distortion, nullptr);

    // Right camera sits 0.1 m along +x of the left one.
    const features::StereoRig& rig = reader.stereo_rig();
    EXPECT_TRUE(rig.R_rl.isIdentity());
    EXPECT_TRUE(rig.t_rl.isApprox(geometry::TVector3DF(0.1, 0.0, 0.0)));
    EXPECT_DOUBLE_EQ(rig.baseline, 0.1);
    EXPECT_EQ(rig.distortion_left, reader.camera(0).distortion);
    EXPECT_EQ(rig.distortion_right, reader.camera(1).distortion);
    EXPECT_DOUBLE_EQ(rig.K_right(0, 0), 379.0);
}

TEST(EurocDatasetReaderTest, LoadsFramesWithTimestampsInSecondsAndUnitRangeImages) {
    SyntheticDataset dataset;
    dataset.AddCamera(0, {1000000000, 1050000000}, 255);
    dataset.AddCamera(1, {1000000000, 1050000000}, 51);
    dataset.WriteKalibrStereo();
    EurocDatasetReader reader(dataset.root());

    EXPECT_DOUBLE_EQ(reader.timestamp(1), 1.05);

    MonocularFrame mono = reader.LoadMonocular(1);
    EXPECT_DOUBLE_EQ(mono.timestamp, 1.05);
    EXPECT_EQ(mono.image.Width(), 8u);
    EXPECT_EQ(mono.image.Height(), 6u);
    EXPECT_FLOAT_EQ(mono.image.At(2, 3, 0), 1.0f);

    BinocularFrame stereo = reader.LoadBinocular(0);
    EXPECT_DOUBLE_EQ(stereo.timestamp, 1.0);
    EXPECT_FLOAT_EQ(stereo.left.At(0, 0, 0), 1.0f);
    EXPECT_FLOAT_EQ(stereo.right.At(0, 0, 0), 0.2f);
}

TEST(EurocDatasetReaderTest, MonocularWhenCam1HasNoImages) {
    SyntheticDataset dataset;
    dataset.AddCamera(0, {1000000000}, 10);
    dataset.WriteKalibrStereo();

    EurocDatasetReader reader(dataset.root());
    EXPECT_FALSE(reader.has_stereo());
    EXPECT_THROW((void)reader.stereo_rig(), std::runtime_error);
    EXPECT_THROW((void)reader.LoadBinocular(0), std::runtime_error);
}

TEST(EurocDatasetReaderTest, ReadsEurocSensorYamlWithOpenCvDirective) {
    SyntheticDataset dataset;
    dataset.AddCamera(0, {1000000000}, 10);
    dataset.WriteFile("mav0/cam0/sensor.yaml", R"(%YAML:1.0
sensor_type: camera
T_BS:
  cols: 4
  rows: 4
  data: [1.0, 0.0, 0.0, 0.5,
         0.0, 1.0, 0.0, 0.0,
         0.0, 0.0, 1.0, 0.0,
         0.0, 0.0, 0.0, 1.0]
resolution: [8, 6]
camera_model: pinhole
intrinsics: [458.6, 457.3, 4.0, 3.0]
distortion_model: equidistant
distortion_coefficients: [0.0, 0.0, 0.0, 0.0]
)");

    EurocDatasetReader reader(dataset.root());
    EXPECT_FALSE(reader.has_stereo());
    EXPECT_DOUBLE_EQ(reader.camera(0).K(0, 0), 458.6);
    // T_BS maps camera → body; T_cam_imu is its inverse.
    EXPECT_DOUBLE_EQ(reader.camera(0).T_cam_imu(0, 3), -0.5);
}

TEST(EurocDatasetReaderTest, RejectsUnsupportedDistortionModel) {
    SyntheticDataset dataset;
    dataset.AddCamera(0, {1000000000}, 10);
    dataset.WriteFile("mav0/cam0/sensor.yaml", R"(T_BS:
  data: [1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1]
resolution: [8, 6]
intrinsics: [458.6, 457.3, 4.0, 3.0]
distortion_model: radial-tangential
distortion_coefficients: [-0.28, 0.07, 0.0002, 0.00002]
)");
    EXPECT_THROW(EurocDatasetReader reader(dataset.root()), std::runtime_error);
}

TEST(EurocDatasetReaderTest, RejectsUnsynchronizedCameras) {
    SyntheticDataset dataset;
    dataset.AddCamera(0, {1000000000, 1050000000}, 10);
    dataset.AddCamera(1, {1000000000, 1060000000}, 10);
    dataset.WriteKalibrStereo();
    EXPECT_THROW(EurocDatasetReader reader(dataset.root()), std::runtime_error);
}

TEST(EurocDatasetReaderTest, RejectsDirectoryWithoutMav0) {
    SyntheticDataset dataset;
    fs::create_directories(dataset.root());
    EXPECT_THROW(EurocDatasetReader reader(dataset.root()), std::runtime_error);
}

} // namespace
} // namespace voyahslam::dataset_reader
