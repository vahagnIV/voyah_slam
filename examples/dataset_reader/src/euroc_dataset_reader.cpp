#include <dataset_reader/euroc_dataset_reader.h>

#include <fstream>
#include <sstream>
#include <stdexcept>
#include <voyahslam/features/distortion_models/kannala_brandt_distortion_model.h>
#include <voyahslam/io/read_image.h>
#include <yaml-cpp/yaml.h>

namespace voyahslam::dataset_reader {

namespace fs = std::filesystem;

namespace {

constexpr int kMaxCameras = 2;

[[noreturn]] void Fail(const std::string& message) {
    throw std::runtime_error("EurocDatasetReader: " + message);
}

// Parses data.csv: "#timestamp [ns],filename" header (or any '#' lines).
std::vector<std::pair<int64_t, std::string>> ReadDataCsv(const fs::path& path) {
    std::ifstream file(path);
    if (!file) Fail("cannot open " + path.string());
    std::vector<std::pair<int64_t, std::string>> rows;
    std::string line;
    while (std::getline(file, line)) {
        if (!line.empty() && line.back() == '\r') line.pop_back();
        if (line.empty() || line[0] == '#') continue;
        auto comma = line.find(',');
        if (comma == std::string::npos) Fail("malformed line in " + path.string() + ": " + line);
        std::string name = line.substr(comma + 1);
        name.erase(0, name.find_first_not_of(' '));
        rows.emplace_back(std::stoll(line.substr(0, comma)), name);
    }
    return rows;
}

// yaml-cpp rejects OpenCV's "%YAML:1.0" directive used by EuRoC sensor.yaml.
YAML::Node LoadYaml(const fs::path& path) {
    std::ifstream file(path);
    if (!file) Fail("cannot open " + path.string());
    std::stringstream content;
    std::string line;
    while (std::getline(file, line)) {
        if (line.rfind("%YAML", 0) == 0) continue;
        content << line << '\n';
    }
    return YAML::Load(content.str());
}

std::vector<double> ReadDoubles(const YAML::Node& node, const std::string& what) {
    if (!node || !node.IsSequence()) Fail("missing or invalid '" + what + "'");
    std::vector<double> values;
    for (const auto& value : node) values.push_back(value.as<double>());
    return values;
}

geometry::TMatrix44F ReadRows4x4(const YAML::Node& node, const std::string& what) {
    if (!node || !node.IsSequence() || node.size() != 4) Fail("'" + what + "' must be 4x4");
    geometry::TMatrix44F T;
    for (int r = 0; r < 4; ++r) {
        auto row = ReadDoubles(node[r], what);
        if (row.size() != 4) Fail("'" + what + "' must be 4x4");
        for (int c = 0; c < 4; ++c) T(r, c) = row[c];
    }
    return T;
}

struct RawCalibration {
    std::vector<double> intrinsics;  // fx, fy, cx, cy
    std::string distortion_model;
    std::vector<double> distortion_coeffs;
    std::vector<double> resolution;  // width, height
    geometry::TMatrix44F T_cam_imu;
};

RawCalibration ReadKalibrCamera(const YAML::Node& cam) {
    RawCalibration raw;
    raw.intrinsics = ReadDoubles(cam["intrinsics"], "intrinsics");
    raw.distortion_model = cam["distortion_model"].as<std::string>();
    raw.distortion_coeffs = ReadDoubles(cam["distortion_coeffs"], "distortion_coeffs");
    raw.resolution = ReadDoubles(cam["resolution"], "resolution");
    raw.T_cam_imu = ReadRows4x4(cam["T_cam_imu"], "T_cam_imu");
    return raw;
}

RawCalibration ReadEurocSensor(const YAML::Node& sensor) {
    RawCalibration raw;
    raw.intrinsics = ReadDoubles(sensor["intrinsics"], "intrinsics");
    raw.distortion_model = sensor["distortion_model"].as<std::string>();
    raw.distortion_coeffs = ReadDoubles(sensor["distortion_coefficients"], "distortion_coefficients");
    raw.resolution = ReadDoubles(sensor["resolution"], "resolution");
    auto data = ReadDoubles(sensor["T_BS"]["data"], "T_BS.data");
    if (data.size() != 16) Fail("'T_BS.data' must have 16 values");
    geometry::TMatrix44F T_imu_cam;  // T_BS: sensor (camera) → body (IMU)
    for (int i = 0; i < 16; ++i) T_imu_cam(i / 4, i % 4) = data[i];
    raw.T_cam_imu = T_imu_cam.inverse();
    return raw;
}

} // namespace

EurocDatasetReader::EurocDatasetReader(const fs::path& root) : mav_dir_(root / "mav0") {
    if (!fs::is_directory(mav_dir_)) Fail("not a EuRoC/TUM-VI dataset (no mav0/): " + root.string());

    for (const auto& [timestamp_ns, name] : ReadDataCsv(mav_dir_ / "cam0" / "data.csv")) {
        entries_.push_back({timestamp_ns, name});
    }
    if (entries_.empty()) Fail("no frames in cam0/data.csv");

    LoadCalibration(root);

    if (has_stereo()) {
        auto right = ReadDataCsv(mav_dir_ / "cam1" / "data.csv");
        if (right.size() != entries_.size()) Fail("cam0 and cam1 have different frame counts");
        for (size_t i = 0; i < right.size(); ++i) {
            if (right[i].first != entries_[i].timestamp_ns || right[i].second != entries_[i].file_name) {
                Fail("cam0 and cam1 are not synchronized at frame " + std::to_string(i));
            }
        }
        BuildStereoRig();
    }
}

EurocDatasetReader::~EurocDatasetReader() = default;

void EurocDatasetReader::LoadCalibration(const fs::path& root) {
    std::vector<RawCalibration> raws;
    fs::path camchain = root / "dso" / "camchain.yaml";
    if (fs::exists(camchain)) {
        YAML::Node yaml = LoadYaml(camchain);
        for (int i = 0; i < kMaxCameras; ++i) {
            YAML::Node cam = yaml["cam" + std::to_string(i)];
            if (!cam) break;
            if (cam["camera_model"].as<std::string>() != "pinhole") Fail("only pinhole cameras are supported");
            raws.push_back(ReadKalibrCamera(cam));
        }
    } else {
        for (int i = 0; i < kMaxCameras; ++i) {
            fs::path sensor = mav_dir_ / ("cam" + std::to_string(i)) / "sensor.yaml";
            if (!fs::exists(sensor)) break;
            raws.push_back(ReadEurocSensor(LoadYaml(sensor)));
        }
    }
    if (raws.empty()) Fail("no camera calibration (dso/camchain.yaml or mav0/cam0/sensor.yaml)");
    if (raws.size() > 1 && !fs::exists(mav_dir_ / "cam1" / "data.csv")) raws.resize(1);

    for (const RawCalibration& raw : raws) {
        if (raw.intrinsics.size() != 4) Fail("intrinsics must be [fx, fy, cx, cy]");
        if (raw.resolution.size() != 2) Fail("resolution must be [width, height]");
        CameraCalibration camera;
        camera.K << raw.intrinsics[0], 0, raw.intrinsics[2],
                    0, raw.intrinsics[1], raw.intrinsics[3],
                    0, 0, 1;
        camera.width = static_cast<unsigned>(raw.resolution[0]);
        camera.height = static_cast<unsigned>(raw.resolution[1]);
        camera.T_cam_imu = raw.T_cam_imu;

        if (raw.distortion_model == "equidistant") {
            auto model = std::make_unique<features::distortion_models::KannalaBrandtDistortionModel>(
                raw.distortion_coeffs);
            model->PrecomputeInitialGuessLookup(camera.K, camera.width, camera.height);
            camera.distortion = model.get();
            distortion_models_.push_back(std::move(model));
        } else if (raw.distortion_model != "none") {
            Fail("unsupported distortion model '" + raw.distortion_model + "'");
        }
        cameras_.push_back(camera);
    }
}

void EurocDatasetReader::BuildStereoRig() {
    const CameraCalibration& left = cameras_[0];
    const CameraCalibration& right = cameras_[1];
    // T_left_right = T_left_imu * T_imu_right: maps right-camera points into the left camera.
    geometry::TMatrix44F T_left_right = left.T_cam_imu * right.T_cam_imu.inverse();
    stereo_rig_.K_left = left.K;
    stereo_rig_.K_right = right.K;
    stereo_rig_.R_rl = T_left_right.topLeftCorner<3, 3>();
    stereo_rig_.t_rl = T_left_right.topRightCorner<3, 1>();
    stereo_rig_.baseline = stereo_rig_.t_rl.norm();
    stereo_rig_.distortion_left = left.distortion;
    stereo_rig_.distortion_right = right.distortion;
}

double EurocDatasetReader::timestamp(size_t idx) const {
    return static_cast<double>(entries_.at(idx).timestamp_ns) * 1e-9;
}

const CameraCalibration& EurocDatasetReader::camera(size_t camera_idx) const {
    return cameras_.at(camera_idx);
}

const features::StereoRig& EurocDatasetReader::stereo_rig() const {
    if (!has_stereo()) Fail("dataset has no second camera");
    return stereo_rig_;
}

MonocularFrame EurocDatasetReader::LoadMonocular(size_t idx) const {
    return {timestamp(idx), LoadImage(0, idx)};
}

BinocularFrame EurocDatasetReader::LoadBinocular(size_t idx) const {
    if (!has_stereo()) Fail("dataset has no second camera");
    return {timestamp(idx), LoadImage(0, idx), LoadImage(1, idx)};
}

images::TWorkingImage EurocDatasetReader::LoadImage(size_t camera_idx, size_t idx) const {
    fs::path path = mav_dir_ / ("cam" + std::to_string(camera_idx)) / "data" / entries_.at(idx).file_name;
    std::ifstream file(path, std::ios::binary);
    if (!file) Fail("cannot open " + path.string());
    images::TImageU8Gray image_u8 = io::ReadImageFromStream(file);
    images::TWorkingImage image = image_u8.Convert<float>();
    image *= 1.0f / 255.0f;
    return image;
}

} // namespace voyahslam::dataset_reader
