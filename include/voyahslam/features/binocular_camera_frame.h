#ifndef VOYAHSLAM_FEATURES_BINOCULAR_CAMERA_FRAME_H
#define VOYAHSLAM_FEATURES_BINOCULAR_CAMERA_FRAME_H

#include <voyahslam/features/camera_frame.h>
#include <voyahslam/features/stereo_rig.h>

namespace voyahslam::features {

template <typename TScalar, unsigned int Length>
class BinocularCameraFrame {
public:
    BinocularCameraFrame() = default;
    BinocularCameraFrame(
        ImageFeatures<TScalar, Length>&& left_features,
        ImageFeatures<TScalar, Length>&& right_features,
        const StereoRig& rig);

    BinocularCameraFrame(const BinocularCameraFrame&) = default;
    BinocularCameraFrame& operator=(const BinocularCameraFrame&) = default;
    BinocularCameraFrame(BinocularCameraFrame&&) = default;
    BinocularCameraFrame& operator=(BinocularCameraFrame&&) = default;

    [[nodiscard]] const CameraFrame<TScalar, Length>& left() const { return left_; }
    [[nodiscard]] const CameraFrame<TScalar, Length>& right() const { return right_; }
    [[nodiscard]] const geometry::TRowVectorF& depths() const { return depths_; }
    [[nodiscard]] const Eigen::VectorXi& right_indices() const { return right_indices_; }
    [[nodiscard]] precision_t depth(int idx) const { return depths_(idx); }

private:
    CameraFrame<TScalar, Length> left_;
    CameraFrame<TScalar, Length> right_;
    geometry::TRowVectorF depths_;
    Eigen::VectorXi right_indices_;
};

template <typename TScalar, unsigned int Length>
BinocularCameraFrame<TScalar, Length>::BinocularCameraFrame(
    ImageFeatures<TScalar, Length>&& left_features,
    ImageFeatures<TScalar, Length>&& right_features,
    const StereoRig& rig)
    : left_(std::move(left_features), rig.K_left, rig.distortion_left)
    , right_(std::move(right_features), rig.K_right, rig.distortion_right) {
    auto num_left = left_.points().cols();
    depths_ = geometry::TRowVectorF::Constant(num_left, -1.0);
    right_indices_ = Eigen::VectorXi::Constant(num_left, -1);
}

} // namespace voyahslam::features

#endif // VOYAHSLAM_FEATURES_BINOCULAR_CAMERA_FRAME_H
