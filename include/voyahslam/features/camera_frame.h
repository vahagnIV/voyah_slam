#ifndef VOYAHSLAM_FEATURES_CAMERA_FRAME_H
#define VOYAHSLAM_FEATURES_CAMERA_FRAME_H

#include <voyahslam/features/distortion_models/distortion_model.h>
#include <voyahslam/features/image_features.h>
#include <voyahslam/defs.h>

namespace voyahslam::features {

template <typename TScalar, unsigned int Length>
class CameraFrame final : public ImageFeatures<TScalar, Length> {
public:
    CameraFrame() = default;
    CameraFrame(ImageFeatures<TScalar, Length>&& features,
                const geometry::TMatrix33F& K,
                const distortion_models::DistortionModel* distortion_model);

    CameraFrame(const CameraFrame&) = default;
    CameraFrame& operator=(const CameraFrame&) = default;
    CameraFrame(CameraFrame&&) = default;
    CameraFrame& operator=(CameraFrame&&) = default;

    [[nodiscard]] const geometry::TMatrix33F& K() const;
    [[nodiscard]] const geometry::THomogenousPointsF& normalized_points() const;
    [[nodiscard]] geometry::THomogenousPointF normalized_point(int idx) const;
    [[nodiscard]] geometry::TPoint2DF point(int idx) const;

private:
    geometry::TMatrix33F K_;
    geometry::THomogenousPointsF normalized_points_;
};

template <typename TScalar, unsigned int Length>
CameraFrame<TScalar, Length>::CameraFrame(
    ImageFeatures<TScalar, Length>&& features,
    const geometry::TMatrix33F& K,
    const distortion_models::DistortionModel* distortion_model)
    : ImageFeatures<TScalar, Length>(std::move(features)), K_(K) {
    geometry::THomogenousPointsF homogeneous_points = this->points_.colwise().homogeneous();
    normalized_points_ = K_.inverse() * homogeneous_points;
    if (distortion_model != nullptr) {
        distortion_model->Undistort(normalized_points_);
        this->points_ = (K_ * normalized_points_).colwise().hnormalized();
    }
}

template <typename TScalar, unsigned int Length>
const geometry::TMatrix33F& CameraFrame<TScalar, Length>::K() const {
    return K_;
}

template <typename TScalar, unsigned int Length>
const geometry::THomogenousPointsF& CameraFrame<TScalar, Length>::normalized_points() const {
    return normalized_points_;
}

template <typename TScalar, unsigned int Length>
geometry::THomogenousPointF CameraFrame<TScalar, Length>::normalized_point(int idx) const {
    assert(idx < normalized_points_.cols() && "Index out of bounds in normalized_points");
    return normalized_points_.col(idx);
}

template <typename TScalar, unsigned int Length>
geometry::TPoint2DF CameraFrame<TScalar, Length>::point(int idx) const {
    assert(idx < this->points_.cols() && "Index out of bounds in points");
    return this->points_.col(idx);
}

} // namespace voyahslam::features

#endif // VOYAHSLAM_FEATURES_CAMERA_FRAME_H
