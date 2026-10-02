#ifndef VOYAHSLAM_FEATURES_MONOCULAR_MEASUREMENT_H
#define VOYAHSLAM_FEATURES_MONOCULAR_MEASUREMENT_H

#include <voyahslam/features/distortion_models/distortion_model.h>
#include <voyahslam/features/image_features.h>
#include <voyahslam/defs.h>

namespace voyahslam::features {

// Features of one camera image, undistorted, with the camera intrinsics and the
// capture time in seconds.
template <typename TScalar, unsigned int Length>
class MonocularMeasurement final : public ImageFeatures<TScalar, Length> {
public:
    MonocularMeasurement() = default;
    MonocularMeasurement(ImageFeatures<TScalar, Length>&& features,
                         const geometry::TMatrix33F& K,
                         const distortion_models::DistortionModel* distortion_model,
                         double timestamp);

    MonocularMeasurement(const MonocularMeasurement&) = default;
    MonocularMeasurement& operator=(const MonocularMeasurement&) = default;
    MonocularMeasurement(MonocularMeasurement&&) = default;
    MonocularMeasurement& operator=(MonocularMeasurement&&) = default;

    [[nodiscard]] const geometry::TMatrix33F& K() const;
    [[nodiscard]] const geometry::THomogenousPointsF& normalized_points() const;
    [[nodiscard]] geometry::THomogenousPointF normalized_point(int idx) const;
    [[nodiscard]] geometry::TPoint2DF point(int idx) const;
    [[nodiscard]] double timestamp() const { return timestamp_; }

private:
    geometry::TMatrix33F K_;
    geometry::THomogenousPointsF normalized_points_;
    double timestamp_ = 0.0;
};

template <typename TScalar, unsigned int Length>
MonocularMeasurement<TScalar, Length>::MonocularMeasurement(
    ImageFeatures<TScalar, Length>&& features,
    const geometry::TMatrix33F& K,
    const distortion_models::DistortionModel* distortion_model,
    double timestamp)
    : ImageFeatures<TScalar, Length>(std::move(features)), K_(K), timestamp_(timestamp) {
    geometry::THomogenousPointsF homogeneous_points = this->points_.colwise().homogeneous();
    normalized_points_ = K_.inverse() * homogeneous_points;
    if (distortion_model != nullptr) {
        distortion_model->Undistort(normalized_points_);
        this->points_ = (K_ * normalized_points_).colwise().hnormalized();
    }
}

template <typename TScalar, unsigned int Length>
const geometry::TMatrix33F& MonocularMeasurement<TScalar, Length>::K() const {
    return K_;
}

template <typename TScalar, unsigned int Length>
const geometry::THomogenousPointsF& MonocularMeasurement<TScalar, Length>::normalized_points() const {
    return normalized_points_;
}

template <typename TScalar, unsigned int Length>
geometry::THomogenousPointF MonocularMeasurement<TScalar, Length>::normalized_point(int idx) const {
    assert(idx < normalized_points_.cols() && "Index out of bounds in normalized_points");
    return normalized_points_.col(idx);
}

template <typename TScalar, unsigned int Length>
geometry::TPoint2DF MonocularMeasurement<TScalar, Length>::point(int idx) const {
    assert(idx < this->points_.cols() && "Index out of bounds in points");
    return this->points_.col(idx);
}

} // namespace voyahslam::features

#endif // VOYAHSLAM_FEATURES_MONOCULAR_MEASUREMENT_H
