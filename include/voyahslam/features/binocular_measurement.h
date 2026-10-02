#ifndef VOYAHSLAM_FEATURES_BINOCULAR_MEASUREMENT_H
#define VOYAHSLAM_FEATURES_BINOCULAR_MEASUREMENT_H

#include <voyahslam/features/monocular_measurement.h>
#include <voyahslam/features/stereo_rig.h>

namespace voyahslam::features {

// Simultaneous left/right measurements of a stereo rig plus stereo matching
// results. The capture time is the left camera's.
template <typename TScalar, unsigned int Length>
class BinocularMeasurement {
public:
    BinocularMeasurement() = default;
    BinocularMeasurement(
        ImageFeatures<TScalar, Length>&& left_features,
        ImageFeatures<TScalar, Length>&& right_features,
        const StereoRig& rig,
        double timestamp);

    BinocularMeasurement(const BinocularMeasurement&) = default;
    BinocularMeasurement& operator=(const BinocularMeasurement&) = default;
    BinocularMeasurement(BinocularMeasurement&&) = default;
    BinocularMeasurement& operator=(BinocularMeasurement&&) = default;

    [[nodiscard]] const MonocularMeasurement<TScalar, Length>& left() const { return left_; }
    [[nodiscard]] const MonocularMeasurement<TScalar, Length>& right() const { return right_; }
    [[nodiscard]] const geometry::TRowVectorF& depths() const { return depths_; }
    [[nodiscard]] const Eigen::VectorXi& right_indices() const { return right_indices_; }
    [[nodiscard]] precision_t depth(int idx) const { return depths_(idx); }
    [[nodiscard]] double timestamp() const { return left_.timestamp(); }

private:
    MonocularMeasurement<TScalar, Length> left_;
    MonocularMeasurement<TScalar, Length> right_;
    geometry::TRowVectorF depths_;
    Eigen::VectorXi right_indices_;
};

template <typename TScalar, unsigned int Length>
BinocularMeasurement<TScalar, Length>::BinocularMeasurement(
    ImageFeatures<TScalar, Length>&& left_features,
    ImageFeatures<TScalar, Length>&& right_features,
    const StereoRig& rig,
    double timestamp)
    : left_(std::move(left_features), rig.K_left, rig.distortion_left, timestamp)
    , right_(std::move(right_features), rig.K_right, rig.distortion_right, timestamp) {
    auto num_left = left_.points().cols();
    depths_ = geometry::TRowVectorF::Constant(num_left, -1.0);
    right_indices_ = Eigen::VectorXi::Constant(num_left, -1);
}

} // namespace voyahslam::features

#endif // VOYAHSLAM_FEATURES_BINOCULAR_MEASUREMENT_H
