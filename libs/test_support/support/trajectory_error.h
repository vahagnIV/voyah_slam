#ifndef VOYAHSLAM_TESTS_SUPPORT_TRAJECTORY_ERROR_H
#define VOYAHSLAM_TESTS_SUPPORT_TRAJECTORY_ERROR_H

#include <algorithm>
#include <cmath>
#include <numbers>

#include <voyahslam/defs.h>
#include <voyahslam/geometry/rigid_transformation.h>

namespace voyahslam::test {

/// Maps ground truth (world = a synthetic sequence's frame 0) into a map's
/// frame: the map's world is its first keyframe's camera, and a monocular map
/// has an arbitrary scale. The scale is the ratio of the second keyframe's
/// estimated baseline to its true baseline (1 for a metric map).
class MapAlignment {
public:
    /// Metric map whose world is the camera of a frame with true pose `T_cw_first`.
    explicit MapAlignment(const geometry::RigidTransformation& T_cw_first)
        : T_first_w_(T_cw_first) {}

    /// Monocular map: keyframes 1 and 2 with true poses `T_cw_first`,
    /// `T_cw_second` and the second keyframe's estimated pose `estimated_second`
    /// (the first one's is the identity).
    MapAlignment(const geometry::RigidTransformation& T_cw_first,
                 const geometry::RigidTransformation& T_cw_second,
                 const geometry::RigidTransformation& estimated_second)
        : T_first_w_(T_cw_first) {
        const double true_baseline = Centre(Relative(T_cw_second)).norm();
        scale_ = Centre(estimated_second).norm() / true_baseline;
    }

    [[nodiscard]] double scale() const { return scale_; }

    /// Rotation error in degrees of an estimated T_cw against the true T_cw.
    [[nodiscard]] double RotationErrorDeg(const geometry::RigidTransformation& estimated,
                                          const geometry::RigidTransformation& truth) const {
        const geometry::TMatrix33F delta =
            estimated.rotation() * Relative(truth).rotation().transpose();
        const double cos_angle = std::clamp((delta.trace() - 1.0) / 2.0, -1.0, 1.0);
        return std::acos(cos_angle) * 180.0 / std::numbers::pi;
    }

    /// Camera-centre error in true (ground-truth) units of an estimated T_cw.
    [[nodiscard]] double CentreError(const geometry::RigidTransformation& estimated,
                                     const geometry::RigidTransformation& truth) const {
        return (Centre(estimated) / scale_ - Centre(Relative(truth))).norm();
    }

    static geometry::TVector3DF Centre(const geometry::RigidTransformation& T_cw) {
        return -(T_cw.rotation().transpose() * T_cw.translation());
    }

private:
    /// True pose with the first keyframe's camera as the world.
    [[nodiscard]] geometry::RigidTransformation Relative(const geometry::RigidTransformation& T_cw) const {
        return T_cw * T_first_w_.Inverse();
    }

    geometry::RigidTransformation T_first_w_;
    double scale_ = 1.0;
};

}  // namespace voyahslam::test

#endif  // VOYAHSLAM_TESTS_SUPPORT_TRAJECTORY_ERROR_H
