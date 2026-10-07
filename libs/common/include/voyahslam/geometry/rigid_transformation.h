#ifndef VOYAHSLAM_GEOMETRY_RIGID_TRANSFORMATION_H
#define VOYAHSLAM_GEOMETRY_RIGID_TRANSFORMATION_H

#include <Eigen/Dense>

#include "voyahslam/defs.h"

namespace voyahslam::geometry {

/// Rigid transform T_ab: maps points from frame b to frame a (p_a = R p_b + t).
/// Composition: T_ac = T_ab * T_bc.
class RigidTransformation {
public:
    RigidTransformation() : rotation_(TMatrix33F::Identity()), translation_(TVector3DF::Zero()) {}

    RigidTransformation(const TMatrix33F& rotation, const TVector3DF& translation)
        : rotation_(rotation), translation_(translation) {}

    [[nodiscard]] static RigidTransformation Identity() { return RigidTransformation(); }

    [[nodiscard]] const TMatrix33F& rotation() const { return rotation_; }
    [[nodiscard]] const TVector3DF& translation() const { return translation_; }

    [[nodiscard]] RigidTransformation Inverse() const {
        const TMatrix33F rotation_inverse = rotation_.transpose();
        return RigidTransformation(rotation_inverse, -(rotation_inverse * translation_));
    }

    /// this ∘ other: applies other first, then this.
    [[nodiscard]] RigidTransformation operator*(const RigidTransformation& other) const {
        return RigidTransformation(rotation_ * other.rotation_,
                                   rotation_ * other.translation_ + translation_);
    }

    [[nodiscard]] TPoint3DF Transform(const TPoint3DF& point) const {
        return rotation_ * point + translation_;
    }

    [[nodiscard]] TPoints3DF Transform(const TPoints3DF& points) const {
        TPoints3DF result = rotation_ * points;
        result.colwise() += translation_;
        return result;
    }

    [[nodiscard]] TMatrix34F ToMatrix34() const {
        TMatrix34F matrix;
        matrix.leftCols<3>() = rotation_;
        matrix.col(3) = translation_;
        return matrix;
    }

    [[nodiscard]] TMatrix44F ToMatrix44() const {
        TMatrix44F matrix = TMatrix44F::Identity();
        matrix.topLeftCorner<3, 4>() = ToMatrix34();
        return matrix;
    }

private:
    TMatrix33F rotation_;
    TVector3DF translation_;
};

} // namespace voyahslam::geometry

#endif // VOYAHSLAM_GEOMETRY_RIGID_TRANSFORMATION_H
