#ifndef VOYAHSLAM_GEOMETRY_SIM3_H
#define VOYAHSLAM_GEOMETRY_SIM3_H

#include <Eigen/Dense>

#include "voyahslam/defs.h"

namespace voyahslam::geometry {

/// Similarity transform S_ab: maps points from frame b to frame a, x_a = scale · R · x_b + t.
/// A plain value (events carry it); scale > 0 and R a rotation are the caller's invariants.
struct Sim3 {
    double scale = 1.0;
    TMatrix33F R = TMatrix33F::Identity();
    TVector3DF t = TVector3DF::Zero();
};

/// S_ba from S_ab: x_b = (1/s) Rᵀ (x_a - t).
[[nodiscard]] inline Sim3 Inverse(const Sim3& S) {
    const TMatrix33F R_inverse = S.R.transpose();
    const double scale_inverse = 1.0 / S.scale;
    return Sim3{scale_inverse, R_inverse, -scale_inverse * (R_inverse * S.t)};
}

/// a ∘ b: applies b first, then a (S_ac = Compose(S_ab, S_bc)).
[[nodiscard]] inline Sim3 Compose(const Sim3& a, const Sim3& b) {
    return Sim3{a.scale * b.scale, a.R * b.R, a.scale * (a.R * b.t) + a.t};
}

[[nodiscard]] inline TPoint3DF Apply(const Sim3& S, const TPoint3DF& x) {
    return S.scale * (S.R * x) + S.t;
}

}  // namespace voyahslam::geometry

#endif  // VOYAHSLAM_GEOMETRY_SIM3_H
