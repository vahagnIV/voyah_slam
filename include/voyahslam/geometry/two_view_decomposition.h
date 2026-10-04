#ifndef VOYAHSLAM_GEOMETRY_TWO_VIEW_DECOMPOSITION_H
#define VOYAHSLAM_GEOMETRY_TWO_VIEW_DECOMPOSITION_H

#include <array>
#include <vector>

#include "voyahslam/defs.h"
#include "voyahslam/geometry/rigid_transformation.h"

namespace voyahslam::geometry {

/// One motion hypothesis from a homography: camera 1 -> camera 2 (‖t‖ = 1) and the unit
/// normal of the plane in camera 1 (oriented with a positive z component).
struct HomographyCandidate {
    RigidTransformation T_21;
    TVector3DF plane_normal;
};

/// The 4 (R, ±t) poses consistent with the essential matrix E_21 = [t]x R;
/// det(R) = +1, ‖t‖ = 1. Scale and sign of E are arbitrary.
[[nodiscard]] std::array<RigidTransformation, 4> DecomposeEssential(const TMatrix33F& a_E);

/// Faugeras' SVD decomposition (as ORB-SLAM3 ReconstructH) of a calibrated homography
/// H_21 (x2 ~ H x1, normalized coordinates): 8 candidates with det(R) = +1, ‖t‖ = 1.
/// Returns none when the singular values are (nearly) repeated, e.g. for a pure rotation.
[[nodiscard]] std::vector<HomographyCandidate> DecomposeHomography(const TMatrix33F& a_H);

} // namespace voyahslam::geometry

#endif // VOYAHSLAM_GEOMETRY_TWO_VIEW_DECOMPOSITION_H
