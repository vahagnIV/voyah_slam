#ifndef VOYAHSLAM_OPTIMIZATION_ESSENTIAL_GRAPH_H
#define VOYAHSLAM_OPTIMIZATION_ESSENTIAL_GRAPH_H

#include <cstddef>
#include <optional>
#include <utility>
#include <vector>

#include "voyahslam/geometry/rigid_transformation.h"
#include "voyahslam/geometry/sim3.h"
#include "voyahslam/map/frame_id.h"

namespace voyahslam::optimization {

/// The Sim3 pose graph of a loop correction (ORB-SLAM3 Optimizer::OptimizeEssentialGraph,
/// visual). Per Frame i one vertex S_iw, starting from corrected_S_cw[i] where set (the loop
/// window) and from T_cw[i] with scale 1 otherwise. Edge (i, j) measures S_ji = S_jw · S_iw⁻¹:
/// - `edges` (the spanning tree, the loop edges, the covisibility edges with enough shared
///   points) from the non-corrected poses T_cw;
/// - `loop_connections` (the links the correction created between window Frames and Frames
///   across the loop) from the initial values (corrected where set, T_cw otherwise).
/// Each unordered Frame pair gives one edge: loop_connections first, then edges, and a pair
/// already added is skipped.
struct EssentialGraphProblem {
    std::vector<map::FrameId> frames;                         ///< Which Frame each index is (for the caller).
    std::vector<geometry::RigidTransformation> T_cw;          ///< Current (non-corrected) poses.
    std::vector<std::optional<geometry::Sim3>> corrected_S_cw;  ///< Set for the loop window.
    std::vector<std::pair<size_t, size_t>> edges;             ///< Index pairs, non-corrected measurement.
    std::vector<std::pair<size_t, size_t>> loop_connections;  ///< Index pairs, initial-value measurement.
    size_t fixed = 0;        ///< Index of the vertex held constant.
    bool fix_scale = false;  ///< Stereo: every vertex keeps its start scale.
    int iterations = 20;     ///< Levenberg-Marquardt iterations.
};

struct EssentialGraphResult {
    bool success = false;
    /// Optimized S_cw per Frame; empty on failure. The caller folds the scale into the SE3 pose
    /// (R, t / scale).
    std::vector<geometry::Sim3> S_cw;
};

/// Optimizes the graph with identity information on every edge and the fixed vertex held
/// constant. Fails without optimizing on empty or invalid input: mismatched sizes, an index out
/// of range, a self edge, a non-finite pose, a corrected scale <= 0, iterations <= 0. A graph
/// without edges returns the start values.
[[nodiscard]] EssentialGraphResult OptimizeEssentialGraph(const EssentialGraphProblem& problem);

}  // namespace voyahslam::optimization

#endif  // VOYAHSLAM_OPTIMIZATION_ESSENTIAL_GRAPH_H
