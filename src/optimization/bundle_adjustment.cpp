#include "voyahslam/optimization/bundle_adjustment.h"

#include <cmath>
#include <istream>
#include <memory>
#include <ostream>
#include <vector>

#include <g2o/core/base_binary_edge.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/robust_kernel_impl.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/solvers/eigen/linear_solver_eigen.h>
#include <g2o/types/sba/vertex_se3_expmap.h>
#include <g2o/types/slam3d/vertex_pointxyz.h>

#include "optimization/projection.h"

namespace voyahslam::optimization {
namespace {

using geometry::RigidTransformation;
using geometry::TPoint2DF;
using geometry::TPoint3DF;

g2o::SE3Quat ToSE3Quat(const RigidTransformation& pose) {
    return g2o::SE3Quat(pose.rotation(), pose.translation());
}

RigidTransformation FromSE3Quat(const g2o::SE3Quat& pose) {
    return RigidTransformation(pose.rotation().toRotationMatrix(), pose.translation());
}

/// Normalized-plane reprojection error of a world point (vertex 0) seen by a T_cw pose
/// (vertex 1): e = pi(R X + t) - measurement.
class EdgeNormalizedProjection
    : public g2o::BaseBinaryEdge<2, Eigen::Vector2d, g2o::VertexPointXYZ, g2o::VertexSE3Expmap> {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    bool read(std::istream& /*is*/) override { return false; }
    bool write(std::ostream& /*os*/) const override { return false; }

    void computeError() override {
        _error = ProjectNormalized(Pose(), Point()) - _measurement;
    }

    void linearizeOplus() override {
        Eigen::Matrix<precision_t, 2, 6> pose_jacobian;
        Eigen::Matrix<precision_t, 2, 3> point_jacobian;
        ProjectionJacobians(Pose(), Point(), pose_jacobian, point_jacobian);
        _jacobianOplusXi = point_jacobian;
        _jacobianOplusXj = pose_jacobian;
    }

    /// Camera-frame depth of the point at the current estimates.
    [[nodiscard]] double Depth() const { return Pose().Transform(Point()).z(); }

private:
    [[nodiscard]] TPoint3DF Point() const { return vertexXn<0>()->estimate(); }
    [[nodiscard]] RigidTransformation Pose() const {
        return FromSE3Quat(vertexXn<1>()->estimate());
    }
};

bool IsValid(const BaProblem& problem) {
    if (problem.keyframe_poses.empty() || problem.points.cols() == 0 ||
        problem.observations.empty() ||
        problem.keyframe_fixed.size() != problem.keyframe_poses.size() ||
        !problem.points.allFinite()) {
        return false;
    }
    for (const RigidTransformation& pose : problem.keyframe_poses) {
        if (!pose.rotation().allFinite() || !pose.translation().allFinite()) {
            return false;
        }
    }
    const auto num_points = static_cast<size_t>(problem.points.cols());
    for (const BaObservation& observation : problem.observations) {
        if (observation.keyframe >= problem.keyframe_poses.size() ||
            observation.point >= num_points || !observation.measurement.allFinite() ||
            !std::isfinite(observation.sigma) || observation.sigma <= 0.0) {
            return false;
        }
    }
    return true;
}

}  // namespace

BaResult BundleAdjust(const BaProblem& problem, const BundleAdjustmentOptions& options) {
    BaResult result;
    if (!IsValid(problem) || options.robust_iterations < 0 || options.iterations < 0) {
        return result;
    }

    using BlockSolver = g2o::BlockSolver_6_3;
    auto linear_solver = std::make_unique<g2o::LinearSolverEigen<BlockSolver::PoseMatrixType>>();
    g2o::SparseOptimizer optimizer;
    // The optimizer owns the algorithm, the vertices and the edges (and the edges their kernels).
    optimizer.setAlgorithm(new g2o::OptimizationAlgorithmLevenberg(
        std::make_unique<BlockSolver>(std::move(linear_solver))));
    optimizer.setVerbose(false);

    // Only vertices that take part in an observation enter the graph.
    const size_t num_keyframes = problem.keyframe_poses.size();
    const auto num_points = static_cast<size_t>(problem.points.cols());
    std::vector<g2o::VertexSE3Expmap*> pose_vertices(num_keyframes, nullptr);
    std::vector<g2o::VertexPointXYZ*> point_vertices(num_points, nullptr);
    std::vector<EdgeNormalizedProjection*> edges;
    edges.reserve(problem.observations.size());
    for (const BaObservation& observation : problem.observations) {
        g2o::VertexSE3Expmap*& pose_vertex = pose_vertices[observation.keyframe];
        if (pose_vertex == nullptr) {
            pose_vertex = new g2o::VertexSE3Expmap();
            pose_vertex->setId(static_cast<int>(observation.keyframe));
            pose_vertex->setEstimate(ToSE3Quat(problem.keyframe_poses[observation.keyframe]));
            pose_vertex->setFixed(problem.keyframe_fixed[observation.keyframe]);
            optimizer.addVertex(pose_vertex);
        }
        g2o::VertexPointXYZ*& point_vertex = point_vertices[observation.point];
        if (point_vertex == nullptr) {
            point_vertex = new g2o::VertexPointXYZ();
            point_vertex->setId(static_cast<int>(num_keyframes + observation.point));
            point_vertex->setEstimate(
                problem.points.col(static_cast<Eigen::Index>(observation.point)));
            point_vertex->setMarginalized(true);
            optimizer.addVertex(point_vertex);
        }

        auto* edge = new EdgeNormalizedProjection();
        edge->setVertex(0, point_vertex);
        edge->setVertex(1, pose_vertex);
        edge->setMeasurement(observation.measurement);
        edge->setInformation(Eigen::Matrix2d::Identity() /
                             (observation.sigma * observation.sigma));
        if (options.robust) {
            auto* kernel = new g2o::RobustKernelHuber();
            kernel->setDelta(std::sqrt(options.huber_chi2));
            edge->setRobustKernel(kernel);
        }
        optimizer.addEdge(edge);
        edges.push_back(edge);
    }

    // Runs one pass over the level-0 edges; false if the optimizer fails.
    const auto run_pass = [&](int iterations) {
        if (iterations == 0) {
            return true;
        }
        if (!optimizer.initializeOptimization(0)) {
            return false;
        }
        const int performed = optimizer.optimize(iterations);
        if (performed <= 0) {
            return false;
        }
        result.iterations += performed;
        return true;
    };

    for (EdgeNormalizedProjection* edge : edges) {
        edge->computeError();
        result.initial_cost += edge->chi2();
    }

    if (options.robust) {
        if (!run_pass(options.robust_iterations)) {
            return result;
        }
        // Deactivate the outliers of the robust pass; the plain pass runs without a kernel.
        for (EdgeNormalizedProjection* edge : edges) {
            edge->computeError();
            if (edge->chi2() > options.huber_chi2 || !(edge->Depth() > 0.0)) {
                edge->setLevel(1);
            }
            edge->setRobustKernel(nullptr);
        }
        if (!run_pass(options.iterations)) {
            return result;
        }
    } else if (!run_pass(options.robust_iterations + options.iterations)) {
        return result;
    }

    // Fixed keyframes keep their input pose exactly; free ones are read back.
    result.keyframe_poses = problem.keyframe_poses;
    for (size_t k = 0; k < num_keyframes; ++k) {
        if (pose_vertices[k] != nullptr && !problem.keyframe_fixed[k]) {
            result.keyframe_poses[k] = FromSE3Quat(pose_vertices[k]->estimate());
        }
    }
    result.points = problem.points;
    for (size_t i = 0; i < num_points; ++i) {
        if (point_vertices[i] != nullptr) {
            result.points.col(static_cast<Eigen::Index>(i)) = point_vertices[i]->estimate();
        }
    }

    // chi2 / inlier for every edge, the deactivated ones included.
    result.chi2.reserve(edges.size());
    result.inlier.reserve(edges.size());
    for (EdgeNormalizedProjection* edge : edges) {
        edge->computeError();
        const double chi2 = edge->chi2();
        result.chi2.push_back(chi2);
        result.inlier.push_back(chi2 <= options.huber_chi2 && edge->Depth() > 0.0);
        if (edge->level() == 0) {
            result.final_cost += chi2;
        }
    }

    result.success = result.points.allFinite();
    for (const RigidTransformation& pose : result.keyframe_poses) {
        result.success = result.success && pose.rotation().allFinite() &&
                         pose.translation().allFinite();
    }
    return result;
}

}  // namespace voyahslam::optimization
