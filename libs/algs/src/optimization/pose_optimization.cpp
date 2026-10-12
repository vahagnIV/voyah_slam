#include "voyahslam/optimization/pose_optimization.h"

#include <cmath>
#include <istream>
#include <memory>
#include <ostream>
#include <vector>

#include <g2o/core/base_unary_edge.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/robust_kernel_impl.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/solvers/eigen/linear_solver_eigen.h>
#include <g2o/types/sba/vertex_se3_expmap.h>

#include "optimization/projection.h"
#include "voyahslam/logging/logger.h"

namespace voyahslam::optimization {
namespace {

using geometry::Camera;
using geometry::RigidTransformation;
using geometry::TPoint2DF;
using geometry::TPoint3DF;

g2o::SE3Quat ToSE3Quat(const RigidTransformation& pose) {
    return g2o::SE3Quat(pose.rotation(), pose.translation());
}

RigidTransformation FromSE3Quat(const g2o::SE3Quat& pose) {
    return RigidTransformation(pose.rotation().toRotationMatrix(), pose.translation());
}

bool IsFinite(const RigidTransformation& pose) {
    return pose.rotation().allFinite() && pose.translation().allFinite();
}

/// Edges refer to their cameras, never copy them: a Camera holds a shared_ptr to its distortion
/// model, so a copy per edge would cost two contended atomic operations. The cameras are the const
/// arguments of OptimizePose (left, *stereo), which outlive the optimizer and its edges.

/// Left-camera raw-pixel reprojection error of a fixed world point seen by the T_cw pose
/// vertex: e = left.Project(T_cw X) - left measurement.
class EdgeMonoPose : public g2o::BaseUnaryEdge<2, Eigen::Vector2d, g2o::VertexSE3Expmap> {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    EdgeMonoPose(const TPoint3DF& point, const Camera& camera) : point_(point), camera_(camera) {}

    bool read(std::istream& /*is*/) override { return false; }
    bool write(std::ostream& /*os*/) const override { return false; }

    void computeError() override { _error = ProjectPixel(camera_, Pose(), point_) - _measurement; }

    void linearizeOplus() override {
        Eigen::Matrix<precision_t, 2, 6> pose_jacobian;
        Eigen::Matrix<precision_t, 2, 3> point_jacobian;
        PixelJacobians(camera_, Pose(), point_, pose_jacobian, point_jacobian);
        _jacobianOplusXi = pose_jacobian;
    }

    /// Positive camera-frame depth at the current estimate.
    [[nodiscard]] bool InFront() const { return Pose().Transform(point_).z() > 0.0; }

private:
    [[nodiscard]] RigidTransformation Pose() const {
        return FromSE3Quat(vertexXn<0>()->estimate());
    }

    TPoint3DF point_;
    const Camera& camera_;
};

/// Stereo error of a fixed world point: the left rows of EdgeMonoPose stacked over the
/// right-camera rows right.Project(T_rl T_cw X) - right; measurement = (left, right), raw px.
class EdgeStereoPose : public g2o::BaseUnaryEdge<4, Eigen::Vector4d, g2o::VertexSE3Expmap> {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    EdgeStereoPose(const TPoint3DF& point, const Camera& left, const StereoCameras& stereo)
        : point_(point), left_(left), right_(stereo.right), T_rl_(stereo.T_rl) {}

    bool read(std::istream& /*is*/) override { return false; }
    bool write(std::ostream& /*os*/) const override { return false; }

    void computeError() override {
        const RigidTransformation pose = Pose();
        _error.head<2>() = ProjectPixel(left_, pose, point_) - _measurement.head<2>();
        _error.tail<2>() = ProjectPixelRight(right_, pose, T_rl_, point_) - _measurement.tail<2>();
    }

    void linearizeOplus() override {
        const RigidTransformation pose = Pose();
        Eigen::Matrix<precision_t, 2, 6> left_jacobian;
        Eigen::Matrix<precision_t, 2, 3> point_jacobian;
        Eigen::Matrix<precision_t, 2, 6> right_jacobian;
        PixelJacobians(left_, pose, point_, left_jacobian, point_jacobian);
        PixelJacobianRightPose(right_, pose, T_rl_, point_, right_jacobian);
        _jacobianOplusXi.topRows<2>() = left_jacobian;
        _jacobianOplusXi.bottomRows<2>() = right_jacobian;
    }

    /// Positive depth in both cameras at the current estimate.
    [[nodiscard]] bool InFront() const {
        const TPoint3DF xc = Pose().Transform(point_);
        return xc.z() > 0.0 && T_rl_.Transform(xc).z() > 0.0;
    }

private:
    [[nodiscard]] RigidTransformation Pose() const {
        return FromSE3Quat(vertexXn<0>()->estimate());
    }

    TPoint3DF point_;
    const Camera& left_;
    const Camera& right_;
    RigidTransformation T_rl_;
};

bool IsValid(const Camera& left, const std::optional<StereoCameras>& stereo,
             const std::vector<PoseObservation>& observations,
             const PoseOptimizationOptions& options) {
    if (observations.empty() || options.rounds <= 0 || options.iterations_per_round <= 0 ||
        !(options.mono_chi2 > 0.0) || !(options.stereo_chi2 > 0.0) || !left.K.allFinite()) {
        return false;
    }
    if (stereo && (!IsFinite(stereo->T_rl) || !stereo->right.K.allFinite())) {
        return false;
    }
    for (const PoseObservation& observation : observations) {
        if (!observation.point.allFinite() || !observation.left.allFinite() ||
            !std::isfinite(observation.sigma) || observation.sigma <= 0.0) {
            return false;
        }
        if (observation.right && (!stereo || !observation.right->allFinite())) {
            return false;
        }
    }
    return true;
}

// One logger for all calls; created on first use (after the app set the level).
const std::shared_ptr<spdlog::logger>& Log() {
    static const std::shared_ptr<spdlog::logger> log = Logger::CreateLogger("OptimizePose");
    return log;
}

}  // namespace

PoseOptimizationResult OptimizePose(const RigidTransformation& initial_T_cw,
                                    const Camera& left,
                                    const std::optional<StereoCameras>& stereo,
                                    const std::vector<PoseObservation>& observations,
                                    const PoseOptimizationOptions& options) {
    PoseOptimizationResult result;
    result.T_cw = initial_T_cw;
    result.inlier.assign(observations.size(), false);
    if (!IsFinite(initial_T_cw) || !IsValid(left, stereo, observations, options)) {
        Log()->debug("Pose optimization skipped: invalid input or options ({} observations)",
                     observations.size());
        return result;
    }
    if (observations.size() < 3) {
        Log()->debug("Pose optimization skipped: {} observations, at least 3 needed",
                     observations.size());
        return result;
    }

    using BlockSolver = g2o::BlockSolver_6_3;
    auto linear_solver = std::make_unique<g2o::LinearSolverEigen<BlockSolver::PoseMatrixType>>();
    g2o::SparseOptimizer optimizer;
    // The optimizer owns the algorithm, the vertex and the edges (and the edges their kernels).
    optimizer.setAlgorithm(new g2o::OptimizationAlgorithmLevenberg(
        std::make_unique<BlockSolver>(std::move(linear_solver))));
    optimizer.setVerbose(false);

    auto* pose_vertex = new g2o::VertexSE3Expmap();
    pose_vertex->setId(0);
    pose_vertex->setEstimate(ToSE3Quat(initial_T_cw));
    optimizer.addVertex(pose_vertex);

    // Exactly one of mono_edges[i] / stereo_edges[i] is set per observation.
    std::vector<EdgeMonoPose*> mono_edges(observations.size(), nullptr);
    std::vector<EdgeStereoPose*> stereo_edges(observations.size(), nullptr);
    size_t num_stereo = 0;
    for (size_t i = 0; i < observations.size(); ++i) {
        const PoseObservation& observation = observations[i];
        const double information = 1.0 / (observation.sigma * observation.sigma);
        g2o::OptimizableGraph::Edge* edge = nullptr;
        double chi2 = options.mono_chi2;
        if (observation.right) {
            auto* stereo_edge = new EdgeStereoPose(observation.point, left, *stereo);
            stereo_edge->setMeasurement(
                (Eigen::Vector4d() << observation.left, *observation.right).finished());
            stereo_edge->setInformation(Eigen::Matrix4d::Identity() * information);
            stereo_edges[i] = stereo_edge;
            edge = stereo_edge;
            chi2 = options.stereo_chi2;
            ++num_stereo;
        } else {
            auto* mono = new EdgeMonoPose(observation.point, left);
            mono->setMeasurement(observation.left);
            mono->setInformation(Eigen::Matrix2d::Identity() * information);
            mono_edges[i] = mono;
            edge = mono;
        }
        edge->setVertex(0, pose_vertex);
        auto* kernel = new g2o::RobustKernelHuber();
        kernel->setDelta(std::sqrt(chi2));
        edge->setRobustKernel(kernel);
        optimizer.addEdge(edge);
    }
    Log()->debug("Pose optimization: {} observations ({} mono, {} stereo), {} rounds x {} iterations",
                 observations.size(), observations.size() - num_stereo, num_stereo, options.rounds,
                 options.iterations_per_round);

    // Every round resets the vertex to the input pose, so a failed round would
    // leave the unoptimized input in it: keep the last successfully optimized
    // estimate and report that one (none after a round-0 failure: no success).
    g2o::SE3Quat optimized = pose_vertex->estimate();
    bool optimized_once = false;
    size_t num_active = observations.size();
    for (int round = 0; round < options.rounds; ++round) {
        if (num_active == 0) {
            Log()->debug("Pose optimization round {}: no inlier left to optimize", round);
            break;
        }
        // Every round restarts from the input pose (ORB-SLAM3); only the edge set changes.
        pose_vertex->setEstimate(ToSE3Quat(initial_T_cw));
        if (round == 2) {
            for (g2o::HyperGraph::Edge* edge : optimizer.edges()) {
                static_cast<g2o::OptimizableGraph::Edge*>(edge)->setRobustKernel(nullptr);
            }
        }
        if (!optimizer.initializeOptimization(0)) {
            Log()->debug("Pose optimization round {} failed: optimizer could not initialize", round);
            break;
        }
        const int performed = optimizer.optimize(options.iterations_per_round);
        if (performed <= 0) {
            Log()->debug("Pose optimization round {} failed: optimizer returned {}", round,
                         performed);
            break;
        }

        optimized = pose_vertex->estimate();
        optimized_once = true;

        // Re-classify every observation, the excluded ones included.
        num_active = 0;
        for (size_t i = 0; i < observations.size(); ++i) {
            bool inlier = false;
            g2o::OptimizableGraph::Edge* edge = nullptr;
            if (stereo_edges[i] != nullptr) {
                stereo_edges[i]->computeError();
                inlier = stereo_edges[i]->chi2() <= options.stereo_chi2 &&
                         stereo_edges[i]->InFront();
                edge = stereo_edges[i];
            } else {
                mono_edges[i]->computeError();
                inlier = mono_edges[i]->chi2() <= options.mono_chi2 && mono_edges[i]->InFront();
                edge = mono_edges[i];
            }
            edge->setLevel(inlier ? 0 : 1);
            result.inlier[i] = inlier;
            num_active += inlier ? 1 : 0;
        }
        result.num_inliers = num_active;
        Log()->debug("Pose optimization round {} ({}): {} of {} inliers after {} iterations",
                     round, round < 2 ? "huber" : "plain", num_active, observations.size(),
                     performed);
    }

    result.T_cw = FromSE3Quat(optimized);
    result.success = optimized_once && result.num_inliers >= 3 && IsFinite(result.T_cw);
    if (!result.success) {
        Log()->debug("Pose optimization failed: {} inliers{}", result.num_inliers,
                     IsFinite(result.T_cw) ? "" : ", non-finite pose");
    }
    return result;
}

}  // namespace voyahslam::optimization
