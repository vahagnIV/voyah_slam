#include "voyahslam/optimization/sim3_optimization.h"

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
#include <g2o/types/sim3/types_seven_dof_expmap.h>

#include "optimization/projection.h"
#include "voyahslam/logging/logger.h"

namespace voyahslam::optimization {
namespace {

using geometry::Camera;
using geometry::Sim3;
using geometry::TPoint3DF;

/// ORB-SLAM3 OptimizeSim3: fewer matches than this after the first stage is a failure.
constexpr size_t kMinMatches = 10;
constexpr int kFirstStageIterations = 5;
constexpr int kSecondStageIterations = 5;
constexpr int kSecondStageIterationsAfterRemoval = 10;
/// Residual (px, per coordinate) of a point behind the camera, which cannot be projected: a
/// constant with a zero Jacobian, far above any χ² threshold.
constexpr double kBehindCameraError = 1e4;

g2o::Sim3 ToG2o(const Sim3& S) { return g2o::Sim3(S.R, S.t, S.scale); }

Sim3 FromG2o(const g2o::Sim3& S) {
    return Sim3{S.scale(), S.rotation().toRotationMatrix(), S.translation()};
}

bool IsFinite(const Sim3& S) { return std::isfinite(S.scale) && S.R.allFinite() && S.t.allFinite(); }

/// Frame-1 raw-pixel error of the fixed camera-2 point X2 (ORB-SLAM3 EdgeSim3ProjectXYZ):
/// e = camera1.Project(S_12 X2) - pixel1.
class EdgeSim3Pixel : public g2o::BaseUnaryEdge<2, Eigen::Vector2d, g2o::VertexSim3Expmap> {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    EdgeSim3Pixel(const TPoint3DF& X2, const Camera& camera1) : X2_(X2), camera1_(camera1) {}

    bool read(std::istream& /*is*/) override { return false; }
    bool write(std::ostream& /*os*/) const override { return false; }

    void computeError() override {
        if (!InFront()) {
            _error.setConstant(kBehindCameraError);
            return;
        }
        _error = Sim3Pixel(camera1_, S12(), X2_) - _measurement;
    }

    void linearizeOplus() override {
        if (!InFront()) {
            _jacobianOplusXi.setZero();
            return;
        }
        _jacobianOplusXi = Sim3PixelJacobian(camera1_, S12(), X2_);
        if (vertexXn<0>()->_fix_scale) {
            _jacobianOplusXi.col(6).setZero();  // the vertex ignores the scale update
        }
    }

    /// Positive camera-1 depth of S_12 X2 at the current estimate.
    [[nodiscard]] bool InFront() const { return geometry::Apply(S12(), X2_).z() > 0.0; }

private:
    [[nodiscard]] Sim3 S12() const { return FromG2o(vertexXn<0>()->estimate()); }

    TPoint3DF X2_;
    Camera camera1_;
};

/// Frame-2 raw-pixel error of the fixed camera-1 point X1 (ORB-SLAM3
/// EdgeInverseSim3ProjectXYZ): e = camera2.Project(S_12⁻¹ X1) - pixel2.
class EdgeInverseSim3Pixel : public g2o::BaseUnaryEdge<2, Eigen::Vector2d, g2o::VertexSim3Expmap> {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    EdgeInverseSim3Pixel(const TPoint3DF& X1, const Camera& camera2) : X1_(X1), camera2_(camera2) {}

    bool read(std::istream& /*is*/) override { return false; }
    bool write(std::ostream& /*os*/) const override { return false; }

    void computeError() override {
        if (!InFront()) {
            _error.setConstant(kBehindCameraError);
            return;
        }
        _error = InverseSim3Pixel(camera2_, S12(), X1_) - _measurement;
    }

    void linearizeOplus() override {
        if (!InFront()) {
            _jacobianOplusXi.setZero();
            return;
        }
        _jacobianOplusXi = InverseSim3PixelJacobian(camera2_, S12(), X1_);
        if (vertexXn<0>()->_fix_scale) {
            _jacobianOplusXi.col(6).setZero();
        }
    }

    /// Positive camera-2 depth of S_12⁻¹ X1 at the current estimate.
    [[nodiscard]] bool InFront() const {
        return geometry::Apply(geometry::Inverse(S12()), X1_).z() > 0.0;
    }

private:
    [[nodiscard]] Sim3 S12() const { return FromG2o(vertexXn<0>()->estimate()); }

    TPoint3DF X1_;
    Camera camera2_;
};

bool IsValid(const Camera& camera1, const Camera& camera2, const std::vector<Sim3Match>& matches,
             const Sim3& initial_S_12, double huber_chi2) {
    if (!IsFinite(initial_S_12) || !(initial_S_12.scale > 0.0) || !(huber_chi2 > 0.0) ||
        !std::isfinite(huber_chi2) || !camera1.K.allFinite() || !camera2.K.allFinite()) {
        return false;
    }
    for (const Sim3Match& match : matches) {
        if (!match.X1.allFinite() || !match.pixel1.allFinite() || !match.X2.allFinite() ||
            !match.pixel2.allFinite() || !std::isfinite(match.sigma1) || !(match.sigma1 > 0.0) ||
            !std::isfinite(match.sigma2) || !(match.sigma2 > 0.0)) {
            return false;
        }
    }
    return true;
}

// One logger for all calls; created on first use (after the app set the level).
const std::shared_ptr<spdlog::logger>& Log() {
    static const std::shared_ptr<spdlog::logger> log = Logger::CreateLogger("OptimizeSim3");
    return log;
}

}  // namespace

Sim3OptimizationResult OptimizeSim3(const Camera& camera1, const Camera& camera2, const std::vector<Sim3Match>& matches,
                                    const Sim3& initial_S_12, bool fix_scale, double huber_chi2) {
    Sim3OptimizationResult result;
    result.S_12 = initial_S_12;
    result.inlier.assign(matches.size(), false);
    if (!IsValid(camera1, camera2, matches, initial_S_12, huber_chi2)) {
        Log()->debug("Sim3 optimization skipped: invalid input ({} matches)", matches.size());
        return result;
    }

    using BlockSolver = g2o::BlockSolverX;
    auto linear_solver = std::make_unique<g2o::LinearSolverEigen<BlockSolver::PoseMatrixType>>();
    g2o::SparseOptimizer optimizer;
    // The optimizer owns the algorithm, the vertex and the edges (and the edges their kernels).
    optimizer.setAlgorithm(
        new g2o::OptimizationAlgorithmLevenberg(std::make_unique<BlockSolver>(std::move(linear_solver))));
    optimizer.setVerbose(false);

    auto* vertex = new g2o::VertexSim3Expmap();
    vertex->_fix_scale = fix_scale;
    vertex->setId(0);
    vertex->setEstimate(ToG2o(initial_S_12));
    optimizer.addVertex(vertex);

    // Edge pair k is matches[match_of[k]]; matches with a point behind its own camera get none.
    std::vector<EdgeSim3Pixel*> edges12;
    std::vector<EdgeInverseSim3Pixel*> edges21;
    std::vector<size_t> match_of;
    const double delta = std::sqrt(huber_chi2);
    for (size_t i = 0; i < matches.size(); ++i) {
        const Sim3Match& match = matches[i];
        if (!(match.X1.z() > 0.0) || !(match.X2.z() > 0.0)) {
            continue;
        }
        auto* e12 = new EdgeSim3Pixel(match.X2, camera1);
        e12->setMeasurement(match.pixel1);
        e12->setInformation(Eigen::Matrix2d::Identity() / (match.sigma1 * match.sigma1));
        auto* e21 = new EdgeInverseSim3Pixel(match.X1, camera2);
        e21->setMeasurement(match.pixel2);
        e21->setInformation(Eigen::Matrix2d::Identity() / (match.sigma2 * match.sigma2));
        for (g2o::OptimizableGraph::Edge* edge : {static_cast<g2o::OptimizableGraph::Edge*>(e12),
                                                  static_cast<g2o::OptimizableGraph::Edge*>(e21)}) {
            edge->setVertex(0, vertex);
            auto* kernel = new g2o::RobustKernelHuber();
            kernel->setDelta(delta);
            edge->setRobustKernel(kernel);
            optimizer.addEdge(edge);
        }
        edges12.push_back(e12);
        edges21.push_back(e21);
        match_of.push_back(i);
    }
    const size_t num_correspondences = match_of.size();
    if (num_correspondences < kMinMatches) {
        Log()->debug("Sim3 optimization failed: {} usable matches of {}, at least {} needed", num_correspondences,
                     matches.size(), kMinMatches);
        return result;
    }

    // An edge pair is an inlier when both points are in front and both χ² pass.
    const auto is_inlier = [&](size_t k) {
        edges12[k]->computeError();
        edges21[k]->computeError();
        return edges12[k]->InFront() && edges21[k]->InFront() && edges12[k]->chi2() <= huber_chi2 &&
               edges21[k]->chi2() <= huber_chi2;
    };

    if (!optimizer.initializeOptimization(0) || optimizer.optimize(kFirstStageIterations) <= 0) {
        Log()->debug("Sim3 optimization failed: the first stage did not run");
        return result;
    }
    size_t num_removed = 0;
    for (size_t k = 0; k < num_correspondences; ++k) {
        if (is_inlier(k)) {
            edges12[k]->setRobustKernel(nullptr);
            edges21[k]->setRobustKernel(nullptr);
        } else {
            edges12[k]->setLevel(1);
            edges21[k]->setLevel(1);
            ++num_removed;
        }
    }
    if (num_correspondences - num_removed < kMinMatches) {
        Log()->debug("Sim3 optimization failed: {} of {} matches left after the first stage",
                     num_correspondences - num_removed, num_correspondences);
        return result;
    }

    const int iterations = num_removed > 0 ? kSecondStageIterationsAfterRemoval : kSecondStageIterations;
    if (!optimizer.initializeOptimization(0) || optimizer.optimize(iterations) <= 0) {
        Log()->debug("Sim3 optimization failed: the second stage did not run");
        return result;
    }
    const Sim3 optimized = FromG2o(vertex->estimate());
    if (!IsFinite(optimized)) {
        Log()->debug("Sim3 optimization failed: non-finite estimate");
        return result;
    }

    result.success = true;
    result.S_12 = optimized;
    for (size_t k = 0; k < num_correspondences; ++k) {
        if (edges12[k]->level() == 0 && is_inlier(k)) {
            result.inlier[match_of[k]] = true;
            ++result.num_inliers;
        }
    }
    Log()->debug("Sim3 optimization: {} of {} matches inliers ({} removed after the first stage), scale {}",
                 result.num_inliers, matches.size(), num_removed, result.S_12.scale);
    return result;
}

}  // namespace voyahslam::optimization
