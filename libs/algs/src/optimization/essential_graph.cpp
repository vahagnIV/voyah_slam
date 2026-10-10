#include "voyahslam/optimization/essential_graph.h"

#include <algorithm>
#include <cmath>
#include <memory>
#include <set>
#include <utility>
#include <vector>

#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/solvers/eigen/linear_solver_eigen.h>
#include <g2o/types/sim3/types_seven_dof_expmap.h>

#include "voyahslam/logging/logger.h"

namespace voyahslam::optimization {
namespace {

using geometry::RigidTransformation;
using geometry::Sim3;
using IndexPair = std::pair<size_t, size_t>;

/// ORB-SLAM3 OptimizeEssentialGraph: the Levenberg damping starts (almost) at Gauss-Newton.
constexpr double kUserLambdaInit = 1e-16;

g2o::Sim3 ToG2o(const Sim3& S) { return g2o::Sim3(S.R, S.t, S.scale); }

Sim3 FromG2o(const g2o::Sim3& S) {
    return Sim3{S.scale(), S.rotation().toRotationMatrix(), S.translation()};
}

bool IsFinite(const Sim3& S) { return std::isfinite(S.scale) && S.R.allFinite() && S.t.allFinite(); }

bool IsValid(const EssentialGraphProblem& problem) {
    const size_t n = problem.T_cw.size();
    if (n == 0 || problem.frames.size() != n || problem.corrected_S_cw.size() != n || problem.fixed >= n ||
        problem.iterations <= 0 ||
        std::any_of(problem.also_fixed.begin(), problem.also_fixed.end(), [n](size_t i) { return i >= n; })) {
        return false;
    }
    for (size_t i = 0; i < n; ++i) {
        const RigidTransformation& T = problem.T_cw[i];
        if (!T.rotation().allFinite() || !T.translation().allFinite()) {
            return false;
        }
        const std::optional<Sim3>& corrected = problem.corrected_S_cw[i];
        if (corrected.has_value() && (!IsFinite(*corrected) || !(corrected->scale > 0.0))) {
            return false;
        }
    }
    const auto valid_pair = [n](const IndexPair& pair) {
        return pair.first < n && pair.second < n && pair.first != pair.second;
    };
    return std::all_of(problem.edges.begin(), problem.edges.end(), valid_pair) &&
           std::all_of(problem.loop_connections.begin(), problem.loop_connections.end(), valid_pair);
}

// One logger for all calls; created on first use (after the app set the level).
const std::shared_ptr<spdlog::logger>& Log() {
    static const std::shared_ptr<spdlog::logger> log = Logger::CreateLogger("OptimizeEssentialGraph");
    return log;
}

}  // namespace

EssentialGraphResult OptimizeEssentialGraph(const EssentialGraphProblem& problem) {
    EssentialGraphResult result;
    if (!IsValid(problem)) {
        Log()->debug("Essential graph skipped: invalid input ({} frames)", problem.T_cw.size());
        return result;
    }
    const size_t n = problem.T_cw.size();

    // vScw (initial values) and the non-corrected poses, as in ORB-SLAM3.
    std::vector<g2o::Sim3> initial;
    std::vector<g2o::Sim3> non_corrected;
    initial.reserve(n);
    non_corrected.reserve(n);
    for (size_t i = 0; i < n; ++i) {
        non_corrected.push_back(ToG2o(Sim3{1.0, problem.T_cw[i].rotation(), problem.T_cw[i].translation()}));
        initial.push_back(problem.corrected_S_cw[i].has_value() ? ToG2o(*problem.corrected_S_cw[i])
                                                                : non_corrected.back());
    }
    if (problem.edges.empty() && problem.loop_connections.empty()) {
        for (const g2o::Sim3& S : initial) {
            result.S_cw.push_back(FromG2o(S));
        }
        result.success = true;
        return result;
    }

    using BlockSolver = g2o::BlockSolver_7_3;
    auto linear_solver = std::make_unique<g2o::LinearSolverEigen<BlockSolver::PoseMatrixType>>();
    auto* algorithm =
        new g2o::OptimizationAlgorithmLevenberg(std::make_unique<BlockSolver>(std::move(linear_solver)));
    algorithm->setUserLambdaInit(kUserLambdaInit);
    g2o::SparseOptimizer optimizer;
    // The optimizer owns the algorithm, the vertices and the edges.
    optimizer.setAlgorithm(algorithm);
    optimizer.setVerbose(false);

    std::vector<g2o::VertexSim3Expmap*> vertices;
    vertices.reserve(n);
    for (size_t i = 0; i < n; ++i) {
        auto* vertex = new g2o::VertexSim3Expmap();
        vertex->setId(static_cast<int>(i));
        vertex->setEstimate(initial[i]);
        vertex->setFixed(i == problem.fixed || std::find(problem.also_fixed.begin(), problem.also_fixed.end(), i) !=
                                                    problem.also_fixed.end());
        vertex->_fix_scale = problem.fix_scale;
        optimizer.addVertex(vertex);
        vertices.push_back(vertex);
    }

    // Edge (i, j): vertex 0 is S_iw, vertex 1 is S_jw, the measurement S_ji = S_jw · S_iw⁻¹ (g2o
    // EdgeSim3 error log(S_ji · S_iw · S_jw⁻¹)), identity information.
    std::set<IndexPair> inserted;
    const auto add_edge = [&](const IndexPair& pair, const std::vector<g2o::Sim3>& poses) {
        const auto [i, j] = pair;
        if (!inserted.insert(std::minmax(i, j)).second) {
            return;
        }
        auto* edge = new g2o::EdgeSim3();
        edge->setVertex(0, vertices[i]);
        edge->setVertex(1, vertices[j]);
        edge->setMeasurement(poses[j] * poses[i].inverse());
        edge->setInformation(Eigen::Matrix<double, 7, 7>::Identity());
        optimizer.addEdge(edge);
    };
    for (const IndexPair& pair : problem.loop_connections) {
        add_edge(pair, initial);
    }
    for (const IndexPair& pair : problem.edges) {
        add_edge(pair, non_corrected);
    }

    if (!optimizer.initializeOptimization()) {
        Log()->debug("Essential graph failed: the optimizer did not initialize");
        return result;
    }
    optimizer.computeActiveErrors();
    const double initial_chi2 = optimizer.activeChi2();
    optimizer.optimize(problem.iterations);
    optimizer.computeActiveErrors();

    result.S_cw.reserve(n);
    for (const g2o::VertexSim3Expmap* vertex : vertices) {
        const Sim3 optimized = FromG2o(vertex->estimate());
        if (!IsFinite(optimized) || !(optimized.scale > 0.0)) {
            Log()->debug("Essential graph failed: non-finite estimate");
            result.S_cw.clear();
            return result;
        }
        result.S_cw.push_back(optimized);
    }
    result.success = true;
    Log()->debug("Essential graph: {} frames, {} edges, chi2 {} -> {}", n, inserted.size(), initial_chi2,
                 optimizer.activeChi2());
    return result;
}

}  // namespace voyahslam::optimization
