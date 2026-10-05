#include <iostream>
#include <memory>

#include "g2o/core/block_solver.h"
#include "g2o/core/optimization_algorithm_levenberg.h"
#include "g2o/core/sparse_optimizer.h"
#include "g2o/solvers/eigen/linear_solver_eigen.h"
#include "g2o/types/sba/edge_project_xyz2uv.h"
#include "g2o/types/sba/parameter_cameraparameters.h"
#include "g2o/types/sba/vertex_se3_expmap.h"
#include "g2o/types/slam3d/vertex_pointxyz.h"

int main() {
  using BlockSolver = g2o::BlockSolver_6_3;
  using LinearSolver = g2o::LinearSolverEigen<BlockSolver::PoseMatrixType>;

  g2o::SparseOptimizer optimizer;
  optimizer.setAlgorithm(new g2o::OptimizationAlgorithmLevenberg(
      std::make_unique<BlockSolver>(std::make_unique<LinearSolver>())));

  auto* pose = new g2o::VertexSE3Expmap();
  pose->setId(0);
  pose->setFixed(true);
  optimizer.addVertex(pose);

  auto* point = new g2o::VertexPointXYZ();
  point->setId(1);
  point->setMarginalized(true);
  optimizer.addVertex(point);

  auto* camera = new g2o::CameraParameters(500.0, Eigen::Vector2d(320.0, 240.0), 0.0);
  camera->setId(0);
  optimizer.addParameter(camera);

  // An edge is required: initializeOptimization() fails on an edgeless graph.
  auto* edge = new g2o::EdgeProjectXYZ2UV();
  edge->setVertex(0, point);
  edge->setVertex(1, pose);
  edge->setMeasurement(Eigen::Vector2d(320.0, 240.0));
  edge->setInformation(Eigen::Matrix2d::Identity());
  edge->setParameterId(0, 0);
  optimizer.addEdge(edge);

  const bool ok = optimizer.initializeOptimization();
  std::cout << "g2o initializeOptimization: " << (ok ? "ok" : "failed") << "\n";
  return ok ? 0 : 1;
}
