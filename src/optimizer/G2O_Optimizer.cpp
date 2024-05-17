#include "stereo_visual_odometry/optimizer/G2O_Optimizer.hpp"

namespace SVO {
G2O_Optimizer::G2O_Optimizer() {
  auto solver = new g2o::OptimizationAlgorithmGaussNewton(
      g2o::make_unique<BlockSolverType>(g2o::make_unique<LinearSolverType>()));
  optimizer_.setAlgorithm(solver);
  optimizer_.setVerbose(true);
}

void G2O_Optimizer::BundleAdjustment(const WorldPoints &points3D, const CameraPoints &points2D,
                                     const Eigen::Matrix3d &K, Sophus::SE3d &pose) {
  VertexPose *vertexPose = new VertexPose(); // camera vertexPose
  vertexPose->setId(0);
  vertexPose->setEstimate(Sophus::SE3d());
  optimizer_.addVertex(vertexPose);

  int index = 1;
  for (size_t i = 0; i < points2D.size(); ++i) {
    auto p2d = points2D[i];
    auto p3d = points3D[i];
    EdgeProjection *edge = new EdgeProjection(p3d, K);
    edge->setId(index);
    edge->setVertex(0, vertexPose);
    edge->setMeasurement(p2d);
    edge->setInformation(Eigen::Matrix2d::Identity());
    optimizer_.addEdge(edge);
    index++;
  }

  optimizer_.setVerbose(true);
  optimizer_.initializeOptimization();
  optimizer_.optimize(10);

  pose = vertexPose->estimate();

  optimizer_.clear();
}
} // namespace SVO