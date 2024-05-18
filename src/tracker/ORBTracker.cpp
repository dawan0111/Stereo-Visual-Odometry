#include "stereo_visual_odometry/tracker/ORBTracker.hpp"

namespace SVO {
ORBTracker::ORBTracker() : Tracker() {
  std::cout << "ORB Tracker" << std::endl;
  optimizer_ = std::make_unique<G2O_Optimizer>();
  matcher_ = cv::BFMatcher::create(cv::NORM_HAMMING);
  matches_.reserve(1000);
}
void ORBTracker::compute(const FrameDataT &prevFrameData, const FrameDataT &frameData) {
  matcher_->match(prevFrameData.leftDesc, frameData.leftDesc, matches_);
  const auto &leftCamK = config_->getLeftCameraK();
  std::vector<Eigen::Vector3d> worldPoints;
  std::vector<Eigen::Vector2d> cameraPoints;

  worldPoints.reserve(1000);
  cameraPoints.reserve(1000);

  for (auto &match : matches_) {
    if (match.distance <= 40) {
      auto prevWorldPoint = prevFrameData.matches[match.queryIdx].worldPoint;
      auto cameraPoint = frameData.leftKeyPoint[match.trainIdx];

      worldPoints.push_back(prevWorldPoint);
      cameraPoints.emplace_back(cameraPoint.pt.x, cameraPoint.pt.y);
    }
  }

  optimizer_->BundleAdjustment(worldPoints, cameraPoints, leftCamK, pose_);

  std::cout << "Tracker: matching pair: " << worldPoints.size() << std::endl;
  std::cout << "Pose: " << pose_.matrix() << std::endl;
}
} // namespace SVO