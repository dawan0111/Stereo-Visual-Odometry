#include "stereo_visual_odometry/tracker/ORBTracker.hpp"

namespace SVO {
ORBTracker::ORBTracker() : Tracker() {
  std::cout << "ORB Tracker" << std::endl;
  optimizer_ = std::make_unique<G2O_Optimizer>();
  CV_Optimizer_ = std::make_unique<CV_Optimizer>();
  matcher_ = cv::BFMatcher::create(cv::NORM_HAMMING);
  matches_.reserve(2000);
}
void ORBTracker::compute(const FrameDataT &prevFrameData, const FrameDataT &frameData) {
  std::vector<cv::DMatch> matches;
  matcher_->match(prevFrameData.leftDesc, frameData.leftDesc, matches);
  const auto &leftCamK = config_->getLeftCameraK();
  // std::vector<Eigen::Vector3d> worldPoints;
  // std::vector<Eigen::Vector2d> cameraPoints;

  std::vector<cv::Point3f> worldPoints;
  std::vector<cv::Point2f> cameraPoints;

  matches_.clear();
  worldPoints.reserve(4000);
  cameraPoints.reserve(4000);

  for (auto &match : matches) {
    if (match.distance < 30) {
      auto worldPoint = frameData.matches[match.trainIdx].worldPoint;
      auto prevCameraPoint = prevFrameData.leftKeyPoint[match.queryIdx];

      worldPoints.emplace_back(worldPoint(0), worldPoint(1), worldPoint(2));
      cameraPoints.emplace_back(prevCameraPoint.pt.x, prevCameraPoint.pt.y);
      matches_.push_back(match);
    }
  }

  cv::Mat cvLeftCamK = cv::Mat::eye(3, 3, CV_64F);
  cv::Mat distCoeffs = cv::Mat::zeros(4, 1, CV_64F);

  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      cvLeftCamK.at<double>(i, j) = leftCamK(i, j);
    }
  }

  CV_Optimizer_->BundleAdjustment(worldPoints, cameraPoints, cvLeftCamK, distCoeffs, pose_);
}

cv::Mat ORBTracker::getDebugFrame(const FrameDataT &prevFrameData, const FrameDataT &frameData) {
  cv::Mat resultImage;
  cv::vconcat(prevFrameData.leftImage, frameData.rightImage, resultImage);

  auto leftImageRow = prevFrameData.leftImage.rows;
  for (auto &match : matches_) {
    auto leftPoint = prevFrameData.leftKeyPoint[match.queryIdx];
    auto rightPoint = frameData.leftKeyPoint[match.trainIdx];
    auto rightX = rightPoint.pt.x;
    auto rightY = rightPoint.pt.y + leftImageRow;
    cv::circle(resultImage, leftPoint.pt, 4, cv::Scalar(0, 255, 0), 1, cv::LINE_4, 0);
    cv::circle(resultImage, cv::Point2f(rightX, rightY), 4, cv::Scalar(0, 255, 0), 1, cv::LINE_4, 0);
    cv::line(resultImage, leftPoint.pt, cv::Point2f(rightX, rightY), cv::Scalar(0, 255, 0), 1);
  }

  return resultImage;
}
} // namespace SVO