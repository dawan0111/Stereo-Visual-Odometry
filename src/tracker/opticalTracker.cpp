#include "stereo_visual_odometry/tracker/opticalTracker.hpp"

namespace SVO {
OpticalTracker::OpticalTracker() : Tracker() {
  std::cout << "===== Optical tracker =====" << std::endl;
  CV_Optimizer_ = std::make_unique<CV_Optimizer>();
}
void OpticalTracker::compute(const FrameDataT &prevFrameData, FrameDataT &frameData) {
  std::vector<cv::Point2f> prevPts;
  std::vector<cv::Point2f> nextPts;

  std::vector<uchar> status;
  std::vector<float> err;
  std::vector<cv::Point3f> worldPoints;

  const auto &leftCamK = config_->getLeftCameraK();
  auto trackingKeyPoint = frameData.leftKeyPoint;

  prevPoints_.clear();
  nextPoints_.clear();

  for (const auto &kp : trackingKeyPoint) {
    prevPts.push_back(kp.pt);
  }

  if (!prevFrameData.leftImage.empty() && !frameData.leftImage.empty()) {
    cv::calcOpticalFlowPyrLK(frameData.leftImage, prevFrameData.leftImage, prevPts, nextPts, status, err);
  }

  int indexCorrection = 0;
  for (int i = 0; i < status.size(); i++) {
    cv::Point2f pt = nextPts.at(i - indexCorrection);
    if ((status.at(i) == 0) || (pt.x < 0) || (pt.y < 0)) {
      if ((pt.x < 0) || (pt.y < 0)) {
        status.at(i) = 0;
      }
      prevPts.erase(prevPts.begin() + (i - indexCorrection));
      nextPts.erase(nextPts.begin() + (i - indexCorrection));
      indexCorrection++;
    } else {
      auto worldPoint = frameData.matches[i].worldPoint;
      worldPoints.emplace_back(worldPoint(0), worldPoint(1), worldPoint(2));
    }
  }

  cv::Mat cvLeftCamK = cv::Mat::eye(3, 3, CV_64F);
  cv::Mat distCoeffs = cv::Mat::zeros(4, 1, CV_64F);

  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      cvLeftCamK.at<double>(i, j) = leftCamK(i, j);
    }
  }

  CV_Optimizer_->BundleAdjustment(worldPoints, nextPts, cvLeftCamK, distCoeffs, pose_);

  prevPoints_ = prevPts;
  nextPoints_ = nextPts;
}
cv::Mat OpticalTracker::getDebugFrame(const FrameDataT &prevFrameData, const FrameDataT &frameData) {
  cv::Mat resultImage = prevFrameData.leftImage.clone();

  auto leftImageRow = prevFrameData.leftImage.rows;
  for (int i = 0; i < prevPoints_.size(); ++i) {
    auto leftPoint = prevPoints_[i];
    auto rightPoint = nextPoints_[i];
    auto rightX = rightPoint.x;
    auto rightY = rightPoint.y;
    cv::circle(resultImage, leftPoint, 4, cv::Scalar(0, 255, 255), 1, cv::LINE_4, 0);
    cv::circle(resultImage, cv::Point2f(rightX, rightY), 4, cv::Scalar(0, 255, 0), 1, cv::LINE_4, 0);
    cv::arrowedLine(resultImage, leftPoint, cv::Point2f(rightX, rightY), cv::Scalar(0, 255, 0), 1);
  }

  return resultImage;
}
} // namespace SVO