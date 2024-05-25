#include "stereo_visual_odometry/tracker/opticalTracker.hpp"

namespace SVO {
OpticalTracker::OpticalTracker() : Tracker() {
  std::cout << "OpticalTracker" << std::endl;
  CV_Optimizer_ = std::make_unique<CV_Optimizer>();
}
void OpticalTracker::compute(const FrameDataT &prevFrameData, FrameDataT &frameData) {
  std::vector<cv::Point2f> prevPts;
  std::vector<cv::Point2f> nextPts;

  std::vector<uchar> status;
  std::vector<float> err;
  std::vector<cv::Point3f> worldPoints;

  cameraPts_.clear();

  const auto &leftCamK = config_->getLeftCameraK();
  auto trackingKeyPoint = frameData.leftKeyPoint;

  // if (trackingKeyPoint.size() <= 250) {
  //   trackingKeyPoint = prevFrameData.leftKeyPoint;
  // }

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

  // for (size_t i = 0; i < nextPts.size(); ++i) {
  //   frameData.trackingKeyPoint.push_back(cv::KeyPoint(nextPts[i], 1.f));
  // }

  cv::Mat cvLeftCamK = cv::Mat::eye(3, 3, CV_64F);
  cv::Mat distCoeffs = cv::Mat::zeros(4, 1, CV_64F);

  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      cvLeftCamK.at<double>(i, j) = leftCamK(i, j);
    }
  }

  CV_Optimizer_->BundleAdjustment(worldPoints, nextPts, cvLeftCamK, distCoeffs, pose_);

  // double focal = 718.8560;
  // cv::Point2d pp(607.1928, 185.2157);
  // cv::Mat E, R, t, mask;
  // E = cv::findEssentialMat(nextPts, prevPts, focal, pp, cv::RANSAC, 0.999, 1.0, mask);
  // cv::recoverPose(E, nextPts, prevPts, R, t, focal, pp, mask);

  // Eigen::Matrix3d eigenR;
  // Eigen::Vector3d eigenT;

  // for (int i = 0; i < 3; ++i) {
  //   for (int j = 0; j < 3; ++j) {
  //     eigenR(i, j) = R.at<double>(i, j);
  //   }
  // }

  // eigenT(0) = t.at<double>(0);
  // eigenT(1) = t.at<double>(1);
  // eigenT(2) = t.at<double>(2);

  // pose_ = Sophus::SE3d(eigenR, eigenT);
}
cv::Mat OpticalTracker::getDebugFrame(const FrameDataT &prevFrameData, const FrameDataT &frameData) {
  cv::Mat resultImage;
  cv::vconcat(prevFrameData.leftImage, frameData.rightImage, resultImage);

  auto leftImageRow = prevFrameData.leftImage.rows;
  for (auto &match : matches_) {
    auto leftPoint = prevFrameData.leftKeyPoint[match.queryIdx];
    auto rightPoint = cameraPts_[match.trainIdx];
    auto rightX = rightPoint.x;
    auto rightY = rightPoint.y + leftImageRow;
    cv::circle(resultImage, leftPoint.pt, 4, cv::Scalar(0, 255, 0), 1, cv::LINE_4, 0);
    cv::circle(resultImage, cv::Point2f(rightX, rightY), 4, cv::Scalar(0, 255, 0), 1, cv::LINE_4, 0);
    cv::line(resultImage, leftPoint.pt, cv::Point2f(rightX, rightY), cv::Scalar(0, 255, 0), 1);
  }

  return resultImage;
}
} // namespace SVO