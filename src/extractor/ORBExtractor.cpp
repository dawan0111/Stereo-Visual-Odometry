#include "stereo_visual_odometry/extractor/ORBExtractor.hpp"

namespace SVO {
ORBExtractor::ORBExtractor() : Extractor() {
  std::cout << "===== ORB Extractor =====" << std::endl;
  detector_ = cv::ORB::create(2000);
  matcher_ = cv::BFMatcher::create(cv::NORM_HAMMING);

  result_.leftKeyPoint.reserve(2000);
  result_.rightKeyPoint.reserve(2000);
  result_.matches.reserve(2000);
}
void ORBExtractor::compute(const cv::Mat &leftImage, const cv::Mat &rightImage) {
  clear();
  const auto &leftCamK = config_->getLeftCameraK();
  const auto &baseline = config_->getBaseline();

  std::vector<cv::KeyPoint> leftKeyPoints;
  std::vector<cv::KeyPoint> rightKeyPoints;
  cv::Mat leftDesc;
  cv::Mat rightDesc;

  leftKeyPoints.reserve(2000);
  rightKeyPoints.reserve(2000);

  detector_->detectAndCompute(leftImage, cv::noArray(), leftKeyPoints, leftDesc);
  detector_->detectAndCompute(rightImage, cv::noArray(), rightKeyPoints, rightDesc);
  matcher_->match(leftDesc, rightDesc, matches_);

  int16_t i = 0;

  std::cout << (baseline * leftCamK(0, 0)) << std::endl;

  for (auto &match : matches_) {
    if (match.distance < 30) {
      auto leftPoint = leftKeyPoints[match.queryIdx];
      auto rightPoint = rightKeyPoints[match.trainIdx];
      Eigen::Vector3d worldPoint = Eigen::Vector3d::Zero();

      double z = (baseline * leftCamK(0, 0)) / (leftPoint.pt.x - rightPoint.pt.x);
      double x = ((leftPoint.pt.x - leftCamK(0, 2)) * z) / leftCamK(0, 0);
      double y = ((leftPoint.pt.y - leftCamK(1, 2)) * z) / leftCamK(1, 1);

      if (z >= 35 || z <= 0 || leftPoint.pt.y != rightPoint.pt.y || leftPoint.pt.x == rightPoint.pt.x) {
        continue;
      }

      worldPoint(0) = x;
      worldPoint(1) = y;
      worldPoint(2) = z;

      result_.leftKeyPoint.push_back(leftPoint);
      result_.rightKeyPoint.push_back(rightPoint);

      auto leftDescriptor = leftDesc.row(match.queryIdx).clone();
      auto rightDescriptor = rightDesc.row(match.trainIdx).clone();

      if (i < 1) {
        result_.leftDesc = leftDescriptor;
        result_.rightDesc = rightDescriptor;
      } else {
        cv::vconcat(result_.leftDesc, leftDescriptor, result_.leftDesc);
        cv::vconcat(result_.rightDesc, rightDescriptor, result_.rightDesc);
      }
      result_.matches.emplace_back(i, i, std::move(worldPoint), static_cast<double>(match.distance));
      ++i;
    }
  }

  result_.leftImage = leftImage.clone();
  result_.rightImage = rightImage.clone();
}
void ORBExtractor::clear() {
  result_.leftKeyPoint.clear();
  result_.rightKeyPoint.clear();
  result_.leftDesc = cv::Mat::zeros(1, 1, CV_8SC1);
  result_.rightDesc = cv::Mat::zeros(1, 1, CV_8SC1);
  result_.leftImage = cv::Mat::zeros(1, 1, CV_8SC1);
  result_.rightImage = cv::Mat::zeros(1, 1, CV_8SC1);
  result_.matches.clear();
}
cv::Mat ORBExtractor::getDebugFrame(const cv::Mat &leftImage, const cv::Mat &rightImage) {
  cv::Mat resultImage;
  cv::hconcat(leftImage, rightImage, resultImage);

  auto leftImageCol = leftImage.cols;
  for (auto &match : result_.matches) {
    auto leftPoint = result_.leftKeyPoint[match.left_i];
    auto rightPoint = result_.rightKeyPoint[match.right_i];
    auto rightX = rightPoint.pt.x + leftImageCol;
    auto rightY = rightPoint.pt.y;
    cv::circle(resultImage, leftPoint.pt, 2, cv::Scalar(0, 255, 0), 1, cv::LINE_4, 0);
    cv::circle(resultImage, cv::Point2f(rightX, rightY), 4, cv::Scalar(0, 255, 0), 1, cv::LINE_4, 0);
    cv::line(resultImage, leftPoint.pt, cv::Point2f(rightX, rightY), cv::Scalar(0, 255, 0), 1);
  }
  return resultImage;
}
} // namespace SVO