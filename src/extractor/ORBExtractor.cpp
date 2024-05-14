#include "stereo_visual_odometry/extractor/ORBExtractor.hpp"

namespace SVO {
ORBExtractor::ORBExtractor() : Extractor() {
  std::cout << "===== ORB Extractor =====" << std::endl;
  detector_ = cv::ORB::create();
  matcher_ = cv::BFMatcher::create(cv::NORM_HAMMING);
}
void ORBExtractor::compute() {
  std::cout << "[ORB_EXTRACTOR] compute!!" << std::endl;
  auto leftCamK = config_->getLeftCameraK();
  auto rightCamK = config_->getRightCameraK();
  auto baseline = config_->getBaseline();

  std::cout << "baseline: " << baseline << std::endl;

  detector_->detectAndCompute(leftImage_, cv::noArray(), result_.leftKeyPoint, result_.leftDesc);
  detector_->detectAndCompute(rightImage_, cv::noArray(), result_.rightKeyPoint, result_.rightDesc);
  matcher_->match(result_.leftDesc, result_.rightDesc, matches_);

  // std::cout << "baseline: " << baseline << std::endl;
  // std::cout << "leftCamK: " << leftCamK << std::endl;

  for (auto &match : matches_) {
    if (match.distance < 40) {
      auto leftPoint = result_.leftKeyPoint[match.queryIdx];
      auto rightPoint = result_.rightKeyPoint[match.trainIdx];
      Eigen::Vector3d worldPoint = Eigen::Vector3d::Zero();

      double z = (baseline * leftCamK(0, 0)) / (leftPoint.pt.x - rightPoint.pt.x);
      double x = ((leftPoint.pt.x - leftCamK(0, 2)) * z) / leftCamK(0, 0);
      double y = ((leftPoint.pt.y - leftCamK(1, 2)) * z) / leftCamK(1, 1);

      worldPoint(0) = x;
      worldPoint(1) = y;
      worldPoint(2) = z;

      std::cout << worldPoint(0) << std::endl;

      result_.matches.emplace_back(match.queryIdx, match.trainIdx, std::move(worldPoint),
                                   static_cast<double>(match.distance));
    }
  }
}
cv::Mat ORBExtractor::getDebugFrame() {
  cv::Mat resultImage;
  cv::hconcat(leftImage_, rightImage_, resultImage);
  int16_t size = std::max({result_.leftKeyPoint.size(), result_.leftKeyPoint.size()});

  auto leftImageCol = leftImage_.cols;
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