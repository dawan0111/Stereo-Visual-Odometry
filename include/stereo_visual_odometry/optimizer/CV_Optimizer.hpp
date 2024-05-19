#ifndef CV_OPTIMIZER_H
#define CV_OPTIMIZER_H
#include <opencv2/opencv.hpp>
#include <sophus/se3.hpp>

namespace SVO {
class CV_Optimizer {
  using WorldPoints = std::vector<cv::Point3f>;
  using CameraPoints = std::vector<cv::Point2f>;

public:
  CV_Optimizer();
  void BundleAdjustment(const WorldPoints &points3D, const CameraPoints &points2D, const cv::Mat &K,
                        const cv::Mat &distCoeffs, Sophus::SE3d &pose);
};
} // namespace SVO

#endif