#include "stereo_visual_odometry/optimizer/CV_Optimizer.hpp"

namespace SVO {
CV_Optimizer::CV_Optimizer() { std::cout << "===== CV_Optimizer (SolvePnP) =====" << std::endl; }
void CV_Optimizer::BundleAdjustment(const WorldPoints &points3D, const CameraPoints &points2D, const cv::Mat &K,
                                    const cv::Mat &distCoeffs, Sophus::SE3d &pose) {
  cv::Mat rVec, tVec;
  bool success = cv::solvePnPRansac(points3D, points2D, K, distCoeffs, rVec, tVec, false, 200, 2, 0.999);
  if (success) {
    cv::Mat R;
    Eigen::Matrix3d eigenR;
    Eigen::Vector3d eigenT;
    cv::Rodrigues(rVec, R);

    for (int i = 0; i < 3; ++i) {
      for (int j = 0; j < 3; ++j) {
        eigenR(i, j) = R.at<double>(i, j);
      }
    }

    eigenT(0) = tVec.at<double>(0);
    eigenT(1) = tVec.at<double>(1);
    eigenT(2) = tVec.at<double>(2);

    pose = Sophus::SE3d(eigenR, eigenT);
  }
}

} // namespace SVO