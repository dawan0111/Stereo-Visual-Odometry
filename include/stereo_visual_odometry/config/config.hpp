#ifndef CONFIG_H
#define CONFIG_H
#include <Eigen/Core>
namespace SVO {
class Config {
public:
  Config(){};
  void setLeftCameraK(Eigen::Matrix3d &&leftCameraK) { leftCameraK_ = leftCameraK; };
  void setRightCameraK(Eigen::Matrix3d &&rightCameraK) { leftCameraK_ = rightCameraK; };
  void setBaseline(double baseline) { baseline_ = baseline; };
  const Eigen::Matrix3d &getLeftCameraK() { return leftCameraK_; };
  const Eigen::Matrix3d &getRightCameraK() { return rightCameraK_; };
  const double &getBaseline() { return baseline_; };

private:
  Eigen::Matrix3d leftCameraK_;
  Eigen::Matrix3d rightCameraK_;
  double baseline_;
};
} // namespace SVO

#endif