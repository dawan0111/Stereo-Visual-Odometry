#ifndef STEREO_VISUAL_ODOMETRY_H
#define STEREO_VISUAL_ODOMETRY_H

#include "rclcpp/rclcpp.hpp"
#include "stereo_visual_odometry/extractor/ORBExtractor.hpp"
namespace SVO {
template <typename T> class StereoVisualOdometry : public rclcpp::Node {
public:
  // explicit StereoVisualOdometry(const rclcpp::NodeOptions &);
  explicit StereoVisualOdometry(const rclcpp::NodeOptions &,
                                std::unique_ptr<T> extractor);

  // Deduction guide
  // template <typename T>
  // StereoVisualOdometry(const rclcpp::NodeOptions &options, T &&extractor)
  //     -> StereoVisualOdometry<T>;

private:
  void registerExtractor();
  void registerTracker();

private:
  rclcpp::TimerBase::SharedPtr timer_;
  std::unique_ptr<T> extractor_;
};
} // namespace SVO

#endif