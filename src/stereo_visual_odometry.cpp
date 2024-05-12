#include "stereo_visual_odometry/stereo_visual_odometry.hpp"
namespace SVO {
template <typename T>
StereoVisualOdometry<T>::StereoVisualOdometry(
    const rclcpp::NodeOptions &options, std::unique_ptr<T> extractor)
    : Node("stereo_visual_odometry_node", options),
      extractor_(std::move(extractor)) {
  RCLCPP_INFO(this->get_logger(), "==== stereo_visual_odometry_node =====");

  double frequency = 30.0;
  auto interval = std::chrono::duration<double>(1.0 / frequency);
  auto callback = [this]() -> void {
    // RCLCPP_INFO(this->get_logger(), "==== Timer Callback ====");
  };
  timer_ = this->create_wall_timer(interval, callback);
  // extractor_ = std::move(extractor);
}
template <typename T> void StereoVisualOdometry<T>::registerExtractor() {
  RCLCPP_INFO(this->get_logger(), "==== registerExtractor =====");
}
template <typename T> void StereoVisualOdometry<T>::registerTracker() {
  RCLCPP_INFO(this->get_logger(), "==== registerTracker =====");
}

template class StereoVisualOdometry<SVO::ORBExtractor>;
} // namespace SVO