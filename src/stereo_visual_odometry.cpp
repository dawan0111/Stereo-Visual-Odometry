#include "stereo_visual_odometry/stereo_visual_odometry.hpp"

StereoVisualOdometry::StereoVisualOdometry(const rclcpp::NodeOptions &options)
    : Node("stereo_visual_odometry_node", options) {
  RCLCPP_INFO(this->get_logger(), "==== stereo_visual_odometry_node =====");

  double frequency = 30.0;
  auto interval = std::chrono::duration<double>(1.0 / frequency);
  auto callback = [this]() -> void {
    RCLCPP_INFO(this->get_logger(), "==== Timer Callback ====");
  };
  timer_ = this->create_wall_timer(interval, callback);
}

void StereoVisualOdometry::registerExtractor() {
  RCLCPP_INFO(this->get_logger(), "==== registerExtractor =====");
}
void StereoVisualOdometry::registerTracker() {
  RCLCPP_INFO(this->get_logger(), "==== registerTracker =====");
}