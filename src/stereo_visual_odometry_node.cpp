#include "rclcpp/rclcpp.hpp"
#include "stereo_visual_odometry/stereo_visual_odometry.hpp"
#include <iostream>

int32_t main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  const rclcpp::NodeOptions options;
  auto stereo_visual_odometry_node =
      std::make_shared<StereoVisualOdometry>(options);

  rclcpp::spin(stereo_visual_odometry_node->get_node_base_interface());
  rclcpp::shutdown();

  return 0;
}