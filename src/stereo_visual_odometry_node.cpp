#include "rclcpp/rclcpp.hpp"
#include "stereo_visual_odometry/extractor/ORBExtractor.hpp"
#include "stereo_visual_odometry/stereo_visual_odometry.hpp"
#include "stereo_visual_odometry/tracker/ORBTracker.hpp"
#include "stereo_visual_odometry/tracker/opticalTracker.hpp"
#include <iostream>

int32_t main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  const rclcpp::NodeOptions options;
  auto extractor = std::make_unique<SVO::ORBExtractor>();
  auto tracker = std::make_unique<SVO::OpticalTracker>();

  using SVONode = SVO::StereoVisualOdometry<SVO::ORBExtractor, SVO::OpticalTracker>;

  auto stereo_visual_odometry_node = std::make_shared<SVONode>(options, std::move(extractor), std::move(tracker));

  rclcpp::spin(stereo_visual_odometry_node->get_node_base_interface());
  rclcpp::shutdown();

  return 0;
}