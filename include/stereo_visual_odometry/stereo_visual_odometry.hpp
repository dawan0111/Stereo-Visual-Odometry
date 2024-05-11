#ifndef STEREO_VISUAL_ODOMETRY_H
#define STEREO_VISUAL_ODOMETRY_H

#include "rclcpp/rclcpp.hpp"

class StereoVisualOdometry : public rclcpp::Node {
public:
  explicit StereoVisualOdometry(const rclcpp::NodeOptions &);

private:
  void registerExtractor();
  void registerTracker();

private:
  rclcpp::TimerBase::SharedPtr timer_;
};

#endif