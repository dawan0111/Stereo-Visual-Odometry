#ifndef STEREO_VISUAL_ODOMETRY_H
#define STEREO_VISUAL_ODOMETRY_H

#include "rclcpp/rclcpp.hpp"
#include "stereo_visual_odometry/extractor/ORBExtractor.hpp"
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/msg/image.hpp>

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
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr stereoImagePublisher_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr leftImageSubscriber_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr
      rightImageSubscriber_;
  std::unique_ptr<T> extractor_;
};
} // namespace SVO

#endif