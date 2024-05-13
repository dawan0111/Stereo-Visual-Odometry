#include "stereo_visual_odometry/stereo_visual_odometry.hpp"
namespace SVO {
template <typename T>
StereoVisualOdometry<T>::StereoVisualOdometry(
    const rclcpp::NodeOptions &options, std::unique_ptr<T> extractor)
    : Node("stereo_visual_odometry_node", options),
      extractor_(std::move(extractor)) {
  RCLCPP_INFO(this->get_logger(), "==== stereo_visual_odometry_node =====");

  stereoImagePublisher_ =
      this->create_publisher<sensor_msgs::msg::Image>("/stereo/image", 10);
  std::string datasetPath =
      "/home/kdw/dataset/data_odometry_gray/dataset/sequences/00";
  cv::Mat leftImage = cv::imread(datasetPath + "/image_0/000000.png");
  cv::Mat rightImage = cv::imread(datasetPath + "/image_1/000000.png");

  extractor_->registerFairImage(std::move(leftImage), std::move(rightImage));
  extractor_->compute();

  double frequency = 30.0;
  auto interval = std::chrono::duration<double>(1.0 / frequency);
  timer_ = this->create_wall_timer(interval, [this]() -> void {
    auto debugFrame = extractor_->getDebugFrame();
    auto message =
        cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", debugFrame)
            .toImageMsg();
    stereoImagePublisher_->publish(*message);
    // RCLCPP_INFO(this->get_logger(), "==== Timer Callback ====");
    // this->stereoImagePublisher_->publish();
  });
}
template <typename T> void StereoVisualOdometry<T>::registerExtractor() {
  RCLCPP_INFO(this->get_logger(), "==== registerExtractor =====");
}
template <typename T> void StereoVisualOdometry<T>::registerTracker() {
  RCLCPP_INFO(this->get_logger(), "==== registerTracker =====");
}

template class StereoVisualOdometry<SVO::ORBExtractor>;
} // namespace SVO