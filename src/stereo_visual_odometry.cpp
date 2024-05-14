#include "stereo_visual_odometry/stereo_visual_odometry.hpp"
namespace SVO {
template <typename T>
StereoVisualOdometry<T>::StereoVisualOdometry(const rclcpp::NodeOptions &options, std::unique_ptr<T> extractor)
    : Node("stereo_visual_odometry_node", options), extractor_(std::move(extractor)) {
  RCLCPP_INFO(this->get_logger(), "==== stereo_visual_odometry_node =====");
  config_ = std::make_shared<Config>();
  initializeParameter();

  stereoImagePublisher_ = this->create_publisher<Image>("/stereo/image", 10);
  pointCloudPublisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/stereo/pointcloud", 10);

  leftImageSub_ = std::make_shared<message_filters::Subscriber<Image>>(this, "/stereo/image_left");
  rightImageSub_ = std::make_shared<message_filters::Subscriber<Image>>(this, "/stereo/image_right");
  sync_ = std::make_shared<message_filters::Synchronizer<ImageSyncPolicy>>(ImageSyncPolicy(10), *leftImageSub_,
                                                                           *rightImageSub_);

  // sync_->
  sync_->registerCallback(
      std::bind(&StereoVisualOdometry::ImageCallback, this, std::placeholders::_1, std::placeholders::_2));
  // std::string datasetPath = "/home/kdw/dataset/data_odometry_gray/dataset/sequences/00";
  // cv::Mat leftImage = cv::imread(datasetPath + "/image_0/000000.png");
  // cv::Mat rightImage = cv::imread(datasetPath + "/image_1/000000.png");

  // double frequency = 30.0;
  // auto interval = std::chrono::duration<double>(1.0 / frequency);
  // timer_ = this->create_wall_timer(interval, [this]() -> void {
  //   auto result = extractor_->getResult();
  //   auto debugFrame = extractor_->getDebugFrame();
  //   std::vector<Eigen::Vector3d> worldPoints;

  //   for (auto &match : result.matches) {
  //     worldPoints.push_back(match.worldPoint);
  //   }

  //   auto message = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", debugFrame).toImageMsg();
  //   message->header.stamp = this->get_clock()->now();
  //   stereoImagePublisher_->publish(*message);

  //   auto pointCloud = Utils::vector3dToPointCloud(worldPoints);
  //   pointCloud.header.frame_id = "camera_optical_link";
  //   pointCloud.header.stamp = this->get_clock()->now();
  //   pointCloudPublisher_->publish(pointCloud);
  // });
}
template <typename T> void StereoVisualOdometry<T>::initializeParameter() {
  RCLCPP_INFO(this->get_logger(), "initializeParameter");

  /* clang-format off */
  std::vector<double> LCamK = {
    718.856, 0.0, 607.1928,
    0.0, 718.856, 185.2157,
    0.0, 0.0,   1.0
  };
  std::vector<double> RCamK = {
    718.856, 0.0, 607.1928,
    0.0, 718.856, 185.2157,
    0.0, 0.0,   1.0
  };
  double baseline = 0.537;

  Eigen::Matrix3d LCamK_;
  Eigen::Matrix3d RCamK_;

  for (int8_t i = 0; i < 3; ++i) {
    for (int8_t j = 0; j < 3; ++j) {
      LCamK_(i, j) = LCamK[i * 3 + j];
      RCamK_(i, j) = RCamK[i * 3 + j];
    }
  }
  config_->setBaseline(baseline);
  config_->setLeftCameraK(std::move(LCamK_));
  config_->setRightCameraK(std::move(RCamK_));

  extractor_->registerConfig(config_);
}
template <typename T> void StereoVisualOdometry<T>::ImageCallback(const Image::ConstSharedPtr &leftImage, const Image::ConstSharedPtr &rightImage) {
  auto leftCVImage = cv_bridge::toCvCopy(leftImage, leftImage->encoding)->image;
  auto rightCVImage = cv_bridge::toCvCopy(rightImage, rightImage->encoding)->image;
  extractor_->registerFairImage(std::move(leftCVImage), std::move(rightCVImage));
  extractor_->compute();

  auto result = extractor_->getResult();
  auto debugFrame = extractor_->getDebugFrame();
  std::vector<Eigen::Vector3d> worldPoints;

  for (auto &match : result.matches) {
    worldPoints.push_back(match.worldPoint);
  }

  auto message = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", debugFrame).toImageMsg();
  message->header.stamp = this->get_clock()->now();
  stereoImagePublisher_->publish(*message);

  // auto pointCloud = Utils::vector3dToPointCloud(worldPoints);
  // pointCloud.header.frame_id = "camera_optical_link";
  // pointCloud.header.stamp = this->get_clock()->now();
  // pointCloudPublisher_->publish(pointCloud);
}
template <typename T> void StereoVisualOdometry<T>::registerExtractor() {
  RCLCPP_INFO(this->get_logger(), "==== registerExtractor =====");
}
template <typename T> void StereoVisualOdometry<T>::registerTracker() {
  RCLCPP_INFO(this->get_logger(), "==== registerTracker =====");
}

template class StereoVisualOdometry<SVO::ORBExtractor>;
} // namespace SVO