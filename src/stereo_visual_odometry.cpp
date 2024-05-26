#include "stereo_visual_odometry/stereo_visual_odometry.hpp"
namespace SVO {
template <typename T, typename U>
StereoVisualOdometry<T, U>::StereoVisualOdometry(const rclcpp::NodeOptions &options, std::unique_ptr<T> extractor,
                                                 std::unique_ptr<U> tracker)
    : Node("stereo_visual_odometry_node", options), extractor_(std::move(extractor)), tracker_(std::move(tracker)),
      frameCount_(0), debugFlag_(true), latestPose_(Eigen::Matrix4d::Identity()) {
  RCLCPP_INFO(this->get_logger(), "==== stereo_visual_odometry_node =====");
  config_ = std::make_shared<Config>();
  TFbroadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

  initializeParameter();

  stereoImagePublisher_ = this->create_publisher<Image>("/stereo/image", 10);
  trackingImagePublisher_ = this->create_publisher<Image>("/stereo/tracking", 10);
  pointCloudPublisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/stereo/pointcloud", 10);
  pathPublisher_ = this->create_publisher<nav_msgs::msg::Path>("/stereo/path", 10);

  leftImageSub_ = std::make_shared<message_filters::Subscriber<Image>>(this, "/stereo/image_left");
  rightImageSub_ = std::make_shared<message_filters::Subscriber<Image>>(this, "/stereo/image_right");
  sync_ = std::make_shared<message_filters::Synchronizer<ImageSyncPolicy>>(ImageSyncPolicy(10), *leftImageSub_,
                                                                           *rightImageSub_);
  sync_->registerCallback(
      std::bind(&StereoVisualOdometry::ImageCallback, this, std::placeholders::_1, std::placeholders::_2));
}
template <typename T, typename U> void StereoVisualOdometry<T, U>::initializeParameter() {
  /* clang-format off */
  T_optical_world_ << 0.0, 0.0, 1.0, 0.0,
                      -1.0, 0, 0.0, 0.0,
                      0.0, -1.0, 0.0, 0.0,
                      0.0, 0.0, 0.0, 1.0;

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
  /* clang-format on */
  double baseline = 0.537;

  cameraOpticalFrameId_ = this->declare_parameter("camera_optical_frame", "camera_optical_link");
  odomFrameId_ = this->declare_parameter("odom_frame", "odom");
  baseline = this->declare_parameter("baseline", baseline);
  LCamK = this->declare_parameter("leftCamK", LCamK);
  RCamK = this->declare_parameter("rightCamK", RCamK);
  debugFlag_ = this->declare_parameter("debug", true);

  odomTransform_.header.frame_id = odomFrameId_;
  odomTransform_.child_frame_id = cameraOpticalFrameId_;

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
  tracker_->registerConfig(config_);

  std::cout << "===== Configuration =====" << std::endl;
  std::cout << "Left camera intrinsic: \n" << config_->getLeftCameraK() << std::endl;
  std::cout << "Right camera intrinsic: \n" << config_->getRightCameraK() << std::endl;
  std::cout << "Stereo baseline: " << config_->getBaseline() << std::endl;
  std::cout << "Odom frame id: " << odomFrameId_ << std::endl;
  std::cout << "Camera optical frame id: " << cameraOpticalFrameId_ << std::endl;
  std::cout << "Debug: " << debugFlag_ << std::endl;
}
template <typename T, typename U>
void StereoVisualOdometry<T, U>::ImageCallback(const Image::ConstSharedPtr &leftImage,
                                               const Image::ConstSharedPtr &rightImage) {
  auto leftCVImage = cv_bridge::toCvCopy(leftImage, leftImage->encoding)->image;
  auto rightCVImage = cv_bridge::toCvCopy(rightImage, rightImage->encoding)->image;
  extractor_->compute(leftCVImage, rightCVImage);

  prevFrameData_ = frameData_;
  frameData_ = extractor_->getResult();

  if (frameCount_ >= 1) {
    tracker_->compute(prevFrameData_, frameData_);
    auto pose = tracker_->getPose();
    latestPose_ = latestPose_ * pose.matrix();

    geometry_msgs::msg::PoseStamped poseStampMsg;
    poseStampMsg.header.stamp = this->get_clock()->now();
    poseStampMsg.header.frame_id = odomFrameId_;
    poseStampMsg.pose = Utils::EigenToPose(T_optical_world_ * latestPose_);

    poses_.push_back(std::move(poseStampMsg));
  }

  if (debugFlag_) {
    RCLCPP_INFO(this->get_logger(), "current Frame: %d", frameCount_);
    std::vector<Eigen::Vector3d> worldPoints;

    for (auto &match : frameData_.matches) {
      worldPoints.push_back(match.worldPoint);
    }

    auto message =
        cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", extractor_->getDebugFrame(leftCVImage, rightCVImage))
            .toImageMsg();
    message->header.stamp = this->get_clock()->now();
    stereoImagePublisher_->publish(*message);

    auto pointCloud = Utils::vector3dToPointCloud(worldPoints);
    pointCloud.header.frame_id = cameraOpticalFrameId_;
    pointCloud.header.stamp = this->get_clock()->now();
    pointCloudPublisher_->publish(pointCloud);

    if (frameCount_ >= 1) {
      auto trackingImageMsg =
          cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", tracker_->getDebugFrame(prevFrameData_, frameData_))
              .toImageMsg();
      trackingImageMsg->header.stamp = this->get_clock()->now();
      trackingImagePublisher_->publish(*trackingImageMsg);
      publishPath();
    }
  }

  publishOdom();

  ++frameCount_;
}

template <typename T, typename U> void StereoVisualOdometry<T, U>::publishPath() {
  nav_msgs::msg::Path pathMsg;
  pathMsg.header.stamp = this->now();
  pathMsg.header.frame_id = odomFrameId_;
  pathMsg.poses = poses_;

  pathPublisher_->publish(pathMsg);
}

template <typename T, typename U> void StereoVisualOdometry<T, U>::publishOdom() {
  odomTransform_.header.stamp = this->get_clock()->now();

  Eigen::Affine3d transform(T_optical_world_ * latestPose_);
  odomTransform_.transform.translation.x = transform.translation().x();
  odomTransform_.transform.translation.y = transform.translation().y();
  odomTransform_.transform.translation.z = transform.translation().z();

  Eigen::Quaterniond quat(transform.rotation());
  odomTransform_.transform.rotation.x = quat.x();
  odomTransform_.transform.rotation.y = quat.y();
  odomTransform_.transform.rotation.z = quat.z();
  odomTransform_.transform.rotation.w = quat.w();

  TFbroadcaster_->sendTransform(odomTransform_);
}

template class StereoVisualOdometry<SVO::ORBExtractor, SVO::ORBTracker>;
template class StereoVisualOdometry<SVO::ORBExtractor, SVO::OpticalTracker>;
} // namespace SVO