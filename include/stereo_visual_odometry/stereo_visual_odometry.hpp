#ifndef STEREO_VISUAL_ODOMETRY_H
#define STEREO_VISUAL_ODOMETRY_H

#include "rclcpp/rclcpp.hpp"
#include "stereo_visual_odometry/config/config.hpp"
#include "stereo_visual_odometry/extractor/ORBExtractor.hpp"
#include "stereo_visual_odometry/tracker/ORBTracker.hpp"
#include "stereo_visual_odometry/tracker/opticalTracker.hpp"
#include "stereo_visual_odometry/utils.hpp"
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
namespace SVO {
template <typename T, typename U> class StereoVisualOdometry : public rclcpp::Node {
public:
  using Image = sensor_msgs::msg::Image;
  using ImageSyncPolicy = message_filters::sync_policies::ApproximateTime<Image, Image>;
  // explicit StereoVisualOdometry(const rclcpp::NodeOptions &);
  explicit StereoVisualOdometry(const rclcpp::NodeOptions &, std::unique_ptr<T> extractor, std::unique_ptr<U> tracker);

  // Deduction guide
  // template <typename T>
  // StereoVisualOdometry(const rclcpp::NodeOptions &options, T &&extractor)
  //     -> StereoVisualOdometry<T>;

private:
  void ImageCallback(const Image::ConstSharedPtr &leftImage, const Image::ConstSharedPtr &rightImage);
  void initializeParameter();
  void publishPath();

private:
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<Image>::SharedPtr stereoImagePublisher_;
  rclcpp::Publisher<Image>::SharedPtr trackingImagePublisher_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointCloudPublisher_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pathPublisher_;
  std::shared_ptr<message_filters::Subscriber<Image>> leftImageSub_;
  std::shared_ptr<message_filters::Subscriber<Image>> rightImageSub_;
  std::shared_ptr<message_filters::Synchronizer<ImageSyncPolicy>> sync_;

  std::shared_ptr<Config> config_;
  std::unique_ptr<T> extractor_;
  std::unique_ptr<U> tracker_;

  typename T::FrameDataT prevFrameData_;
  typename T::FrameDataT frameData_;

  int32_t frameCount_;
  bool debugFlag_;

  std::vector<geometry_msgs::msg::PoseStamped> poses_;
  Eigen::Matrix4d latestPose_;
};
} // namespace SVO

#endif