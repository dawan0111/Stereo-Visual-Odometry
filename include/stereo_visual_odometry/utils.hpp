#ifndef UTILS_H
#define UTILS_H
#include <Eigen/Core>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
namespace SVO::Utils {
inline sensor_msgs::msg::PointCloud2 vector3dToPointCloud(const std::vector<Eigen::Vector3d> &vector) {
  auto msg = sensor_msgs::msg::PointCloud2();
  msg.height = 1;
  msg.width = vector.size();
  msg.is_dense = false;
  sensor_msgs::PointCloud2Modifier modifier(msg);
  modifier.setPointCloud2Fields(3, "x", 1, sensor_msgs::msg::PointField::FLOAT32, "y", 1,
                                sensor_msgs::msg::PointField::FLOAT32, "z", 1, sensor_msgs::msg::PointField::FLOAT32);

  sensor_msgs::PointCloud2Iterator<float> iter_x(msg, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_y(msg, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_z(msg, "z");

  for (const auto &vec : vector) {
    // std::cout << vec << std::endl;
    *iter_x = static_cast<float>(vec(0));
    *iter_y = static_cast<float>(vec(1));
    *iter_z = static_cast<float>(vec(2));
    ++iter_x;
    ++iter_y;
    ++iter_z;
  }

  return msg;
};
} // namespace SVO::Utils

#endif