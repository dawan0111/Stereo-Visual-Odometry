#ifndef TRACKER_H_
#define TRACKER_H_
#include "stereo_visual_odometry/config/config.hpp"
#include <sophus/se3.hpp>
namespace SVO {
template <typename T> class Tracker {
public:
  Tracker(){};
  const Sophus::SE3d &getPose() { return pose_; };
  void registerConfig(std::shared_ptr<Config> &config) { config_ = config; }
  virtual void compute(const T &prevFrameData, const T &frameData) = 0;

protected:
  Sophus::SE3d pose_;
  std::shared_ptr<Config> config_;

protected:
  void clear(){};
};
} // namespace SVO

#endif