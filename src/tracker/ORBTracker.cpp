#include "stereo_visual_odometry/tracker/ORBTracker.hpp"

namespace SVO {
ORBTracker::ORBTracker() : Tracker() { std::cout << "ORB_TRACKER" << std::endl; }
void ORBTracker::compute(const FrameDataT &prevFrameData, const FrameDataT &frameData) {
  std::cout << "Compute!!" << std::endl;
}
} // namespace SVO