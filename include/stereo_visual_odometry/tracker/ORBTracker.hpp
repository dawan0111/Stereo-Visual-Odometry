#ifndef ORB_TRACKER_H_
#define ORB_TRACKER_H_
#include "stereo_visual_odometry/extractor/ORBExtractor.hpp"
#include "stereo_visual_odometry/tracker/tracker.hpp"

namespace SVO {
class ORBTracker : public Tracker<ORBExtractor::FrameDataT> {
  using FrameDataT = ORBExtractor::FrameDataT;

public:
  ORBTracker();
  void compute(const FrameDataT &prevFrameData, const FrameDataT &frameData) override;
};
} // namespace SVO

#endif