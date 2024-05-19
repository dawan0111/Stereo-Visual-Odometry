#ifndef ORB_TRACKER_H_
#define ORB_TRACKER_H_
#include "stereo_visual_odometry/extractor/ORBExtractor.hpp"
#include "stereo_visual_odometry/optimizer/CV_Optimizer.hpp"
#include "stereo_visual_odometry/optimizer/G2O_Optimizer.hpp"
#include "stereo_visual_odometry/tracker/tracker.hpp"
#include <g2o/core/solver.h>
#include <opencv2/opencv.hpp>

namespace SVO {
class ORBTracker : public Tracker<ORBExtractor::FrameDataT> {
  using FrameDataT = ORBExtractor::FrameDataT;

public:
  ORBTracker();
  void compute(const FrameDataT &prevFrameData, const FrameDataT &frameData) override;
  cv::Mat getDebugFrame(const FrameDataT &prevFrameData, const FrameDataT &frameData) override;

private:
  std::unique_ptr<G2O_Optimizer> optimizer_;
  std::unique_ptr<CV_Optimizer> CV_Optimizer_;
  cv::Ptr<cv::DescriptorMatcher> matcher_;
  std::vector<cv::DMatch> matches_;
};
} // namespace SVO

#endif