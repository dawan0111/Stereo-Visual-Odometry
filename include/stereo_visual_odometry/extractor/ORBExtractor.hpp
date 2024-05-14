#ifndef ORB_EXTRACTOR_H
#define ORB_EXTRACTOR_H
#include "stereo_visual_odometry/extractor/extractor.hpp"
#include <iostream>
#include <opencv2/opencv.hpp>
namespace SVO {

class ORBExtractor : public Extractor<cv::KeyPoint, cv::Mat> {
public:
  ORBExtractor();
  void compute() override;
  cv::Mat getDebugFrame() override;
  void clear() override;

private:
  cv::Ptr<cv::Feature2D> detector_;
  cv::Ptr<cv::DescriptorMatcher> matcher_;
  std::vector<cv::DMatch> matches_;
};
} // namespace SVO

#endif