#ifndef EXTRACTOR_H
#define EXTRACTOR_H
#include "stereo_visual_odometry/config/config.hpp"
#include <Eigen/Core>
#include <iostream>
#include <opencv2/opencv.hpp>
namespace SVO {
struct MatchData {
  int16_t left_i;
  int16_t right_i;
  Eigen::Vector3d worldPoint;
  double distance;

  MatchData(){};
  MatchData(int16_t left_i, int16_t right_i, Eigen::Vector3d &&worldPoint, double distance = 1.0)
      : left_i(left_i), right_i(right_i), worldPoint(worldPoint), distance(distance){};
};
template <typename T, typename U> struct FrameData {
  std::vector<T> leftKeyPoint;
  std::vector<T> rightKeyPoint;
  std::vector<T> trackingKeyPoint;
  U leftDesc;
  U rightDesc;
  U trackingDesc;
  std::vector<MatchData> matches;
  cv::Mat leftImage;
  cv::Mat rightImage;
};

template <typename T, typename U> class Extractor {
public:
  using FrameDataT = FrameData<T, U>;
  Extractor(){};
  void registerConfig(std::shared_ptr<Config> &config) { config_ = config; }
  const FrameDataT &getResult() { return result_; };
  virtual void compute(const cv::Mat &leftImage, const cv::Mat &rightImage) = 0;
  virtual void clear() = 0;
  virtual cv::Mat getDebugFrame(const cv::Mat &leftImage, const cv::Mat &rightImage) = 0;

protected:
  cv::Mat debugImage_;
  FrameDataT result_;
  std::shared_ptr<Config> config_;
};
} // namespace SVO
#endif