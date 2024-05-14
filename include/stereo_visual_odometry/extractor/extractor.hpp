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
  U leftDesc;
  U rightDesc;
  std::vector<MatchData> matches;
};

template <typename T, typename U> class Extractor {
  using FrameDataT = FrameData<T, U>;

public:
  Extractor() { std::cout << "hello world!!" << std::endl; };
  void registerFairImage(cv::Mat &&leftImage, cv::Mat &&rightImage) {
    leftImage_ = leftImage;
    rightImage_ = rightImage;
    clear();
  };
  void registerConfig(std::shared_ptr<Config> &config) { config_ = config; }
  const FrameDataT &getResult() { return result_; };
  virtual void compute() = 0;
  virtual void clear() = 0;
  virtual cv::Mat getDebugFrame() = 0;

protected:
  cv::Mat leftImage_;
  cv::Mat rightImage_;
  cv::Mat debugImage_;
  FrameDataT result_;
  std::shared_ptr<Config> config_;
};
} // namespace SVO
#endif