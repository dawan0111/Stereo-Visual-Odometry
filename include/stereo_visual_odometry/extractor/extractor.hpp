#ifndef EXTRACTOR_H
#define EXTRACTOR_H
#include <iostream>
#include <opencv2/opencv.hpp>
namespace SVO {
template <typename T, typename U> struct FrameData {
  std::vector<T> leftKeyPoint;
  std::vector<T> rightKeyPoint;
  U leftDesc;
  U rightDesc;
  std::vector<std::pair<int16_t, int16_t>> matches;
};

template <typename T, typename U> class Extractor {
  using FrameDataT = FrameData<T, U>;

public:
  Extractor() { std::cout << "hello world!!" << std::endl; };
  void registerFairImage(cv::Mat &&leftImage, cv::Mat &&rightImage) {
    leftImage_ = leftImage;
    rightImage_ = rightImage;
  };
  const FrameDataT &getResult() { return result_; };
  virtual void compute() = 0;
  virtual cv::Mat getDebugFrame() = 0;

protected:
  cv::Mat leftImage_;
  cv::Mat rightImage_;
  cv::Mat debugImage_;
  FrameDataT result_;
};
} // namespace SVO
#endif