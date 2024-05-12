#ifndef EXTRACTOR_H
#define EXTRACTOR_H
#include <iostream>
#include <opencv2/opencv.hpp>
namespace SVO {
template <typename T> struct FrameData {
  std::vector<T> leftKeyPoint;
  std::vector<T> rightKeyPoint;
  std::pair<int16_t, int16_t> pair;
};

template <typename T> class Extractor {
  using FrameDataT = FrameData<T>;

public:
  Extractor() { std::cout << "hello world!!" << std::endl; };
  void registerFairImage() { std::cout << "register Image!!" << std::endl; };
  const FrameDataT &getResult() {
    std::cout << "get Result!!" << std::endl;
    return result_;
  };
  virtual void compute() = 0;

private:
  cv::Mat leftImage_;
  cv::Mat rightImage_;
  FrameDataT result_;
};
} // namespace SVO
#endif