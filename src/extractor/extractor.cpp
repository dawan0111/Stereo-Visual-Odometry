#include "stereo_visual_odometry/extractor/extractor.hpp"

namespace SVO {
template <typename T> Extractor<T>::Extractor() {
  std::cout << "hello world!!" << std::endl;
}
template <typename T> void Extractor<T>::registerFairImage() {
  std::cout << "register Image!!" << std::endl;
}

template <typename T> const FrameData<T> &Extractor<T>::getResult() {
  std::cout << "get Result!!" << std::endl;
  return result_;
}
} // namespace SVO