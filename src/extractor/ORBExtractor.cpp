#include "stereo_visual_odometry/extractor/ORBExtractor.hpp"

namespace SVO {
ORBExtractor::ORBExtractor() {
  std::cout << "===== ORB Extractor =====" << std::endl;
}
void ORBExtractor::compute() {
  std::cout << "[ORB_EXTRACTOR] compute!!" << std::endl;
}
} // namespace SVO