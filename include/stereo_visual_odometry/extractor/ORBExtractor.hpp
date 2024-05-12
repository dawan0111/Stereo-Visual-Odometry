#ifndef ORB_EXTRACTOR_H
#define ORB_EXTRACTOR_H
#include "stereo_visual_odometry/extractor/extractor.hpp"
#include <iostream>
namespace SVO {

class ORBExtractor : public Extractor<double> {
public:
  ORBExtractor();
  void compute() override;
};
} // namespace SVO

#endif