#ifndef TRACKER_H_
#define TRACKER_H_

namespace SVO {
template <typename T> class Tracker {
public:
  Tracker(){};
  double getResult() { return result_; };
  virtual void compute(const T &prevFrameData, const T &frameData) = 0;

protected:
  double result_;

protected:
  void clear(){};
};
} // namespace SVO

#endif