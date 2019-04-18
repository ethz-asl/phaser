#pragma once

#include <Eigen/Core>

namespace model {

template <typename T>
class PointXYZ {
  public: 
    explicit PointXYZ(T x, T y, T z) 
      : x_(x), y_(y), z_(z) {}

  private:
    T x_;
    T y_;
    T z_;
};

template <typename T>
class PointXYZI : public PointXYZ<T> {
  public: 
    explicit PointXYZI(T x, T y, T z, uint16_t i) 
      : PointXYZ(x,y,z), i_(i) {}

  private:
    uint16_t i_;
};

} // namespace model
