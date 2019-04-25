#pragma once

#include <Eigen/Core>

namespace model {

template <typename T>
class PointXYZ {
  public: 
    explicit PointXYZ(T x, T y, T z) 
      : x_(x), y_(y), z_(z) {}

    T& x() { return x_; }
    T x() const noexcept { return x_; }

    T& y() { return y_; }
    T y() const noexcept { return y_; }

    T& z() { return z_; }
    T z() const noexcept { return z_; }

  private:
    T x_;
    T y_;
    T z_;
};

template <typename T>
class PointXYZI : public PointXYZ<T> {
  public: 
    explicit PointXYZI(T x, T y, T z, uint16_t i) 
      : PointXYZ<T>(x,y,z), i_(i) {}

    uint16_t& z() { return i_; }
    uint16_t z() const noexcept { return i_; }

  private:
    uint16_t i_;
};

using PointXYZIf = PointXYZI<float>;

} // namespace model
