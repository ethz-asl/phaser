#include "packlo/model/function-value.h"

#include <numeric>
#include <algorithm>

#include <glog/logging.h>

namespace model {

FunctionValue::FunctionValue()
  : interpolation_(0.0), range_(0.0), intensity_(0.0), 
    points_(new common::PointCloud_t) {}

FunctionValue::FunctionValue(double interpolation, 
    double range, double intensity) 
  : interpolation_(interpolation), 
    range_(range), intensity_(intensity), 
    points_(new common::PointCloud_t) {
  
}

double FunctionValue::getAveragedInterpolation() const noexcept {
  return std::accumulate(interpolation_.cbegin(), interpolation_.cend(), 
      0.0) / static_cast<double>(interpolation_.size());
}

double FunctionValue::getAveragedRange() const noexcept {
  return std::accumulate(range_.cbegin(), range_.cend(), 
      0.0) / static_cast<double>(range_.size());
}

double FunctionValue::getAveragedIntensity() const noexcept {
  return std::accumulate(intensity_.cbegin(), intensity_.cend(), 
      0.0) / static_cast<double>(intensity_.size());
}

common::Point_t FunctionValue::getAveragedPoint() const noexcept {
  common::Point_t avg;
  const auto& points = points_->points;
  for (const common::Point_t& p : points) {
    avg.x += p.x;
    avg.y += p.y;
    avg.z += p.z;
  }
  const float n_points = static_cast<float>(points.size());
  avg.x = avg.x / n_points;
  avg.y = avg.y / n_points;
  avg.z = avg.z / n_points;
  return avg;
}

void FunctionValue::addInterpolation(const double interpolation) {
  interpolation_.emplace_back(interpolation);
}

void FunctionValue::addRange(const double range) {
  range_.emplace_back(range);

}

void FunctionValue::addIntensity(const double intensity) {
  intensity_.emplace_back(intensity);
}

void FunctionValue::addPoint(const common::Point_t& point) {
  points_->push_back(point);
}

common::PointCloud_tPtr FunctionValue::getAllPoints() const {
  return points_;
}

} // namespace model
