#include "phaser/model/function-value.h"

#include <algorithm>
#include <glog/logging.h>
#include <numeric>

namespace model {

FunctionValue::FunctionValue()
    : range_(0.0), intensity_(0.0), points_(new common::PointCloud_t) {}

FunctionValue::FunctionValue(double range, double intensity)
    : range_(range), intensity_(intensity), points_(new common::PointCloud_t) {}

double FunctionValue::getAveragedRange() const noexcept {
  if (range_.empty()) {
    return 0.0;
  }
  return std::accumulate(range_.cbegin(), range_.cend(), 0.0) /
         static_cast<double>(range_.size());
}

double FunctionValue::getAveragedIntensity() const noexcept {
  if (intensity_.empty()) {
    return 0.0;
  }
  return std::accumulate(intensity_.cbegin(), intensity_.cend(), 0.0) /
         static_cast<double>(intensity_.size());
}

double FunctionValue::getAveragedReflectivity() const noexcept {
  if (reflectivity_.empty()) {
    return 0.0;
  }
  return std::accumulate(reflectivity_.cbegin(), reflectivity_.cend(), 0.0) /
         static_cast<double>(reflectivity_.size());
}

double FunctionValue::getAveragedAmbientNoise() const noexcept {
  if (ambient_noise_.empty()) {
    return 0.0;
  }
  return std::accumulate(ambient_noise_.cbegin(), ambient_noise_.cend(), 0.0) /
         static_cast<double>(ambient_noise_.size());
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

void FunctionValue::addRange(const double range) {
  range_.emplace_back(range);
}

void FunctionValue::addIntensity(const double intensity) {
  intensity_.emplace_back(intensity);
}

void FunctionValue::addReflectivity(const double reflectivity) {
  reflectivity_.emplace_back(reflectivity);
}

void FunctionValue::addAmbientNoise(const double ambient) {
  ambient_noise_.emplace_back(ambient);
}

void FunctionValue::addSemanticClass(const uint16_t class_id) {
  semantic_classes_.emplace_back(class_id);
}

void FunctionValue::addInstance(const uint16_t instance) {
  semantic_instances_.emplace_back(instance);
}

void FunctionValue::addPoint(const common::Point_t& point) {
  points_->push_back(point);
}

common::PointCloud_tPtr FunctionValue::getAllPoints() const {
  return points_;
}

}  // namespace model
