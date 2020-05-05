#ifndef PHASER_MODEL_FUNCTION_VALUE_H_
#define PHASER_MODEL_FUNCTION_VALUE_H_

#include <vector>

#include <Eigen/Dense>

#include "phaser/common/point-types.h"

namespace model {

class FunctionValue {
 public:
  FunctionValue();
  explicit FunctionValue(double range, double intensity);

  double getAveragedRange() const noexcept;
  double getAveragedIntensity() const noexcept;
  common::Point_t getAveragedPoint() const noexcept;

  void addRange(const double range);
  void addIntensity(const double intensity);
  void addSemanticClass(const uint16_t class_id);
  void addInstance(const uint16_t instance);
  void addPoint(const common::Point_t& point);

  common::PointCloud_tPtr getAllPoints() const;

 private:
  std::vector<double> range_;
  std::vector<double> intensity_;
  std::vector<uint16_t> semantic_classes_;
  std::vector<uint16_t> semantic_instances_;
  common::PointCloud_tPtr points_;
};

using FunctionValueVec = std::vector<FunctionValue>;

}  // namespace model

#endif  // PHASER_MODEL_FUNCTION_VALUE_H_
