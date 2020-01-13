#ifndef PACKLO_MODEL_FUNCTION_VALUE_H_
#define PACKLO_MODEL_FUNCTION_VALUE_H_

#include "packlo/common/point-types.h"

#include <Eigen/Dense>
#include <vector>

namespace model {

class FunctionValue {
 public:
  FunctionValue();
  explicit FunctionValue(double interpolation, double range, double intensity);

  double getAveragedInterpolation() const noexcept;
  double getAveragedRange() const noexcept;
  double getAveragedIntensity() const noexcept;
  common::Point_t getAveragedPoint() const noexcept;

  void addInterpolation(const double interpolation);
  void addRange(const double range);
  void addIntensity(const double intensity);
  void addSemanticClass(const uint16_t class_id);
  void addInstance(const uint16_t instance);
  void addPoint(const common::Point_t& point);

  common::PointCloud_tPtr getAllPoints() const;

 private:
  std::vector<double> interpolation_;
  std::vector<double> range_;
  std::vector<double> intensity_;
  std::vector<uint16_t> semantic_classes_;
  std::vector<uint16_t> semantic_instances_;
  common::PointCloud_tPtr points_;
};

}  // namespace model

#endif  // PACKLO_MODEL_FUNCTION_VALUE_H_
