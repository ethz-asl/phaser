#ifndef PHASER_BACKEND_CORRELATION_BASE_SPATIAL_CORRELATION_H_
#define PHASER_BACKEND_CORRELATION_BASE_SPATIAL_CORRELATION_H_

#include <memory>

#include <Eigen/Dense>

namespace correlation {

class BaseSpatialCorrelation {
 public:
  double* correlateSignals(Eigen::VectorXd* const f, Eigen::VectorXd* const g);
  virtual double* correlateSignals(double* const f, double* const g) = 0;
};

using BaseSpatialCorrelationPtr = std::unique_ptr<BaseSpatialCorrelation>;

}  // namespace correlation

#endif  // PHASER_BACKEND_CORRELATION_BASE_SPATIAL_CORRELATION_H_
