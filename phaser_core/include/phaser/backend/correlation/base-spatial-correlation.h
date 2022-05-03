#ifndef PHASER_BACKEND_CORRELATION_BASE_SPATIAL_CORRELATION_H_
#define PHASER_BACKEND_CORRELATION_BASE_SPATIAL_CORRELATION_H_

#include <Eigen/Dense>
#include <memory>
#include <vector>

namespace phaser_core {

class BaseSpatialCorrelation {
 public:
  virtual double* correlateSignals(
      const std::vector<Eigen::VectorXd*>& f,
      const std::vector<Eigen::VectorXd*>& g) = 0;
};

using BaseSpatialCorrelationPtr = std::unique_ptr<BaseSpatialCorrelation>;

}  // namespace phaser_core

#endif  // PHASER_BACKEND_CORRELATION_BASE_SPATIAL_CORRELATION_H_
