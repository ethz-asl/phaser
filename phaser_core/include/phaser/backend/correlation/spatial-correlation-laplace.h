#ifndef PHASER_BACKEND_CORRELATION_SPATIAL_CORRELATION_LAPLACE_H_
#define PHASER_BACKEND_CORRELATION_SPATIAL_CORRELATION_LAPLACE_H_

#include <vector>

#include "phaser/backend/correlation/spatial-correlation.h"

namespace correlation {

class SpatialCorrelationLaplace : public SpatialCorrelation {
 public:
  explicit SpatialCorrelationLaplace(const uint32_t n_voxels);

  virtual ~SpatialCorrelationLaplace() = default;
  double* correlateSignals(
      const std::vector<Eigen::VectorXd*>& f,
      const std::vector<Eigen::VectorXd*>& g) override;
};

}  // namespace correlation

#endif  // PHASER_BACKEND_CORRELATION_SPATIAL_CORRELATION_LAPLACE_H_
