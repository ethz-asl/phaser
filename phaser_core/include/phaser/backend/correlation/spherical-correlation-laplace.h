#ifndef PHASER_BACKEND_CORRELATION_SPHERICAL_CORRELATION_LAPLACE_H_
#define PHASER_BACKEND_CORRELATION_SPHERICAL_CORRELATION_LAPLACE_H_

#include <vector>

#include "phaser/backend/correlation/spherical-correlation.h"
#include "phaser/backend/fusion/laplace-pyramid.h"

namespace correlation {

class SphericalCorrelationLaplace : public SphericalCorrelation {
 public:
  explicit SphericalCorrelationLaplace(const uint32_t bw = 100);
  virtual ~SphericalCorrelationLaplace() = default;
  void correlateSampledSignals(
      const std::vector<double>& f1, const std::vector<double>& f2) override;

 protected:
  fusion::LaplacePyramid laplace_;
};

}  // namespace correlation

#endif  // PHASER_BACKEND_CORRELATION_SPHERICAL_CORRELATION_LAPLACE_H_
