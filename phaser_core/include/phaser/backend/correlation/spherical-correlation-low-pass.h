#ifndef PHASER_BACKEND_CORRELATION_SPHERICAL_CORRELATION_LOW_PASS_H_
#define PHASER_BACKEND_CORRELATION_SPHERICAL_CORRELATION_LOW_PASS_H_

#include <vector>

#include "phaser/backend/correlation/spherical-correlation.h"

namespace phaser_core {

class SphericalCorrelationLowPass : public SphericalCorrelation {
 public:
  explicit SphericalCorrelationLowPass(
      const uint32_t bw = 100, const uint32_t zero_padding = 0);
  virtual ~SphericalCorrelationLowPass() = default;
  void correlateSampledSignals(
      const std::vector<SampledSignal>& f1,
      const std::vector<SampledSignal>& f2) override;

 protected:
  void filterAndCorrelateCoefficients();
  void shiftSignals(const uint32_t n_points);
  void inverseShiftSignals(const uint32_t n_points);
};

}  // namespace phaser_core

#endif  // PHASER_BACKEND_CORRELATION_SPHERICAL_CORRELATION_LOW_PASS_H_
