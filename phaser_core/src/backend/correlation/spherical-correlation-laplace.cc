#include "phaser/backend/correlation/spherical-correlation-laplace.h"

#include <glog/logging.h>

#include "phaser/common/signal-utils.h"

namespace correlation {

SphericalCorrelationLaplace::SphericalCorrelationLaplace(const uint32_t bw)
    : SphericalCorrelation(bw) {}

void SphericalCorrelationLaplace::correlateSampledSignals(
    const std::vector<double>& f1, const std::vector<double>& f2) {
  VLOG(1) << "--- Spherical laplace correlation [" << bw_ << " bw] -----";
  performSphericalTransforms(f1, f2);
  correlate();
  inverseTransform();
}

}  // namespace correlation
