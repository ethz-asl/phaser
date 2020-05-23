#include "phaser/backend/correlation/spherical-correlation-laplace.h"

#include <glog/logging.h>

#include "phaser/common/signal-utils.h"

namespace correlation {

SphericalCorrelationLaplace::SphericalCorrelationLaplace(const uint32_t bw)
    : SphericalCorrelation(bw) {}

void SphericalCorrelationLaplace::correlateSampledSignals(
    const std::vector<SampledSignal>& f1,
    const std::vector<SampledSignal>& f2) {
  CHECK_EQ(f1.size(), f2.size());
  CHECK_GT(f1.size(), 0u);
  VLOG(1) << "--- Spherical laplace correlation [" << bw_ << " bw] -----";
  performSphericalTransforms(f1[0], f2[0]);
  correlate();
  inverseTransform();
}

}  // namespace correlation
