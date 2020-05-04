#include "phaser/backend/correlation/spherical-correlation-worker.h"

#include <glog/logging.h>

namespace correlation {

SphericalCorrelationWorker::SphericalCorrelationWorker(
    const model::FunctionValueVec& f_values,
    const model::FunctionValueVec& h_values, const uint16_t bandwidth)
    : f_values_(f_values), h_values_(h_values), bw_(bandwidth) {}

void SphericalCorrelationWorker::run() {
  VLOG(1) << "[SphRegistrationWorker] Estimating rotation...";

  sph_corr_.correlateSignals(f_values_, h_values_, bw_);

  is_completed_ = true;
}

std::vector<double> SphericalCorrelationWorker::getCorrelation() const
    noexcept {
  return sph_corr_.getCorrelation();
}

const SphericalCorrelation& SphericalCorrelationWorker::getCorrelationObject()
    const noexcept {
  return sph_corr_;
}

}  // namespace correlation
