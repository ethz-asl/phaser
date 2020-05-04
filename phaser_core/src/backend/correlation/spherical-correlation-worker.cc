#include "phaser/backend/correlation/spherical-correlation-worker.h"

#include <glog/logging.h>

namespace correlation {

SphericalCorrelationWorker::SphericalCorrelationWorker(
    const model::FunctionValueVec& f_values,
    const model::FunctionValueVec& h_values, const uint16_t bandwidth)
    : f_values_(f_values), h_values_(h_values), bw_(bandwidth) {}

void SphericalCorrelationWorker::run() {
  VLOG(1) << "[SphRegistrationWorker] Estimating rotation...";

  std::vector<double> f_intensities;
  std::vector<double> h_intensities;
  std::function<double(const model::FunctionValue&)> func =
      [](const model::FunctionValue& v) { return v.getAveragedIntensity(); };
  convertFunctionValues(f_values_, func, &f_intensities);
  convertFunctionValues(h_values_, func, &h_intensities);
  sph_corr_.correlateSampledSignals(bw_, f_intensities, h_intensities);

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
