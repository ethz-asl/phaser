#include "phaser/backend/correlation/spherical-intensity-worker.h"

#include <glog/logging.h>

namespace correlation {

SphericalIntensityWorker::SphericalIntensityWorker(
    const model::FunctionValueVec& f_values,
    const model::FunctionValueVec& h_values, const uint16_t bandwidth)
    : f_values_(f_values),
      h_values_(h_values),
      bw_(bandwidth),
      sph_corr_(bandwidth) {}

void SphericalIntensityWorker::run() {
  VLOG(1) << "[SphericalIntensityWorker] Estimating rotation...";

  std::vector<double> f_intensities;
  std::vector<double> h_intensities;
  std::function<double(const model::FunctionValue&)> func =
      [](const model::FunctionValue& v) { return v.getAveragedIntensity(); };
  convertFunctionValues(f_values_, func, &f_intensities);
  convertFunctionValues(h_values_, func, &h_intensities);
  sph_corr_.correlateSampledSignals(f_intensities, h_intensities);

  is_completed_ = true;
}

std::vector<double> SphericalIntensityWorker::getCorrelation() const noexcept {
  return sph_corr_.getCorrelation();
}

const SphericalCorrelation& SphericalIntensityWorker::getCorrelationObject()
    const noexcept {
  return sph_corr_;
}

}  // namespace correlation
