#include "phaser/backend/correlation/spherical-range-worker.h"

#include <glog/logging.h>

namespace correlation {

SphericalRangeWorker::SphericalRangeWorker(
    const model::FunctionValueVec& f_values,
    const model::FunctionValueVec& h_values, const uint16_t bandwidth)
    : f_values_(f_values),
      h_values_(h_values),
      bw_(bandwidth),
      sph_corr_(bandwidth) {}

void SphericalRangeWorker::run() {
  VLOG(1) << "[SphericalIntensityWorker] Estimating rotation...";

  std::vector<double> f_range;
  std::vector<double> h_range;
  std::function<double(const model::FunctionValue&)> func =
      [](const model::FunctionValue& v) { return v.getAveragedRange(); };
  convertFunctionValues(f_values_, func, &f_range);
  convertFunctionValues(h_values_, func, &h_range);
  sph_corr_.correlateSampledSignals(f_range, h_range);

  is_completed_ = true;
}

std::vector<double> SphericalRangeWorker::getCorrelation() const noexcept {
  return sph_corr_.getCorrelation();
}

const SphericalCorrelation& SphericalRangeWorker::getCorrelationObject() const
    noexcept {
  return sph_corr_;
}

}  // namespace correlation
