#include "phaser/backend/correlation/spherical-range-worker.h"

#include <glog/logging.h>

#include "phaser/common/core-gflags.h"

namespace phaser_core {

SphericalRangeWorker::SphericalRangeWorker(
    const model::FunctionValueVec& f_values,
    const model::FunctionValueVec& h_values)
    : f_values_(f_values),
      h_values_(h_values),
      sph_corr_(FLAGS_phaser_core_spherical_bandwidth) {}

void SphericalRangeWorker::run() {
  VLOG(1) << "[SphericalIntensityWorker] Estimating rotation...";

  SampledSignal f_range;
  SampledSignal h_range;
  std::function<double(const model::FunctionValue&)> func =
      [](const model::FunctionValue& v) { return v.getAveragedRange(); };
  convertFunctionValues(f_values_, func, &f_range);
  convertFunctionValues(h_values_, func, &h_range);

  VLOG(1) << "===============================================================";
  for (uint32_t i = 0u; i < f_values_.size(); ++i) {
    if (std::isnan(f_range[i]) || std::isnan(h_range[i]))
      VLOG(1) << "rnage: " << f_range[i] << ", " << h_range[i] << " i: " << i;
  }
  VLOG(1) << "===============================================================";
  sph_corr_.correlateSampledSignals({f_range}, {h_range});

  is_completed_ = true;
}

std::vector<double> SphericalRangeWorker::getCorrelation() const noexcept {
  return sph_corr_.getCorrelation();
}

const SphericalCorrelation& SphericalRangeWorker::getCorrelationObject() const
    noexcept {
  return sph_corr_;
}

}  // namespace phaser_core
