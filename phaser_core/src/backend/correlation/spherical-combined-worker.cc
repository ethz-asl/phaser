#include "phaser/backend/correlation/spherical-combined-worker.h"

#include <glog/logging.h>

#include "phaser/backend/correlation/spherical-correlation-low-pass.h"
#include "phaser/backend/correlation/spherical-correlation.h"
#include "phaser/common/core-gflags.h"

namespace correlation {

SphericalCombinedWorker::SphericalCombinedWorker(
    const model::FunctionValueVec& f_values,
    const model::FunctionValueVec& h_values)
    : f_values_(f_values), h_values_(h_values) {
  sph_corr_.reset(new SphericalCorrelation(
      common::FLAGS_phaser_core_spherical_bandwidth,
      common::FLAGS_phaser_core_spherical_zero_padding));
}

void SphericalCombinedWorker::run() {
  CHECK_NOTNULL(sph_corr_);
  VLOG(1) << "[SphericalCombinedWorker] Estimating rotation...";

  // Get the intensities.
  correlation::SampledSignal f_intensities;
  correlation::SampledSignal h_intensities;
  std::function<double(const model::FunctionValue&)> func_intensities =
      [](const model::FunctionValue& v) { return v.getAveragedIntensity(); };
  convertFunctionValues(f_values_, func_intensities, &f_intensities);
  convertFunctionValues(h_values_, func_intensities, &h_intensities);

  // Get the ranges.
  correlation::SampledSignal f_range;
  correlation::SampledSignal h_range;
  std::function<double(const model::FunctionValue&)> func_range =
      [](const model::FunctionValue& v) { return v.getAveragedRange(); };
  convertFunctionValues(f_values_, func_range, &f_range);
  convertFunctionValues(h_values_, func_range, &h_range);

  sph_corr_->correlateSampledSignals(
      {f_intensities, f_range}, {h_intensities, h_range});
  is_completed_ = true;
}

std::vector<double> SphericalCombinedWorker::getCorrelation() const noexcept {
  CHECK_NOTNULL(sph_corr_);
  return sph_corr_->getCorrelation();
}

const SphericalCorrelation& SphericalCombinedWorker::getCorrelationObject()
    const noexcept {
  CHECK_NOTNULL(sph_corr_);
  return *sph_corr_;
}

}  // namespace correlation
