#include "phaser/backend/correlation/spherical-correlation.h"
#include "phaser/model/function-value.h"

extern "C" {
#include <soft/wrap_fftw.h>
}

#include <glog/logging.h>

#include <algorithm>

namespace correlation {

SphericalCorrelation::SphericalCorrelation()
    : statistics_manager_(kReferenceName) {}

void SphericalCorrelation::correlateSignals(
    const std::vector<model::FunctionValue>& f1,
    const std::vector<model::FunctionValue>& f2, const int bw) {
  bw_ = bw;
  VLOG(1) << "Starting the correlation with a " << bw << " bandwidth";

  // Retrieve S2 function values
  std::vector<double> averaged_signal;
  std::vector<double> averaged_pattern;
  retrieveInterpolation(f1, &averaged_signal);
  retrieveInterpolation(f2, &averaged_pattern);

  // Start signal correlation process
  correlateSampledSignals(bw, averaged_signal, averaged_pattern);
}

void SphericalCorrelation::correlateSampledSignals(
    const int bw, std::vector<double>& f1, std::vector<double>& f2) {
  VLOG(1) << "Starting the correlation with a " << bw << " bandwidth";
  bw_ = bw;
  double* signal_values;
  constexpr int is_real = 1;
  softFFTWCor2(bw, f1.data(), f2.data(), &signal_values, is_real);
  CHECK_NOTNULL(signal_values);

  const uint32_t len_corr = 8 * bw * bw * bw;
  corr_.assign(signal_values, signal_values + len_corr);
  delete[] signal_values;
}

void SphericalCorrelation::getStatistics(
    common::StatisticsManager* manager) const noexcept {
  manager->mergeManager(statistics_manager_);
}

void SphericalCorrelation::convertSignalValues(
    double* signal_values, const int bw) {
  VLOG(1) << "ADDING CORR KEYS FOR BW " << bw;
  const std::size_t n_values = 8 * bw * bw * bw;
  for (std::size_t i = 0u; i < n_values; ++i) {
    statistics_manager_.emplaceValue(kSignalKey, signal_values[i]);
  }
}

void SphericalCorrelation::convertSignalCoeff(
    double* signal_coeff, const int bw) {
  VLOG(1) << "ADDING CORR KEYS FOR BW " << bw;
  const std::size_t n_values = (4 * bw * bw * bw - bw) / 3;
  for (std::size_t i = 0u; i < n_values; ++i) {
    statistics_manager_.emplaceValue(kCoeffKey, signal_coeff[i]);
  }
}

void SphericalCorrelation::retrieveInterpolation(
    const std::vector<model::FunctionValue>& f,
    std::vector<double>* interpolation) {
  std::transform(
      f.cbegin(), f.cend(), std::back_inserter(*interpolation),
      [](const model::FunctionValue& interp) {
        return interp.getAveragedIntensity();
      });
}

std::vector<double> SphericalCorrelation::getCorrelation() const noexcept {
  return corr_;
}

uint32_t SphericalCorrelation::getBandwidth() const noexcept {
  return bw_;
}

}  // namespace correlation
