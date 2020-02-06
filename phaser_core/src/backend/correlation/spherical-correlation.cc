#include "packlo/backend/correlation/spherical-correlation.h"
#include "packlo/model/function-value.h"

extern "C" {
#include <soft/wrap_fftw.h>
}

#include <glog/logging.h>

#include <algorithm>

namespace backend {

SphericalCorrelation::SphericalCorrelation()
    : statistics_manager_(kReferenceName) {}

void SphericalCorrelation::correlateSignals(
    const std::vector<model::FunctionValue>& f1,
    const std::vector<model::FunctionValue>& f2, const int bw,
    std::array<double, 3>* const zyz) {
  bw_ = bw;
  VLOG(1) << "Starting the correlation with a " << bw << " bandwidth";
  double alpha, beta, gamma, maxcoeff = 0.0;
  constexpr int is_real = 1;

  // Retrieve S2 function values
  std::vector<double> averaged_signal;
  std::vector<double> averaged_pattern;
  retrieveInterpolation(f1, &averaged_signal);
  retrieveInterpolation(f2, &averaged_pattern);

  // Start signal correlation process
  double* signal_coeff;
  double* signal_values;
  softFFTWCor2(
      bw, averaged_signal.data(), averaged_pattern.data(), &alpha, &beta,
      &gamma, &maxcoeff, &signal_values, &signal_coeff, is_real);
  VLOG(2) << "done, result: " << alpha << ", " << beta << ", " << gamma;
  VLOG(2) << "max at: " << maxcoeff;
  const uint32_t len_corr = 8 * bw * bw * bw;
  corr_.assign(signal_values, signal_values + len_corr);

  // Get result
  (*zyz)[0] = std::fmod(alpha, two_pi_);
  (*zyz)[1] = std::fmod(beta, two_pi_);
  (*zyz)[2] = std::fmod(gamma, two_pi_);

  // CHECK_NOTNULL(signal_values);
  // convertSignalValues(signal_values, bw);
  // convertSignalCoeff(signal_coeff, bw);
  delete[] signal_values;
  delete [] signal_coeff;
}

std::array<double, 3> SphericalCorrelation::getZYZFromIndex(
    const uint32_t index) const noexcept {
  std::array<double, 3> zyz;
  convertSO3toZYZ(index, bw_, &zyz);
  return zyz;
}

void SphericalCorrelation::getStatistics(
    common::StatisticsManager* manager) const noexcept {
  manager->mergeManager(statistics_manager_);
}

void SphericalCorrelation::convertSignalValues(
    double *signal_values, const int bw) {
  VLOG(1) << "ADDING CORR KEYS FOR BW " << bw;
  const std::size_t n_values = 8*bw*bw*bw;
  for (std::size_t i = 0u; i < n_values; ++i) {
    statistics_manager_.emplaceValue(kSignalKey, signal_values[i]);
  }
}

void SphericalCorrelation::convertSignalCoeff(
    double *signal_coeff, const int bw) {
  VLOG(1) << "ADDING CORR KEYS FOR BW " << bw;
  const std::size_t n_values = (4*bw*bw*bw-bw)/3;
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
        return interp.getAveragedInterpolation();
      });
}

std::vector<double> SphericalCorrelation::getCorrelation() const noexcept {
  return corr_;
}

void SphericalCorrelation::convertSO3toZYZ(
    const uint32_t loc, const uint32_t bw,
    std::array<double, 3>* const zyz) const {
  const int32_t ii = floor(loc / (4. * bw * bw));
  int32_t tmp = loc - (ii * 4. * bw * bw);
  const int32_t jj = floor(tmp / (2. * bw));
  const int32_t kk = loc - (ii * 4 * bw * bw) - jj * (2 * bw);

  const double bandwith = static_cast<double>(bw);
  (*zyz)[0] = M_PI * jj / (bandwith);
  (*zyz)[1] = M_PI * (2 * ii + 1) / (4. * bw);
  (*zyz)[2] = M_PI * kk / (bandwith);
}
}  // namespace backend
