#include "phaser/backend/correlation/spherical-correlation.h"
#include "phaser/model/function-value.h"

extern "C" {
#include "soft/makeweights.h"
#include "soft/s2_cospmls.h"
#include "soft/s2_legendreTransforms.h"
#include "soft/s2_semi_memo.h"
#include "soft/so3_correlate_fftw.h"
#include "soft/soft_fftw.h"
#include "soft/wrap_fftw.h"
}

#include <glog/logging.h>

#include <algorithm>

namespace correlation {

SphericalCorrelation::SphericalCorrelation(const uint32_t bw)
    : statistics_manager_(kReferenceName), bw_(bw) {
  initializeAll(bw);
}

void SphericalCorrelation::correlateSignals(
    const std::vector<model::FunctionValue>& f1,
    const std::vector<model::FunctionValue>& f2, const int bw) {
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

void SphericalCorrelation::initializeAll(const uint32_t bw) {
  const uint32_t bwp2 = bw * bw;
  const uint32_t bwp3 = bwp2 * bw;
  const uint32_t so3bw = 8u * bwp3;
  const uint32_t n = 2 * bw;
  rank_ = 1;
  dims_[0].n = n;
  dims_[0].is = 1;
  dims_[0].os = n;
  howmany_rank_ = 1;
  howmany_dims_[0].n = n;
  howmany_dims_[0].is = n;
  howmany_dims_[0].os = 1;

  workspace1_ =
      static_cast<fftw_complex*>(fftw_malloc(sizeof(fftw_complex) * (so3bw)));
  workspace2_ = static_cast<fftw_complex*>(
      fftw_malloc(sizeof(fftw_complex) * ((14 * bwp2) + (48 * bw))));
  workspace3_ =
      static_cast<double*>(malloc(sizeof(double) * (12 * n + n * bw)));

  weights_ = static_cast<double*>(malloc(sizeof(double) * (4 * bw)));
  sig_coef_[0] = static_cast<double*>(malloc(sizeof(double) * bwp2));
  sig_coef_[1] = static_cast<double*>(malloc(sizeof(double) * bwp2));
  pat_coef_[0] = static_cast<double*>(malloc(sizeof(double) * bwp2));
  pat_coef_[1] = static_cast<double*>(malloc(sizeof(double) * bwp2));
  tmp_coef_[0] = static_cast<double*>(malloc(sizeof(double) * (n * n)));
  tmp_coef_[1] = static_cast<double*>(malloc(sizeof(double) * (n * n)));
  so3_sig_ =
      static_cast<fftw_complex*>(fftw_malloc(sizeof(fftw_complex) * (so3bw)));
  so3_coef_ = static_cast<fftw_complex*>(
      fftw_malloc(sizeof(fftw_complex) * ((4 * bwp3 - bw) / 3)));
  seminaive_naive_tablespace_ = static_cast<double*>(malloc(
      sizeof(double) *
      (Reduced_Naive_TableSize(bw, bw) + Reduced_SpharmonicTableSize(bw, bw))));

  dct_plan_ =
      fftw_plan_r2r_1d(n, weights_, workspace3_, FFTW_REDFT10, FFTW_ESTIMATE);

  fft_plan_ = fftw_plan_guru_split_dft(
      rank_, dims_, howmany_rank_, howmany_dims_, tmp_coef_[0], tmp_coef_[1],
      reinterpret_cast<double*>(workspace2_),
      reinterpret_cast<double*>(workspace2_) + (n * n), FFTW_ESTIMATE);

  howmany_ = n * n;
  idist_ = n;
  odist_ = n;
  rank_ = 2;
  inembed_[0] = n;
  inembed_[1] = n * n;
  onembed_[0] = n;
  onembed_[1] = n * n;
  istride_ = 1;
  ostride_ = 1;
  na_[0] = 1;
  na_[1] = n;

  inverse_so3_ = fftw_plan_many_dft(
      rank_, na_, howmany_, workspace1_, inembed_, istride_, idist_, so3_sig_,
      onembed_, ostride_, odist_, FFTW_FORWARD, FFTW_ESTIMATE);

  seminaive_naive_table_ = SemiNaive_Naive_Pml_Table(
      bw, bw, seminaive_naive_tablespace_,
      reinterpret_cast<double*>(workspace2_));

  // make quadrature weights for the S^2 transform.
  makeweights(bw, weights_);
}

}  // namespace correlation
