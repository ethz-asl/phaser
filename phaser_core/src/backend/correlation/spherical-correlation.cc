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
SphericalCorrelation::~SphericalCorrelation() {
  /*
    free(seminaive_naive_table_);
    free(seminaive_naive_tablespace_);
    fftw_destroy_plan(inverse_so3_);
  fftw_destroy_plan(inverse_so3_);
  fftw_destroy_plan(fft_plan_);
  fftw_destroy_plan(dct_plan_);
  fftw_free(workspace1_);
  fftw_free(workspace2_);
  fftw_free(so3_sig_);
  fftw_free(so3_coef_);
  free(workspace3_);
  free(weights_);
  for (std::size_t i = 0u; i < 2u; ++i) {
    free(pat_coef_[i]);
    free(sig_coef_[i]);
    free(tmp_coef_[i]);
  }
  */
}

void SphericalCorrelation::correlateSignals(
    const std::vector<model::FunctionValue>& f1,
    const std::vector<model::FunctionValue>& f2) {
  // Retrieve S2 function values
  std::vector<double> averaged_signal;
  std::vector<double> averaged_pattern;
  retrieveInterpolation(f1, &averaged_signal);
  retrieveInterpolation(f2, &averaged_pattern);

  // Start signal correlation process
  correlateSampledSignals(averaged_signal, averaged_pattern);
}

void SphericalCorrelation::correlateSampledSignals(
    const std::vector<double>& f1, const std::vector<double>& f2) {
  VLOG(1) << "Starting the correlation with a " << bw_ << " bandwidth";
  performSphericalTransforms(f1, f2);
  correlateAndInverseTransform();
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
  return so3_mag_sig_;
}

uint32_t SphericalCorrelation::getBandwidth() const noexcept {
  return bw_;
}

void SphericalCorrelation::initializeAll(const uint32_t bw) {
  const uint32_t bwp2 = bw * bw;
  const uint32_t bwp3 = bwp2 * bw;
  so3_bw_ = 8u * bwp3;
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
      static_cast<fftw_complex*>(fftw_malloc(sizeof(fftw_complex) * (so3_bw_)));
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
      static_cast<fftw_complex*>(fftw_malloc(sizeof(fftw_complex) * (so3_bw_)));
  so3_coef_ = static_cast<fftw_complex*>(
      fftw_malloc(sizeof(fftw_complex) * ((4 * bwp3 - bw) / 3)));
  seminaive_naive_tablespace_ = static_cast<double*>(malloc(
      sizeof(double) *
      (Reduced_Naive_TableSize(bw, bw) + Reduced_SpharmonicTableSize(bw, bw))));
  so3_mag_sig_ = std::vector<double>(so3_bw_);

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

void SphericalCorrelation::performSphericalTransforms(
    const std::vector<double>& f1, const std::vector<double>& f2) {
  CHECK_NOTNULL(tmp_coef_[0]);
  CHECK_NOTNULL(tmp_coef_[1]);
  CHECK_NOTNULL(sig_coef_[0]);
  CHECK_NOTNULL(sig_coef_[1]);
  CHECK_NOTNULL(pat_coef_[0]);
  CHECK_NOTNULL(pat_coef_[1]);
  CHECK_NOTNULL(workspace2_);
  CHECK_NOTNULL(dct_plan_);
  CHECK_NOTNULL(fft_plan_);
  CHECK_NOTNULL(weights_);

#pragma omp parallel for num_threads(2)
  for (uint32_t i = 0u; i < howmany_; ++i) {
    tmp_coef_[0][i] = f1[i];
    tmp_coef_[1][i] = 0.;
  }

  // Perform spherical transform of f1.
  FST_semi_memo(
      tmp_coef_[0], tmp_coef_[1], sig_coef_[0], sig_coef_[1], bw_,
      seminaive_naive_table_, reinterpret_cast<double*>(workspace2_), 1, bw_,
      &dct_plan_, &fft_plan_, weights_);

#pragma omp parallel for num_threads(2)
  for (uint32_t i = 0u; i < howmany_; ++i) {
    tmp_coef_[0][i] = f2[i];
    tmp_coef_[1][i] = 0.;
  }

  // Perform spherical transform of f2.
  FST_semi_memo(
      tmp_coef_[0], tmp_coef_[1], pat_coef_[0], pat_coef_[1], bw_,
      seminaive_naive_table_, reinterpret_cast<double*>(workspace2_), 1, bw_,
      &dct_plan_, &fft_plan_, weights_);
}

void SphericalCorrelation::correlateAndInverseTransform() {
  CHECK_NOTNULL(sig_coef_[0]);
  CHECK_NOTNULL(sig_coef_[1]);
  CHECK_NOTNULL(pat_coef_[0]);
  CHECK_NOTNULL(pat_coef_[1]);
  CHECK_NOTNULL(so3_sig_);
  CHECK_NOTNULL(so3_coef_);
  CHECK_NOTNULL(workspace1_);
  CHECK_NOTNULL(workspace2_);
  CHECK_NOTNULL(workspace3_);

  so3CombineCoef_fftw(
      bw_, bw_, bw_ - 1, sig_coef_[0], sig_coef_[1], pat_coef_[0], pat_coef_[1],
      so3_coef_);
  Inverse_SO3_Naive_fftw(
      bw_, so3_coef_, so3_sig_, workspace1_, workspace2_, workspace3_,
      &inverse_so3_, 1);

  so3_mag_sig_ = std::vector<double>(so3_bw_);
#pragma omp parallel for num_threads(2)
  for (uint32_t i = 0; i < so3_bw_; ++i) {
    const double real = so3_sig_[i][0];
    const double imag = so3_sig_[i][1];
    so3_mag_sig_[i] = real * real + imag * imag;
  }
}

}  // namespace correlation
