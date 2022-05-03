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

#include <algorithm>
#include <fstream>
#include <glog/logging.h>

namespace phaser_core {

SphericalCorrelation::SphericalCorrelation(
    const uint32_t bw, const uint32_t zero_padding)
    : statistics_manager_(kReferenceName),
      bw_(bw),
      bw_out_(bw + zero_padding),
      zero_padding_(zero_padding) {
  initializeAll(bw);
}

void SphericalCorrelation::shutdown() {
  if (!is_initialized_) {
    LOG(ERROR) << "Can't shutdown. Correlation is not initialized";
    return;
  }
  free(workspace3_);
  free(weights_);
  free(seminaive_naive_table_);
  free(seminaive_naive_tablespace_);
  fftw_destroy_plan(inverse_so3_);
  fftw_destroy_plan(fft_plan_);
  fftw_destroy_plan(dct_plan_);
  fftw_free(workspace1_);
  fftw_free(workspace2_);
  fftw_free(so3_sig_);
  fftw_free(so3_coef_);
  for (std::size_t i = 0u; i < 2u; ++i) {
    free(pat_coef_[i]);
    free(sig_coef_[i]);
    free(tmp_coef_[i]);
  }
  is_initialized_ = false;
}

void SphericalCorrelation::correlateSignals(
    const std::vector<model::FunctionValue>& f1,
    const std::vector<model::FunctionValue>& f2) {
  if (!is_initialized_)
    LOG(FATAL) << "[SphericalCorrelation] Is not initialized!";
  // Retrieve S2 function values
  SampledSignal averaged_signal;
  SampledSignal averaged_pattern;
  retrieveInterpolation(f1, &averaged_signal);
  retrieveInterpolation(f2, &averaged_pattern);

  // Start signal correlation process
  correlateSampledSignals({averaged_signal}, {averaged_pattern});
}

void SphericalCorrelation::correlateSampledSignals(
    const std::vector<SampledSignal>& f1,
    const std::vector<SampledSignal>& f2) {
  if (!is_initialized_)
    LOG(FATAL) << "[SphericalCorrelation] Is not initialized!";
  CHECK_EQ(f1.size(), f2.size());
  CHECK_GT(f1.size(), 0u);
  VLOG(1) << "Starting the correlation with a " << bw_ << " bandwidth";
  performSphericalTransforms(f1[0], f2[0]);
  correlate();
  inverseTransform();
}

void SphericalCorrelation::getStatistics(
    common::StatisticsManager* manager) const noexcept {
  manager->mergeManager(statistics_manager_);
}

void SphericalCorrelation::convertSignalValues(
    double* signal_values, const int bw) {
  const std::size_t n_values = 8 * bw * bw * bw;
  for (std::size_t i = 0u; i < n_values; ++i) {
    statistics_manager_.emplaceValue(kSignalKey, signal_values[i]);
  }
}

void SphericalCorrelation::convertSignalCoeff(
    double* signal_coeff, const int bw) {
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
        return 0.6 * interp.getAveragedIntensity() +
               0.4 * interp.getAveragedRange();
      });
}

std::vector<double> SphericalCorrelation::getCorrelation() const noexcept {
  return so3_mag_sig_;
}

uint32_t SphericalCorrelation::getBandwidth() const noexcept {
  return bw_out_;
}

void SphericalCorrelation::initializeAll(const uint32_t bw) {
  VLOG(2) << "Initializing spherical correlation with a " << bw << " bandwidth";
  const uint32_t bwp2 = bw * bw;
  const uint32_t bwp3 = bwp2 * bw;

  const uint32_t bw_out_p2 = bw_out_ * bw_out_;
  const uint32_t bw_out_p3 = bw_out_p2 * bw_out_;
  so3_bw_ = 8u * bw_out_p3;

  const uint32_t n = 2 * bw;
  const uint32_t n_out = 2 * bw_out_;

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

  workspace2_out_ = static_cast<fftw_complex*>(
      fftw_malloc(sizeof(fftw_complex) * ((14 * bw_out_p2) + (48 * bw_out_))));
  workspace3_out_ = static_cast<double*>(
      malloc(sizeof(double) * (12 * n_out + n_out * bw_out_)));

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
      fftw_malloc(sizeof(fftw_complex) * ((4 * bw_out_p3 - bw_out_) / 3)));

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
  const uint32_t howmany_out = n_out * n_out;
  idist_ = n_out;
  odist_ = n_out;
  rank_ = 2;
  inembed_[0] = n_out;
  inembed_[1] = n_out * n_out;
  onembed_[0] = n_out;
  onembed_[1] = n_out * n_out;
  istride_ = 1;
  ostride_ = 1;
  na_[0] = 1;
  na_[1] = n_out;

  inverse_so3_ = fftw_plan_many_dft(
      rank_, na_, howmany_out, workspace1_, inembed_, istride_, idist_,
      so3_sig_, onembed_, ostride_, odist_, FFTW_FORWARD, FFTW_ESTIMATE);

  seminaive_naive_table_ = SemiNaive_Naive_Pml_Table(
      bw, bw, seminaive_naive_tablespace_,
      reinterpret_cast<double*>(workspace2_));

  // make quadrature weights for the S^2 transform.
  makeweights(bw, weights_);
  is_initialized_ = true;
}

void SphericalCorrelation::performSphericalTransforms(
    const std::vector<double>& f1, const std::vector<double>& f2) {
  VLOG(1) << "Performing spherical transformations for input.";
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

  VLOG(2) << "Performing SFT of the first signal.";
  for (uint32_t i = 0u; i < howmany_; ++i) {
    tmp_coef_[0][i] = f1[i];
    tmp_coef_[1][i] = 0.;
  }

  // Perform spherical transform of f1.
  FST_semi_memo(
      tmp_coef_[0], tmp_coef_[1], sig_coef_[0], sig_coef_[1], bw_,
      seminaive_naive_table_, reinterpret_cast<double*>(workspace2_), 1, bw_,
      &dct_plan_, &fft_plan_, weights_);

  VLOG(2) << "Performing SFT of the second signal.";
  // #pragma omp parallel for num_threads(2)
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

void SphericalCorrelation::correlate() {
  CHECK_NOTNULL(sig_coef_[0]);
  CHECK_NOTNULL(sig_coef_[1]);
  CHECK_NOTNULL(pat_coef_[0]);
  CHECK_NOTNULL(pat_coef_[1]);
  CHECK_NOTNULL(so3_coef_);
  VLOG(2) << "Performing correlation of the S^2 coefficients.";
  so3CombineCoef_fftw(
      bw_, bw_out_, bw_ - 1, sig_coef_[0], sig_coef_[1], pat_coef_[0],
      pat_coef_[1], so3_coef_);
}

void SphericalCorrelation::inverseTransform() {
  CHECK_NOTNULL(so3_sig_);
  CHECK_NOTNULL(so3_coef_);
  CHECK_NOTNULL(workspace1_);
  CHECK_NOTNULL(workspace2_);
  CHECK_NOTNULL(workspace3_);
  VLOG(2) << "Taking the inverse SO(3) transform with bandwidth " << bw_out_
          << " of the SO(3) correlated coefficients.";

  Inverse_SO3_Naive_fftw(
      bw_out_, so3_coef_, so3_sig_, workspace1_, workspace2_out_,
      workspace3_out_, &inverse_so3_, 1);

  so3_mag_sig_ = std::vector<double>(so3_bw_);
  for (uint32_t i = 0u; i < so3_bw_; ++i) {
    const double real = so3_sig_[i][0];
    const double imag = so3_sig_[i][1];
    so3_mag_sig_[i] = real * real + imag * imag;
  }
}

}  // namespace phaser_core
