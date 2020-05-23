#include "phaser/backend/correlation/spherical-correlation-laplace.h"

#include <glog/logging.h>

#include "phaser/common/signal-utils.h"

namespace correlation {

SphericalCorrelationLaplace::SphericalCorrelationLaplace(const uint32_t bw)
    : SphericalCorrelation(bw) {}

void SphericalCorrelationLaplace::correlateSampledSignals(
    const std::vector<SampledSignal>& f, const std::vector<SampledSignal>& g) {
  CHECK_EQ(f.size(), g.size());
  CHECK_GT(f.size(), 1u);
  VLOG(1) << "--- Spherical laplace correlation [" << bw_ << " bw] -----";
  const uint32_t full_bw = bw_ * bw_;
  VLOG(2) << "Performing SFT (1/2)";
  auto F1G1 = performFFTandShift(f[0], g[0], full_bw);
  VLOG(2) << "Performing SFT (2/2)";
  // auto F2G2 = performFFTandShift(f[1], g[1], full_bw);

  VLOG(2) << "Fusing channels together.";
  std::vector<fftw_complex*> channels = {F1G1.first};
  std::vector<fusion::complex_t> F_fused =
      laplace_.fuseChannels(channels, full_bw, 2);
  /*
channels = {F1G1.second};
std::vector<fusion::complex_t> G_fused =
  laplace_.fuseChannels(channels, full_bw, 3);
  */

  VLOG(2) << "Setting the fused values for the original input.";
  // setFusedCoefficients(F_fused, G_fused, full_bw);
  for (uint32_t i = 0; i < full_bw; ++i) {
    sig_coef_[0][i] = F_fused[i][0];
    sig_coef_[1][i] = F_fused[i][1];
  }
  VLOG(2) << "Shifting back the signals.";
  inverseShiftSignals(full_bw);

  VLOG(2) << "Correlating and inverse transform.";
  correlate();
  inverseTransform();

  VLOG(2) << "Deleting the created values.";
  delete[] F1G1.first;
  delete[] F1G1.second;
  // delete[] F2G2.first;
  // delete[] F2G2.second;
}

std::pair<fftw_complex*, fftw_complex*>
SphericalCorrelationLaplace::performFFTandShift(
    const SampledSignal& f1, const SampledSignal& f2, const uint32_t n_coeffs) {
  performSphericalTransforms(f1, f2);
  shiftSignals(n_coeffs);
  return std::make_pair(
      convertCoeffArrays(sig_coef_[0], sig_coef_[1], n_coeffs),
      convertCoeffArrays(pat_coef_[0], pat_coef_[1], n_coeffs));
}

fftw_complex* SphericalCorrelationLaplace::convertCoeffArrays(
    double* f_real, double* f_imag, const uint32_t n_coeffs) {
  fftw_complex* coeffs = new fftw_complex[n_coeffs];
  for (uint32_t i = 0u; i < n_coeffs; ++i) {
    coeffs[i][0] = f_real[i];
    coeffs[i][1] = f_imag[i];
  }
  return coeffs;
}

void SphericalCorrelationLaplace::setFusedCoefficients(
    const std::vector<fusion::complex_t>& signal,
    const std::vector<fusion::complex_t>& pattern, const uint32_t n_coeffs) {
  for (uint32_t i = 0; i < n_coeffs; ++i) {
    sig_coef_[0][i] = signal[i][0];
    sig_coef_[1][i] = signal[i][1];
    pat_coef_[0][i] = pattern[i][0];
    pat_coef_[1][i] = pattern[i][1];
  }
}
void SphericalCorrelationLaplace::setFusedCoefficients(
    fftw_complex* signal, fftw_complex* pattern, const uint32_t n_coeffs) {
  for (uint32_t i = 0; i < n_coeffs; ++i) {
    sig_coef_[0][i] = signal[i][0];
    sig_coef_[1][i] = signal[i][1];
    pat_coef_[0][i] = pattern[i][0];
    pat_coef_[1][i] = pattern[i][1];
  }
}

void SphericalCorrelationLaplace::shiftSignals(const uint32_t n_points) {
  common::SignalUtils::FFTShift(sig_coef_[0], n_points);
  common::SignalUtils::FFTShift(sig_coef_[1], n_points);
  common::SignalUtils::FFTShift(pat_coef_[0], n_points);
  common::SignalUtils::FFTShift(pat_coef_[1], n_points);
}

void SphericalCorrelationLaplace::inverseShiftSignals(const uint32_t n_points) {
  common::SignalUtils::IFFTShift(sig_coef_[0], n_points);
  common::SignalUtils::IFFTShift(sig_coef_[1], n_points);
  common::SignalUtils::IFFTShift(pat_coef_[0], n_points);
  common::SignalUtils::IFFTShift(pat_coef_[1], n_points);
}

}  // namespace correlation
