#include "phaser/backend/correlation/spherical-correlation-laplace.h"

#include <fstream>
#include <glog/logging.h>

#include "phaser/common/signal-utils.h"

namespace phaser_core {

SphericalCorrelationLaplace::SphericalCorrelationLaplace(
    const uint32_t bw, const uint32_t zero_padding)
    : SphericalCorrelation(bw, zero_padding) {}

void SphericalCorrelationLaplace::correlateSampledSignals(
    const std::vector<SampledSignal>& f, const std::vector<SampledSignal>& g) {
  CHECK_EQ(f.size(), g.size());
  CHECK_GT(f.size(), 1u);
  VLOG(1) << "--- Spherical laplace correlation [" << bw_ << " bw] -----";
  std::vector<fftw_complex*> f_channels, g_channels;
  extractTransformedChannels(f, g, &f_channels, &g_channels);

  const uint32_t full_bw = bw_ * bw_;
  std::vector<complex_t> F_fused =
      laplace_.fuseChannels(f_channels, full_bw, 5);
  std::vector<complex_t> G_fused =
      laplace_.fuseChannels(g_channels, full_bw, 5);

  VLOG(3) << "Setting the fused values for the original input.";
  setFusedCoefficients(F_fused, G_fused, full_bw);
  VLOG(3) << "Shifting back the signals.";
  inverseShiftSignals(full_bw);

  VLOG(3) << "Correlating and inverse transform.";
  correlate();
  inverseTransform();

  VLOG(3) << "Deleting the created values.";
  freeChannels(&f_channels);
  freeChannels(&g_channels);
}

void SphericalCorrelationLaplace::extractTransformedChannels(
    const std::vector<SampledSignal>& fs, const std::vector<SampledSignal>& gs,
    std::vector<fftw_complex*>* f_channels,
    std::vector<fftw_complex*>* g_channels) {
  CHECK_NOTNULL(f_channels);
  CHECK_NOTNULL(g_channels);
  const uint32_t n_channels = fs.size();
  CHECK_EQ(n_channels, gs.size());
  CHECK_GT(n_channels, 0u);

  const uint32_t full_bw = bw_ * bw_;
  for (uint32_t i = 0u; i < n_channels; ++i) {
    VLOG(2) << "Performing SFT (" << i + 1 << "/" << n_channels << ")";
    auto FiGi = performFFTandShift(fs[i], gs[i], full_bw);
    f_channels->emplace_back(FiGi.first);
    g_channels->emplace_back(FiGi.second);
  }
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
    const std::vector<complex_t>& signal, const std::vector<complex_t>& pattern,
    const uint32_t n_coeffs) {
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

void SphericalCorrelationLaplace::freeChannels(
    std::vector<fftw_complex*>* channels) {
  CHECK_NOTNULL(channels);
  for (fftw_complex* channel : *channels) {
    CHECK_NOTNULL(channel);
    fftw_free(channel);
  }
}

}  // namespace phaser_core
