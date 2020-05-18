#include "phaser/backend/correlation/spherical-correlation-low-pass.h"

#include <glog/logging.h>

#include "phaser/common/signal-utils.h"

namespace correlation {

SphericalCorrelationLowPass::SphericalCorrelationLowPass(const uint32_t bw)
    : SphericalCorrelation(bw) {}

void SphericalCorrelationLowPass::correlateSampledSignals(
    const std::vector<double>& f1, const std::vector<double>& f2) {
  VLOG(1) << "Starting the low pass correlation with a " << bw_ << " bandwidth";
  performSphericalTransforms(f1, f2);
  filterAndCorrelateCoefficients();
  inverseTransform();
}

void SphericalCorrelationLowPass::filterAndCorrelateCoefficients() {
  CHECK_NOTNULL(sig_coef_[0]);
  CHECK_NOTNULL(sig_coef_[1]);
  CHECK_NOTNULL(pat_coef_[0]);
  CHECK_NOTNULL(pat_coef_[1]);

  const uint32_t full_bw = bw_ * bw_;
  const uint32_t third_bw = std::round(full_bw / 4.0);
  const uint32_t upper_third_bw = full_bw - third_bw;
  shiftSignals(full_bw);
  VLOG(1) << "starting lower third: " << 0 << " to " << third_bw;
  for (uint32_t i = 0u; i < third_bw; ++i) {
    sig_coef_[0][i] = 0.0;
    sig_coef_[1][i] = 0.0;
    pat_coef_[0][i] = 0.0;
    pat_coef_[1][i] = 0.0;
  }
  VLOG(1) << "finished lower third.";
  for (uint32_t i = upper_third_bw; i < full_bw; ++i) {
    sig_coef_[0][i] = 0.0;
    sig_coef_[1][i] = 0.0;
    pat_coef_[0][i] = 0.0;
    pat_coef_[1][i] = 0.0;
  }
  VLOG(1) << "Left "
          << (upper_third_bw - third_bw) / static_cast<float>(full_bw) * 100.0
          << "% in the spectrum.";
  inverseShiftSignals(full_bw);
  correlate();
}

void SphericalCorrelationLowPass::shiftSignals(const uint32_t n_points) {
  common::SignalUtils::FFTShift(sig_coef_[0], n_points);
  common::SignalUtils::FFTShift(sig_coef_[1], n_points);
  common::SignalUtils::FFTShift(pat_coef_[0], n_points);
  common::SignalUtils::FFTShift(pat_coef_[1], n_points);
}

void SphericalCorrelationLowPass::inverseShiftSignals(const uint32_t n_points) {
  common::SignalUtils::IFFTShift(sig_coef_[0], n_points);
  common::SignalUtils::IFFTShift(sig_coef_[1], n_points);
  common::SignalUtils::IFFTShift(pat_coef_[0], n_points);
  common::SignalUtils::IFFTShift(pat_coef_[1], n_points);
  /*
  common::SignalUtils::IFFTShift(so3_coef_[0], n_points);
  common::SignalUtils::IFFTShift(so3_coef_[1], n_points);
  */
}

}  // namespace correlation
