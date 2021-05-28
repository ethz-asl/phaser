#include "phaser/backend/correlation/spherical-correlation-low-pass.h"

#include <glog/logging.h>

#include "phaser/common/core-gflags.h"
#include "phaser/common/signal-utils.h"

namespace phaser_core {

SphericalCorrelationLowPass::SphericalCorrelationLowPass(
    const uint32_t bw, const uint32_t zero_padding)
    : SphericalCorrelation(bw, zero_padding) {}

void SphericalCorrelationLowPass::correlateSampledSignals(
    const std::vector<SampledSignal>& f1,
    const std::vector<SampledSignal>& f2) {
  CHECK_EQ(f1.size(), f2.size());
  CHECK_GT(f1.size(), 0u);
  VLOG(1) << "Starting the low pass correlation with a " << bw_ << " bandwidth";
  performSphericalTransforms(f1[0], f2[0]);
  filterAndCorrelateCoefficients();
  inverseTransform();
}

void SphericalCorrelationLowPass::filterAndCorrelateCoefficients() {
  CHECK_GE(FLAGS_phaser_core_spherical_low_pass_lower_bound, 0);
  CHECK_GT(FLAGS_phaser_core_spherical_low_pass_upper_bound, 0);
  CHECK_NOTNULL(sig_coef_[0]);
  CHECK_NOTNULL(sig_coef_[1]);
  CHECK_NOTNULL(pat_coef_[0]);
  CHECK_NOTNULL(pat_coef_[1]);

  const uint32_t full_bw = bw_ * bw_;
  const uint32_t lower_bound = std::min(
      full_bw,
      static_cast<uint32_t>(FLAGS_phaser_core_spherical_low_pass_lower_bound));
  const uint32_t upper_bound = std::min(
      full_bw,
      static_cast<uint32_t>(FLAGS_phaser_core_spherical_low_pass_upper_bound));
  shiftSignals(full_bw);
  for (uint32_t i = 0u; i < lower_bound; ++i) {
    sig_coef_[0][i] = 0.0;
    sig_coef_[1][i] = 0.0;
    pat_coef_[0][i] = 0.0;
    pat_coef_[1][i] = 0.0;
  }
  VLOG(3) << "finished lower third.";
  for (uint32_t i = upper_bound; i < full_bw; ++i) {
    sig_coef_[0][i] = 0.0;
    sig_coef_[1][i] = 0.0;
    pat_coef_[0][i] = 0.0;
    pat_coef_[1][i] = 0.0;
  }
  VLOG(2) << "Left "
          << (upper_bound - lower_bound) / static_cast<float>(full_bw) * 100.0
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
}

}  // namespace phaser_core
