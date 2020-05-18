#include "phaser/backend/correlation/spatial-correlation-low-pass.h"

#include <algorithm>

#include <glog/logging.h>

#include "phaser/common/signal-utils.h"

DEFINE_int32(
    phaser_core_spatial_low_pass_lower_bound, 0,
    "Defines the lower frequency bound of the spatial low pass filtering.");

DEFINE_int32(
    phaser_core_spatial_low_pass_upper_bound, 100000,
    "Defines the lower frequency bound of the spatial low pass filtering.");

namespace correlation {

SpatialCorrelationLowPass::SpatialCorrelationLowPass(const uint32_t n_voxels)
    : SpatialCorrelation(n_voxels),
      low_pass_lower_bound_(FLAGS_phaser_core_spatial_low_pass_lower_bound),
      low_pass_upper_bound_(std::min(
          static_cast<uint32_t>(FLAGS_phaser_core_spatial_low_pass_upper_bound),
          n_voxels)) {
  computeIndicesBasedOnBounds();
}

SpatialCorrelationLowPass::SpatialCorrelationLowPass(
    const uint32_t n_voxels, const uint32_t lower_bound,
    const uint32_t upper_bound)
    : SpatialCorrelation(n_voxels),
      low_pass_lower_bound_(lower_bound),
      low_pass_upper_bound_(upper_bound) {
  computeIndicesBasedOnBounds();
}

void SpatialCorrelationLowPass::shiftSignals(fftw_complex* F, fftw_complex* G) {
  common::SignalUtils::FFTShift(F, total_n_voxels_);
  common::SignalUtils::FFTShift(G, total_n_voxels_);
}

void SpatialCorrelationLowPass::inverseShiftSignals(fftw_complex* C) {
  common::SignalUtils::IFFTShift(C, total_n_voxels_);
}

double* SpatialCorrelationLowPass::correlateSignals(
    double* const f, double* const g) {
  const uint32_t function_size = total_n_voxels_ * sizeof(double);
  memcpy(f_, f, function_size);
  memcpy(g_, g, function_size);

  // Perform the two FFTs on the discretized signals.
  VLOG(1) << "Performing FFT on the first point cloud.";
  fftw_execute(f_plan_);
  VLOG(1) << "Performing FFT on the second point cloud.";
  fftw_execute(g_plan_);

  // Low pass filtering of the signals.
  VLOG(1)
      << "Shifting the low frequency components to the center of the spectrum.";
  shiftSignals(F_, G_);

  // TODO(lbern): Fill C_ with zeros.

  // Correlate the signals in the frequency domain.
  complexMulVecUsingIndices(linear_indices_, F_, G_, C_);

  // Perform the IFFT on the correlation tensor.
  VLOG(1) << "Shifting back the signals. Performing IFFT on low passed filtered"
             " correlation.";
  inverseShiftSignals(C_);
  fftw_execute(c_plan_);
  return c_;
}

uint32_t SpatialCorrelationLowPass::getLowerBound() const noexcept {
  return low_pass_lower_bound_;
}

uint32_t SpatialCorrelationLowPass::getUpperBound() const noexcept {
  return low_pass_upper_bound_;
}

void SpatialCorrelationLowPass::computeIndicesBasedOnBounds() {
  linear_indices_.clear();
  for (uint32_t i = low_pass_lower_bound_; i <= low_pass_upper_bound_; ++i) {
    for (uint32_t j = low_pass_lower_bound_; j <= low_pass_upper_bound_; ++j) {
      for (uint32_t k = low_pass_lower_bound_; k <= low_pass_upper_bound_;
           ++k) {
        linear_indices_.emplace_back(common::SignalUtils::Sub2Ind(
            i, j, k, n_voxels_per_dim_, n_voxels_per_dim_));
      }
    }
  }
  VLOG(1) << "Computed indicies for bounds: " << low_pass_lower_bound_
          << " and " << low_pass_upper_bound_ << ".";
}

uint32_t SpatialCorrelationLowPass::getNumberOfIndices() const noexcept {
  return linear_indices_.size();
}

}  // namespace correlation
