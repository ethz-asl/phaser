#include "phaser/backend/correlation/spatial-correlation-low-pass.h"

#include <glog/logging.h>

namespace correlation {

SpatialCorrelationLowPass::SpatialCorrelationLowPass(const uint32_t n_voxels)
    : SpatialCorrelation(n_voxels) {}

void SpatialCorrelationLowPass::complexMulSeq(
    fftw_complex* F, fftw_complex* G, fftw_complex* C) {
  for (uint32_t i = 0u; i < total_n_voxels_; ++i) {
    C[i][0] = F[i][0] * G[i][0] - F[i][1] * (-G[i][1]);
    C[i][1] = F[i][0] * (-G[i][1]) + F[i][1] * G[i][0];
  }
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

  // Correlate the signals in the frequency domain.
  complexMulVec(F_, G_, C_);

  // Perform the IFFT on the correlation tensor.
  VLOG(1) << "Performing IFFT on correlation.";
  fftw_execute(c_plan_);
  return c_;
}

}  // namespace correlation
