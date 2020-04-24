#include "phaser/backend/alignment/spatial-correlation.h"

#include <glog/logging.h>

namespace alignment {

SpatialCorrelation::SpatialCorrelation(const uint32_t n_voxels)
    : total_n_voxels_(n_voxels * n_voxels * n_voxels),
      n_voxels_per_dim_(n_voxels) {
  const uint32_t n_fftw_size = sizeof(fftw_complex) * total_n_voxels_;
  F_ = static_cast<fftw_complex*>(fftw_malloc(n_fftw_size));
  G_ = static_cast<fftw_complex*>(fftw_malloc(n_fftw_size));
  C_ = static_cast<fftw_complex*>(fftw_malloc(n_fftw_size));

  c_ = new double[total_n_voxels_];
  f_ = new double[total_n_voxels_];
  g_ = new double[total_n_voxels_];

  // Create the FFTW plans for two FFTs and one IFFT.
  f_plan_ =
      fftw_plan_dft_r2c_3d(n_voxels, n_voxels, n_voxels, f_, F_, FFTW_ESTIMATE);

  g_plan_ =
      fftw_plan_dft_r2c_3d(n_voxels, n_voxels, n_voxels, g_, G_, FFTW_ESTIMATE);

  c_plan_ =
      fftw_plan_dft_c2r_3d(n_voxels, n_voxels, n_voxels, C_, c_, FFTW_ESTIMATE);
}

SpatialCorrelation::~SpatialCorrelation() {
  fftw_free(F_);
  fftw_free(G_);
  fftw_free(C_);

  fftw_destroy_plan(f_plan_);
  fftw_destroy_plan(g_plan_);
  fftw_destroy_plan(c_plan_);

  fftw_cleanup();
  delete[] c_;
  delete[] f_;
  delete[] g_;
}

void SpatialCorrelation::complexMulSeq(
    fftw_complex* F, fftw_complex* G, fftw_complex* C) {
  for (uint32_t i = 0u; i < total_n_voxels_; ++i) {
    C[i][0] = F[i][0] * G[i][0] - F[i][1] * (-G[i][1]);
    C[i][1] = F[i][0] * (-G[i][1]) + F[i][1] * G[i][0];
  }
}

double* SpatialCorrelation::correlateSignals(double* const f, double* const g) {
  const uint32_t function_size = total_n_voxels_ * sizeof(double);
  memcpy(f_, f, function_size);
  memcpy(g_, g, function_size);

  // Perform the two FFTs on the discretized signals.
  VLOG(1) << "Performing FFT on the first point cloud.";
  fftw_execute(f_plan_);
  VLOG(1) << "Performing FFT on the second point cloud.";
  fftw_execute(g_plan_);

  // Correlate the signals in the frequency domain.
  complexMulSeq(F_, G_, C_);

  // Perform the IFFT on the correlation tensor.
  VLOG(1) << "Performing IFFT on correlation.";
  fftw_execute(c_plan_);
  return c_;
}

}  // namespace alignment
