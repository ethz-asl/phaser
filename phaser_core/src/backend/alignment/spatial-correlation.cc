#include "phaser/backend/alignment/spatial-correlation.h"

#include <glog/logging.h>

namespace alignment {

SpatialCorrelation::SpatialCorrelation(const uint32_t n_voxels)
    : total_n_voxels_(n_voxels * n_voxels * n_voxels), n_voxels_(n_voxels) {

  const uint32_t n_fftw_size = sizeof(fftw_complex) * total_n_voxels_;
  F_ = static_cast<fftw_complex*>(fftw_malloc(n_fftw_size));
  G_ = static_cast<fftw_complex*>(fftw_malloc(n_fftw_size));
  C_ = static_cast<fftw_complex*>(fftw_malloc(n_fftw_size));

  c_ = new double[total_n_voxels_];
  f_ = new double[total_n_voxels_];
  g_ = new double[total_n_voxels_];

  // Create the FFTW plans for two FFTs and one IFFT.
  f_plan_ = fftw_plan_dft_r2c_3d(n_voxels, n_voxels, n_voxels,
    f_, F_, FFTW_ESTIMATE);

  g_plan_ = fftw_plan_dft_r2c_3d(n_voxels, n_voxels, n_voxels,
    g_, G_, FFTW_ESTIMATE);

  c_plan_ = fftw_plan_dft_c2r_3d(n_voxels, n_voxels, n_voxels,
    C_, c_, FFTW_ESTIMATE);
}

SpatialCorrelation::~SpatialCorrelation() {
  fftw_free(F_);
  fftw_free(G_);
  fftw_free(C_);

  fftw_destroy_plan(f_plan_);
  fftw_destroy_plan(g_plan_);
  fftw_destroy_plan(c_plan_);

  fftw_cleanup();
  delete [] c_;
}

double* SpatialCorrelation::correlateSignals(
    double* f, double* g) {
  const auto max_f = std::max_element(f, f+total_n_voxels_);
  VLOG(1) << "max f: " << *max_f << " dist: " << std::distance(f, max_f);

  const uint32_t function_size = total_n_voxels_ * sizeof(double);
  memcpy(f_, f, function_size);
  memcpy(g_, g, function_size);
  //*f_.data() = *f;
  //*g_.data() = *g;
  const auto max_f2 = std::max_element(f_, f_+total_n_voxels_);
  VLOG(1) << "max f2: " << *max_f2 << " dist: " << std::distance(f_, max_f2);

  // Perform the two FFTs on the discretized signals.
  VLOG(1) << "Performing FFT on the first point cloud.";
  fftw_execute(f_plan_);
  VLOG(1) << "Performing FFT on the second point cloud.";
  fftw_execute(g_plan_);

  // Correlate the signals in the frequency domain.
  for (uint32_t i = 0u; i < n_voxels_; ++i) {
    C_[i][0] = F_[i][0] * G_[i][0] - F_[i][1] * (-G_[i][1]);
    C_[i][1] = F_[i][0] * (-G_[i][1]) + F_[i][1] * G_[i][0];
  }

  // Perform the IFFT on the correlation tensor.
  VLOG(1) << "Performing IFFT on correlation.";
  fftw_execute(c_plan_);

  const auto max_corr = std::max_element(c_, c_+total_n_voxels_);
  const uint32_t max = std::distance(c_, max_corr);
  VLOG(1) << "Found max correlation at " << max;
  return c_;
}

}  // namespace alignment
