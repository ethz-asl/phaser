#include "phaser/backend/correlation/spatial-correlation-laplace.h"

#include <glog/logging.h>

#include "phaser/common/signal-utils.h"

namespace correlation {

SpatialCorrelationLaplace::SpatialCorrelationLaplace(
    const uint32_t n_voxels, const uint32_t zero_padding)
    : SpatialCorrelation(n_voxels, zero_padding),
      n_fftw_size_(sizeof(fftw_complex) * total_n_voxels_) {
  F_intensities_ = static_cast<fftw_complex*>(fftw_malloc(n_fftw_size_));
  G_intensities_ = static_cast<fftw_complex*>(fftw_malloc(n_fftw_size_));
}

double* SpatialCorrelationLaplace::correlateSignals(
    const std::vector<Eigen::VectorXd*>& f,
    const std::vector<Eigen::VectorXd*>& g) {
  CHECK_GT(f.size(), 0u);
  CHECK_GT(g.size(), 0u);
  const uint32_t function_size = total_n_voxels_ * sizeof(double);
  memcpy(f_, f[0]->data(), function_size);
  memcpy(g_, g[0]->data(), function_size);
  performFFTandShift();

  memcpy(F_intensities_, F_, n_fftw_size_);
  memcpy(G_intensities_, G_, n_fftw_size_);
  memcpy(f_, f[1]->data(), function_size);
  memcpy(g_, g[1]->data(), function_size);
  performFFTandShift();

  std::vector<fftw_complex*> channels = {F_intensities_, F_};
  std::vector<fusion::complex_t> F_fused =
      laplace_.fuseChannels(channels, total_n_voxels_, 3);
  channels = {G_intensities_, G_};
  std::vector<fusion::complex_t> G_fused =
      laplace_.fuseChannels(channels, total_n_voxels_, 3);

  complexMulSeq(
      reinterpret_cast<fftw_complex*>(F_fused.data()),
      reinterpret_cast<fftw_complex*>(G_fused.data()), C_);

  // Perform the IFFT on the correlation tensor.
  VLOG(1) << "Shifting back the signals. Performing IFFT on low passed filtered"
             " correlation.";
  common::SignalUtils::IFFTShift(C_, total_n_voxels_);
  fftw_execute(c_plan_);
  return c_;
}
void SpatialCorrelationLaplace::performFFTandShift() {
  fftw_execute(f_plan_);
  fftw_execute(g_plan_);
  common::SignalUtils::FFTShift(F_, total_n_voxels_);
  common::SignalUtils::FFTShift(G_, total_n_voxels_);
}

}  // namespace correlation
