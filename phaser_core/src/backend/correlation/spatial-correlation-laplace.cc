#include "phaser/backend/correlation/spatial-correlation-laplace.h"

#include <glog/logging.h>

#include "phaser/common/signal-utils.h"

namespace phaser_core {

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
  std::vector<fftw_complex*> f_channels;
  std::vector<fftw_complex*> g_channels;
  extractTransformedChannels(f, g, &f_channels, &g_channels);

  std::vector<complex_t> F_fused =
      laplace_.fuseChannels(f_channels, total_n_voxels_, 5);
  std::vector<complex_t> G_fused =
      laplace_.fuseChannels(g_channels, total_n_voxels_, 5);

  complexMulSeq(
      reinterpret_cast<fftw_complex*>(F_fused.data()),
      reinterpret_cast<fftw_complex*>(G_fused.data()), C_);

  // Perform the IFFT on the correlation tensor.
  VLOG(1) << "Shifting back the signals. Performing IFFT on low passed filtered"
             " correlation.";
  common::SignalUtils::IFFTShift(C_, total_n_voxels_);
  fftw_execute(c_plan_);

  if (zero_padding_ != 0u) {
    for (uint32_t i = 0u; i < total_n_voxels_padded_; ++i) {
      C_[i][0] = 0.0;
      C_[i][1] = 0.0;
    }
  }

  // Free the used memory for the fusion.
  freeChannels(&f_channels);
  freeChannels(&g_channels);

  return c_;
}

void SpatialCorrelationLaplace::extractTransformedChannels(
    const std::vector<Eigen::VectorXd*>& fs,
    const std::vector<Eigen::VectorXd*>& gs,
    std::vector<fftw_complex*>* f_channels,
    std::vector<fftw_complex*>* g_channels) {
  CHECK_NOTNULL(f_channels);
  CHECK_NOTNULL(g_channels);
  const uint32_t n_channels = fs.size();
  // const uint32_t n_channels = 2;
  CHECK_EQ(n_channels, gs.size());
  CHECK_GT(n_channels, 0u);

  const uint32_t function_size = total_n_voxels_ * sizeof(double);
  for (uint32_t i = 0u; i < n_channels; ++i) {
    // Perform 3D FFT for channel i.
    CHECK_NOTNULL(fs[i]);
    CHECK_NOTNULL(gs[i]);
    memcpy(f_, fs[i]->data(), function_size);
    memcpy(g_, gs[i]->data(), function_size);
    performFFTandShift();

    // Copy the memory and store in the channels vector.
    fftw_complex* F = static_cast<fftw_complex*>(fftw_malloc(n_fftw_size_));
    fftw_complex* G = static_cast<fftw_complex*>(fftw_malloc(n_fftw_size_));
    memcpy(F, F_, n_fftw_size_);
    memcpy(G, G_, n_fftw_size_);
    f_channels->emplace_back(F);
    g_channels->emplace_back(G);
  }
}

void SpatialCorrelationLaplace::performFFTandShift() {
  fftw_execute(f_plan_);
  fftw_execute(g_plan_);
  common::SignalUtils::FFTShift(F_, total_n_voxels_);
  common::SignalUtils::FFTShift(G_, total_n_voxels_);
}

void SpatialCorrelationLaplace::freeChannels(
    std::vector<fftw_complex*>* channels) {
  CHECK_NOTNULL(channels);
  for (fftw_complex* channel : *channels) {
    CHECK_NOTNULL(channel);
    fftw_free(channel);
  }
}

}  // namespace phaser_core
