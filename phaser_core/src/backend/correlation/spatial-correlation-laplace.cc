#include "phaser/backend/correlation/spatial-correlation-laplace.h"

#include <glog/logging.h>

#include "phaser/common/signal-utils.h"

namespace correlation {

SpatialCorrelationLaplace::SpatialCorrelationLaplace(const uint32_t n_voxels)
    : SpatialCorrelation(n_voxels) {}

double* SpatialCorrelationLaplace::correlateSignals(
    const std::vector<Eigen::VectorXd*>& f,
    const std::vector<Eigen::VectorXd*>& g) {
  CHECK_GT(f.size(), 0u);
  CHECK_GT(g.size(), 0u);
  const uint32_t function_size = total_n_voxels_ * sizeof(double);
  memcpy(f_, f[0]->data(), function_size);
  memcpy(g_, g[0]->data(), function_size);

  // Perform the two FFTs on the discretized signals.
  VLOG(1) << "Performing FFT on the first point cloud.";
  fftw_execute(f_plan_);
  VLOG(1) << "Performing FFT on the second point cloud.";
  fftw_execute(g_plan_);

  common::SignalUtils::FFTShift(F_, total_n_voxels_);
  common::SignalUtils::FFTShift(G_, total_n_voxels_);

  complexMulSeq(F_, G_, C_);

  // Perform the IFFT on the correlation tensor.
  VLOG(1) << "Shifting back the signals. Performing IFFT on low passed filtered"
             " correlation.";
  common::SignalUtils::IFFTShift(C_, total_n_voxels_);
  fftw_execute(c_plan_);
  return c_;
}

}  // namespace correlation
