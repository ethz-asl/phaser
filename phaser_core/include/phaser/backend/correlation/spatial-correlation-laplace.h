#ifndef PHASER_BACKEND_CORRELATION_SPATIAL_CORRELATION_LAPLACE_H_
#define PHASER_BACKEND_CORRELATION_SPATIAL_CORRELATION_LAPLACE_H_

#include <vector>

#include "phaser/backend/correlation/spatial-correlation.h"
#include "phaser/backend/fusion/laplace-pyramid.h"

namespace phaser_core {

class SpatialCorrelationLaplace : public SpatialCorrelation {
 public:
  explicit SpatialCorrelationLaplace(
      const uint32_t n_voxels, const uint32_t zero_padding);

  virtual ~SpatialCorrelationLaplace() = default;
  double* correlateSignals(
      const std::vector<Eigen::VectorXd*>& f,
      const std::vector<Eigen::VectorXd*>& g) override;

 private:
  void extractTransformedChannels(
      const std::vector<Eigen::VectorXd*>& fs,
      const std::vector<Eigen::VectorXd*>& gs,
      std::vector<fftw_complex*>* f_channels,
      std::vector<fftw_complex*>* g_channels);
  void performFFTandShift();
  void freeChannels(std::vector<fftw_complex*>* channels);

  LaplacePyramid laplace_;
  const uint32_t n_fftw_size_;
  fftw_complex *F_intensities_, *G_intensities_;
};

}  // namespace phaser_core

#endif  // PHASER_BACKEND_CORRELATION_SPATIAL_CORRELATION_LAPLACE_H_
