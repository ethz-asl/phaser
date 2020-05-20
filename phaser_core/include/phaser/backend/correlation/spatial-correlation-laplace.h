#ifndef PHASER_BACKEND_CORRELATION_SPATIAL_CORRELATION_LAPLACE_H_
#define PHASER_BACKEND_CORRELATION_SPATIAL_CORRELATION_LAPLACE_H_

#include <vector>

#include "phaser/backend/correlation/spatial-correlation.h"
#include "phaser/backend/fusion/laplace-pyramid.h"

namespace correlation {

class SpatialCorrelationLaplace : public SpatialCorrelation {
 public:
  explicit SpatialCorrelationLaplace(const uint32_t n_voxels);

  virtual ~SpatialCorrelationLaplace() = default;
  double* correlateSignals(
      const std::vector<Eigen::VectorXd*>& f,
      const std::vector<Eigen::VectorXd*>& g) override;

 private:
  void performFFTandShift();

  fusion::LaplacePyramid laplace_;
  const uint32_t n_fftw_size_;
  fftw_complex *F_intensities_, *G_intensities_;
};

}  // namespace correlation

#endif  // PHASER_BACKEND_CORRELATION_SPATIAL_CORRELATION_LAPLACE_H_
