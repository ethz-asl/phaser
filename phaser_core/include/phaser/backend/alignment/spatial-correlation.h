#ifndef PACKLO_BACKEND_ALIGNMENT_SPATIAL_CORRELATION_H_
#define PACKLO_BACKEND_ALIGNMENT_SPATIAL_CORRELATION_H_

#include "phaser/backend/alignment/base-spatial-correlation.h"
#include <fftw3/fftw3.h>

#include <array>
#include <vector>

namespace alignment {

class SpatialCorrelation : public BaseSpatialCorrelation {
public:
  SpatialCorrelation(const uint32_t n_voxels);
  ~SpatialCorrelation();
  double* correlateSignals(double* f, double* g) override;

private:
  fftw_plan f_plan_;
  fftw_plan g_plan_;
  fftw_plan c_plan_;
  fftw_complex *F_, *G_, *C_;
  double* c_;
  double* f_;
  double* g_;
  Eigen::VectorXd hist_;
  const uint32_t total_n_voxels_;
  const uint32_t n_voxels_;
};

}  // namespace alignment

#endif  // PACKLO_BACKEND_ALIGNMENT_SPATIAL_CORRELATION_H_
