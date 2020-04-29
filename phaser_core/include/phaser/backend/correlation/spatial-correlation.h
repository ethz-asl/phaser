#ifndef PHASER_BACKEND_CORRELATION_SPATIAL_CORRELATION_H_
#define PHASER_BACKEND_CORRELATION_SPATIAL_CORRELATION_H_

#include <fftw3/fftw3.h>

#include "phaser/backend/correlation/base-spatial-correlation.h"

namespace alignment {

class SpatialCorrelation : public BaseSpatialCorrelation {
 public:
  explicit SpatialCorrelation(const uint32_t n_voxels);
  ~SpatialCorrelation();
  double* correlateSignals(double* const f, double* const g) override;

 private:
  void complexMulSeq(fftw_complex* F, fftw_complex* G, fftw_complex* C);
  void complexMulVec(fftw_complex* F, fftw_complex* G, fftw_complex* C);

  fftw_plan f_plan_;
  fftw_plan g_plan_;
  fftw_plan c_plan_;
  fftw_complex *F_, *G_, *C_;
  double* c_;
  double* f_;
  double* g_;
  Eigen::VectorXd hist_;
  const uint32_t total_n_voxels_;
  const uint32_t n_voxels_per_dim_;
};

}  // namespace alignment

#endif  // PHASER_BACKEND_CORRELATION_SPATIAL_CORRELATION_H_
