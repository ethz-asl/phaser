#ifndef PHASER_BACKEND_CORRELATION_SPATIAL_CORRELATION_LOW_PASS_H_
#define PHASER_BACKEND_CORRELATION_SPATIAL_CORRELATION_LOW_PASS_H_

#include <fftw3/fftw3.h>

#include "phaser/backend/correlation/spatial-correlation.h"

namespace correlation {

class SpatialCorrelationLowPass : public SpatialCorrelation {
 public:
  explicit SpatialCorrelationLowPass(const uint32_t n_voxels);
  virtual ~SpatialCorrelationLowPass() = default;
  double* correlateSignals(double* const f, double* const g) override;

 private:
  void complexMulSeq(fftw_complex* F, fftw_complex* G, fftw_complex* C);
};

}  // namespace correlation

#endif  // PHASER_BACKEND_CORRELATION_SPATIAL_CORRELATION_LOW_PASS_H_
