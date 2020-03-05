#ifndef PACKLO_BACKEND_ALIGNMENT_SPATIAL_CORRELATION_CUDA_H_
#define PACKLO_BACKEND_ALIGNMENT_SPATIAL_CORRELATION_CUDA_H_

#include <cufft.h>
#include <cuda.h>
#include <cuda_runtime.h>

#include <array>
#include <vector>

namespace alignment {

class SpatialCorrelationCuda {
 public:
  SpatialCorrelationCuda(const uint32_t voxels_per_dim);
  ~SpatialCorrelationCuda();

  void correlateSignals(double* const f, double* const g);

 private:
  cufftHandle f_plan_;
  cufftHandle c_plan_;
  cufftDoubleComplex* F_;
  cufftDoubleComplex* G_;
  double* c_;
  const uint32_t n_voxels_total_;
  const uint32_t n_voxels_per_dim_;
};

}  // namespace alignment

#endif  // PACKLO_BACKEND_ALIGNMENT_SPATIAL_CORRELATION_CUDA_H_
