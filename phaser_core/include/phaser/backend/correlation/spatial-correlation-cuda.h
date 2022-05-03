#ifndef PHASER_BACKEND_CORRELATION_SPATIAL_CORRELATION_CUDA_H_
#define PHASER_BACKEND_CORRELATION_SPATIAL_CORRELATION_CUDA_H_

#include <array>
#include <cuda.h>
#include <cuda_runtime.h>
#include <cufft.h>
#include <vector>

#include "phaser/backend/correlation/base-spatial-correlation.h"

namespace phaser_core {

class SpatialCorrelationCuda : public BaseSpatialCorrelation {
 public:
  explicit SpatialCorrelationCuda(const uint32_t voxels_per_dim);
  ~SpatialCorrelationCuda();

  double* correlateSignals(
      const std::vector<Eigen::VectorXd*>& f,
      const std::vector<Eigen::VectorXd*>& g) override;

 private:
  cufftHandle f_plan_;
  cufftHandle c_plan_;
  cufftDoubleComplex* F_;
  cufftDoubleComplex* G_;
  cufftDoubleComplex* C_;
  double* c_;
  const uint32_t n_voxels_total_;
  const uint32_t n_voxels_per_dim_;
};

}  // namespace phaser_core

#endif  // PHASER_BACKEND_CORRELATION_SPATIAL_CORRELATION_CUDA_H_
