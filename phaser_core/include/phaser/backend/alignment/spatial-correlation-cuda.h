#ifndef PACKLO_BACKEND_ALIGNMENT_PHASE_ALIGNER_GPU_H_
#define PACKLO_BACKEND_ALIGNMENT_PHASE_ALIGNER_GPU_H_

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

  /*
  void alignRegistered(
      const model::PointCloud& cloud_prev,
      const std::vector<model::FunctionValue>& f_prev,
      const model::PointCloud& cloud_reg,
      const std::vector<model::FunctionValue>& f_reg,
      common::Vector_t* xyz) override;
      */
  void correlateSignals(double* const f, double* const g);

 private:
  cufftHandle f_plan_;
  cufftHandle c_plan_;
  cufftDoubleComplex *F_, *G_, *C_;
  double* c_;
  //Eigen::VectorXd f_;
  //Eigen::VectorXd g_;
  const uint32_t n_voxels_total_;
  const uint32_t n_voxels_per_dim_;
};

}  // namespace alignment

#endif  // PACKLO_BACKEND_ALIGNMENT_PHASE_ALIGNER_GPU_H_
