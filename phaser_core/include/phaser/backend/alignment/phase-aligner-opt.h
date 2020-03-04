#ifndef PACKLO_BACKEND_ALIGNMENT_PHASE_ALIGNER_GPU_H_
#define PACKLO_BACKEND_ALIGNMENT_PHASE_ALIGNER_GPU_H_

#include <cufft.h>
#include <cuda.h>
#include <cuda_runtime.h>

#include <array>
#include <vector>

namespace alignment {

class PhaseAlignerGpu {
 public:
  PhaseAlignerGpu();
  ~PhaseAlignerGpu();

  /*
  void alignRegistered(
      const model::PointCloud& cloud_prev,
      const std::vector<model::FunctionValue>& f_prev,
      const model::PointCloud& cloud_reg,
      const std::vector<model::FunctionValue>& f_reg,
      common::Vector_t* xyz) override;
      */

  std::vector<double> getCorrelation() const;

 private:
  cufftHandle f_plan_;
  //cufftHandle g_plan_;
  cufftHandle c_plan_;
  cufftDoubleComplex *F_, *G_, *C_;
  double* c_;
  //Eigen::VectorXd f_;
  //Eigen::VectorXd g_;
  const uint32_t n_voxels_;
};

}  // namespace alignment

#endif  // PACKLO_BACKEND_ALIGNMENT_PHASE_ALIGNER_GPU_H_
