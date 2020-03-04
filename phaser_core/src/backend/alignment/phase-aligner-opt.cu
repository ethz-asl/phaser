#include "phaser/backend/alignment/phase-aligner-opt.h"
#include <complex.h>
#include <cufft.h>

#include <gflags/gflags.h>
#include <glog/logging.h>

DEFINE_double(
    phase_gpu_discretize_lower, -50,
    "Specifies the lower bound for the discretization.");
DEFINE_double(
    phase_gpu_discretize_upper, 50,
    "Specifies the upper bound for the discretization.");
DEFINE_double(
    phase_gpu_n_voxels, 200,
    "Specifies the number of voxels for the discretization.");
DEFINE_double(
    phase_gpu_batch, 1,
    "");

namespace alignment {

PhaseAlignerGpu::PhaseAlignerGpu()
    : n_voxels_(
          FLAGS_phase_gpu_n_voxels * FLAGS_phase_gpu_n_voxels
          * FLAGS_phase_gpu_n_voxels) {
  VLOG(1) << "Initializing phase alignment with " << FLAGS_phase_gpu_n_voxels
          << " voxels in [" << FLAGS_phase_gpu_discretize_lower << ", "
          << FLAGS_phase_gpu_discretize_upper << "].";

  // Allocate memory for the FFT and IFFT.
  uint32_t data_size = sizeof(cufftComplex)*n_voxels_*FLAGS_phase_gpu_batch;
  cudaMalloc((void**)&F_, data_size);
  cudaMalloc((void**)&G_, data_size);
  cudaMalloc((void**)&C_, data_size);
  c_ = new double[n_voxels_];

  // Allocate memory for the function signals in the time domain.
  //f_ = Eigen::VectorXd::Zero(n_voxels_);
  //g_ = Eigen::VectorXd::Zero(n_voxels_);

  // Create the cuda plans for two FFTs and one IFFT.
  cufftPlan3d(&f_plan_, FLAGS_phase_gpu_n_voxels,
      FLAGS_phase_gpu_n_voxels, FLAGS_phase_gpu_n_voxels,
      CUFFT_R2C);
   cufftPlan3d(&c_plan_, FLAGS_phase_gpu_n_voxels,
      FLAGS_phase_gpu_n_voxels, FLAGS_phase_gpu_n_voxels,
      CUFFT_C2R);
}

PhaseAlignerGpu::~PhaseAlignerGpu() {
  cudaFree(F_);
  cudaFree(G_);
  cudaFree(C_);

  cufftDestroy(f_plan_);
  cufftDestroy(c_plan_);

  //fftw_cleanup();
  delete [] c_;
}

__global__ void correlation(cufftComplex* F, cufftComplex* G, uint32_t size) {
  const int idx = blockIdx.x*blockDim.x + threadIdx.x;
  if (idx >= size) return;

  F[idx].x = F[idx].x * G[idx].x - F[idx].y * (-G[idx].y);
  F[idx].x = F[idx].x * (-G[idx].y) + F[idx].y * G[idx].x;
}

  /*
void PhaseAlignerGpu::alignRegistered(
    const model::PointCloud& cloud_prev,
    const std::vector<model::FunctionValue>&,
    const model::PointCloud& cloud_reg,
    const std::vector<model::FunctionValue>&, common::Vector_t* xyz) {
  CHECK(xyz);
  auto start = std::chrono::high_resolution_clock::now();
  discretizePointcloud(cloud_prev, &f_, &hist_);
  discretizePointcloud(cloud_reg, &g_, &hist_);

  // Perform the two FFTs on the discretized signals.
  VLOG(1) << "Performing FFT on the first point cloud.";
  double* d_input;
  cudaMalloc((&d_input), sizeof(double)*n_voxels_);
  cudaMemcpy(d_input, f_.data(), sizeof(double)*n_voxels_, cudaMemcpyHostToDevice);
  cufftExecR2C(f_plan_, d_input, F_);
  VLOG(1) << "Performing FFT on the second point cloud.";
  cudaMemcpy(d_input, g_.data(), sizeof(double)*n_voxels_, cudaMemcpyHostToDevice);
  cufftExecR2C(f_plan_, d_input, G_);

  // Correlate the signals in the frequency domain.
  const uint32_t dim = FLAGS_phase_gpu_n_voxels;
  dim3 dimBlock(dim, dim, dim);
  dim3 dimGrid(1,1,1);
  correlation<<<dimGrid, dimBlock>>>(F_, G_, n_voxels_);

  // Perform the IFFT on the correlation tensor.
  VLOG(1) << "Performing IFFT on correlation.";
  double* d_input;
  cufftExecR2C(c_plan_, F_, d_input);
  cudaMemcpy(c_, d_input, sizeof(double)*n_voxels_, cudaMemcpyDeviceToHost);

  // Find the index that maximizes the correlation.
  auto end = std::chrono::high_resolution_clock::now();
  double duration =
      std::chrono::duration_cast<std::chrono::milliseconds>(end - start)
          .count();
  std::cout << "Translation alignment times: " << duration << " \n";
}
  */

std::vector<double> PhaseAlignerGpu::getCorrelation() const {
  return std::vector<double>(c_, c_+n_voxels_);
}

}  // namespace alignment
