#include "phaser/backend/alignment/spatial-correlation-cuda.h"
#include <complex.h>
#include <cufft.h>

#include <gflags/gflags.h>
#include <glog/logging.h>

DEFINE_double(
    phase_gpu_batch, 1,
    "");

namespace alignment {

SpatialCorrelationCuda::SpatialCorrelationCuda(const uint32_t voxels_per_dim)
    : n_voxels_total_(voxels_per_dim * voxels_per_dim * voxels_per_dim),
      n_voxels_per_dim_(voxels_per_dim) {

  // Allocate memory for the FFT and IFFT.
  const uint32_t data_size = sizeof(cufftComplex) * n_voxels_total_
    * FLAGS_phase_gpu_batch;
  cudaMalloc((void**)&F_, data_size);
  cudaMalloc((void**)&G_, data_size);
  cudaMalloc((void**)&C_, data_size);
  c_ = new double[n_voxels_total_];

  // Create the cuda plans for two FFTs and one IFFT.
  cufftPlan3d(&f_plan_, voxels_per_dim, voxels_per_dim, voxels_per_dim,
    CUFFT_R2C);
  cufftPlan3d(&c_plan_, voxels_per_dim, voxels_per_dim, voxels_per_dim,
    CUFFT_C2R);
}

SpatialCorrelationCuda::~SpatialCorrelationCuda() {
  cudaFree(F_);
  cudaFree(G_);
  cudaFree(C_);

  cufftDestroy(f_plan_);
  cufftDestroy(c_plan_);

  delete [] c_;
}

// CUDA kernel for the spatial phase correlation.
__global__ void correlation(cufftDoubleComplex* F, cufftDoubleComplex* G,
    uint32_t size) {
  const int idx = blockIdx.x*blockDim.x + threadIdx.x;
  if (idx >= size) return;

  F[idx].x = F[idx].x * G[idx].x - F[idx].y * (-G[idx].y);
  F[idx].y = F[idx].x * (-G[idx].y) + F[idx].y * G[idx].x;
}

void SpatialCorrelationCuda::correlateSignals(double* const f,
    double* const g) {
  // Perform the two FFTs on the discretized signals.
  VLOG(1) << "Performing FFT on the first point cloud.";
  double* d_input;
  cudaMalloc((&d_input), sizeof(double)*n_voxels_total_);
  cudaMemcpy(d_input, f, sizeof(double)*n_voxels_total_,
    cudaMemcpyHostToDevice);
  // R2C is for floats, D2Z for doubles.
  cufftExecD2Z(f_plan_, d_input, F_);

  VLOG(1) << "Performing FFT on the second point cloud.";
  cudaMemcpy(d_input, g, sizeof(double)*n_voxels_total_,
    cudaMemcpyHostToDevice);
  cufftExecD2Z(f_plan_, d_input, G_);

  // Correlate the signals in the frequency domain.
  dim3 dimBlock(n_voxels_per_dim_, n_voxels_per_dim_, n_voxels_per_dim_);
  dim3 dimGrid(1,1,1);
  correlation<<<dimGrid, dimBlock>>>(F_, G_, n_voxels_total_);

  // Perform the IFFT on the correlation tensor.
  VLOG(1) << "Performing IFFT on correlation.";
  cufftExecZ2D(c_plan_, F_, d_input);
  cudaMemcpy(c_, d_input, sizeof(double)*n_voxels_total_, cudaMemcpyDeviceToHost);
}

}  // namespace alignment
