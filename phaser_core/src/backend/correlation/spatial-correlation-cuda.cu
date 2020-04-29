#include <complex.h>
#include <cufft.h>
#include "phaser/backend/correlation/spatial-correlation-cuda.h"

#include <gflags/gflags.h>
#include <glog/logging.h>

DEFINE_double(phase_gpu_batch, 1, "");

namespace alignment {

SpatialCorrelationCuda::SpatialCorrelationCuda(const uint32_t voxels_per_dim)
    : n_voxels_total_(voxels_per_dim * voxels_per_dim * voxels_per_dim),
      n_voxels_per_dim_(voxels_per_dim) {
  // Allocate memory for the FFT and IFFT.
  const uint32_t data_size = sizeof(cufftDoubleComplex) * n_voxels_total_;
  cudaMalloc((void**)&F_, data_size);
  cudaMalloc((void**)&G_, data_size);
  cudaMalloc((void**)&C_, data_size);
  c_ = new double[n_voxels_total_];

  // Create the cuda plans for two FFTs and one IFFT.
  cufftPlan3d(
      &f_plan_, voxels_per_dim, voxels_per_dim, voxels_per_dim, CUFFT_D2Z);
  cufftPlan3d(
      &c_plan_, voxels_per_dim, voxels_per_dim, voxels_per_dim, CUFFT_Z2D);
}

SpatialCorrelationCuda::~SpatialCorrelationCuda() {
  cudaFree(F_);
  cudaFree(G_);
  cudaFree(C_);
  cufftDestroy(f_plan_);
  cufftDestroy(c_plan_);
  delete[] c_;
}

// CUDA kernel for the spatial phase correlation.
// Simple complex multiplication version.
__global__ void correlation(
    cufftDoubleComplex* F, cufftDoubleComplex* G, cufftDoubleComplex* C,
    uint32_t size) {
  // unique block index inside a 3D block grid
  const unsigned long long int blockId =
      blockIdx.x                             // 1D
      + blockIdx.y * gridDim.x               // 2D
      + gridDim.x * gridDim.y * blockIdx.z;  // 3D

  // global unique thread index, block dimension uses only x-coordinate
  const unsigned long long int idx = blockId * blockDim.x + threadIdx.x;

  C[idx].x = F[idx].x * G[idx].x - F[idx].y * (-G[idx].y);
  C[idx].y = F[idx].x * (-G[idx].y) + F[idx].y * G[idx].x;
}

double* SpatialCorrelationCuda::correlateSignals(
    double* const f, double* const g) {
  // Perform the two FFTs on the discretized signals.
  VLOG(1) << "Performing FFT on the first point cloud.";
  double* d_input;
  cudaMalloc((void**)(&d_input), sizeof(double) * n_voxels_total_);
  cudaMemcpy(
      d_input, f, sizeof(double) * n_voxels_total_, cudaMemcpyHostToDevice);
  // R2C is for floats, D2Z for doubles.
  if (cufftExecD2Z(f_plan_, d_input, F_) != CUFFT_SUCCESS) {
    LOG(FATAL) << "Forward cufft transform failed.";
  }

  VLOG(1) << "Performing FFT on the second point cloud.";
  cudaMemcpy(
      d_input, g, sizeof(double) * n_voxels_total_, cudaMemcpyHostToDevice);
  cufftExecD2Z(f_plan_, d_input, G_);

  // Correlate the signals in the frequency domain.
  const auto tile_width = 32;
  const auto n_threads = std::ceil((float)n_voxels_per_dim_ / tile_width);
  dim3 dimBlock(n_threads, n_threads, n_threads);
  dim3 dimGrid(tile_width, tile_width, 1);
  correlation<<<dimGrid, dimBlock>>>(F_, G_, C_, n_voxels_total_);

  // Perform the IFFT on the correlation tensor.
  VLOG(1) << "Performing IFFT on correlation.";
  cufftExecZ2D(c_plan_, C_, d_input);
  cudaMemcpy(
      c_, d_input, sizeof(double) * n_voxels_total_, cudaMemcpyDeviceToHost);
  return c_;
}

}  // namespace alignment
