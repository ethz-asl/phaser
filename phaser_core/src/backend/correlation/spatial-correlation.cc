#include "phaser/backend/correlation/spatial-correlation.h"

#include <numeric>

#include <emmintrin.h>
#include <glog/logging.h>

namespace correlation {

SpatialCorrelation::SpatialCorrelation(const uint32_t n_voxels)
    : total_n_voxels_(n_voxels * n_voxels * n_voxels),
      n_voxels_per_dim_(n_voxels) {
  const uint32_t n_fftw_size = sizeof(fftw_complex) * total_n_voxels_;
  F_ = static_cast<fftw_complex*>(fftw_malloc(n_fftw_size));
  G_ = static_cast<fftw_complex*>(fftw_malloc(n_fftw_size));
  C_ = static_cast<fftw_complex*>(fftw_malloc(n_fftw_size));

  c_ = new double[total_n_voxels_];
  f_ = new double[total_n_voxels_];
  g_ = new double[total_n_voxels_];

  // Create the FFTW plans for two FFTs and one IFFT.
  f_plan_ =
      fftw_plan_dft_r2c_3d(n_voxels, n_voxels, n_voxels, f_, F_, FFTW_ESTIMATE);

  g_plan_ =
      fftw_plan_dft_r2c_3d(n_voxels, n_voxels, n_voxels, g_, G_, FFTW_ESTIMATE);

  c_plan_ =
      fftw_plan_dft_c2r_3d(n_voxels, n_voxels, n_voxels, C_, c_, FFTW_ESTIMATE);
}

SpatialCorrelation::~SpatialCorrelation() {
  fftw_free(F_);
  fftw_free(G_);
  fftw_free(C_);

  fftw_destroy_plan(f_plan_);
  fftw_destroy_plan(g_plan_);
  fftw_destroy_plan(c_plan_);

  fftw_cleanup();
  delete[] c_;
  delete[] f_;
  delete[] g_;
}

void SpatialCorrelation::complexMulSeq(
    fftw_complex* F, fftw_complex* G, fftw_complex* C) {
  std::vector<uint32_t> indices(total_n_voxels_);
  std::iota(indices.begin(), indices.end(), 0);
  complexMulSeqUsingIndices(indices, F, G, C);
}

void SpatialCorrelation::complexMulVec(
    fftw_complex* F, fftw_complex* G, fftw_complex* C) {
  std::vector<uint32_t> indices(total_n_voxels_);
  std::iota(indices.begin(), indices.end(), 0);
  complexMulVecUsingIndices(indices, F, G, C);
}

void SpatialCorrelation::complexMulVecUsingIndices(
    const std::vector<uint32_t>& indices, fftw_complex* F, fftw_complex* G,
    fftw_complex* C) {
  CHECK(!indices.empty());
  VLOG(1) << "Performing vectorized correlation with " << indices.size()
          << " samples.";
  __m128d vec_F_real = _mm_setzero_pd();
  __m128d vec_F_img = _mm_setzero_pd();
  __m128d vec_G_real = _mm_setzero_pd();
  __m128d neg_vec_G_img = _mm_setzero_pd();
  __m128d vec_C_real = _mm_setzero_pd();
  __m128d vec_C_img = _mm_setzero_pd();
  const uint32_t n_points = indices.size();
  for (uint32_t idx = 0u; idx < n_points; idx += 2) {
    const uint32_t i = indices[idx];
    const uint32_t j = indices[idx + 1];
    vec_F_real[0] = F[i][0];
    vec_F_real[1] = F[j][0];
    vec_F_img[0] = F[i][1];
    vec_F_img[1] = F[j][1];
    vec_G_real[0] = G[i][0];
    vec_G_real[1] = G[j][0];
    neg_vec_G_img[0] = -G[i][1];
    neg_vec_G_img[1] = -G[j][1];
    vec_C_real[0] = C[i][0];
    vec_C_real[1] = C[j][0];
    vec_C_img[0] = C[i][1];
    vec_C_img[1] = C[j][1];

    // Perform complex multiplication.
    vec_C_real = _mm_sub_pd(
        _mm_mul_pd(vec_F_real, vec_G_real),
        _mm_mul_pd(vec_F_img, neg_vec_G_img));
    vec_C_img = _mm_add_pd(
        _mm_mul_pd(vec_F_real, neg_vec_G_img),
        _mm_mul_pd(vec_F_img, vec_G_real));

    // Write the memory back to C.
    C[i][0] = vec_C_real[0];
    C[i][1] = vec_C_img[0];
    C[j][0] = vec_C_real[1];
    C[j][1] = vec_C_img[1];
  }
}

void SpatialCorrelation::complexMulSeqUsingIndices(
    const std::vector<uint32_t>& indices, fftw_complex* F, fftw_complex* G,
    fftw_complex* C) {
  CHECK(!indices.empty());
  VLOG(1) << "Performing correlation using: " << indices.size() << " samples.";
  const uint32_t n_points = indices.size();
#pragma omp parallel for num_threads(8)
  for (uint32_t i = 0u; i < n_points; ++i) {
    C[i][0] = F[i][0] * G[i][0] - F[i][1] * (-G[i][1]);
    C[i][1] = F[i][0] * (-G[i][1]) + F[i][1] * G[i][0];
  }
}

double* SpatialCorrelation::correlateSignals(
    const std::vector<Eigen::VectorXd*>& f,
    const std::vector<Eigen::VectorXd*>& g) {
  CHECK_GT(f.size(), 0u);
  CHECK_GT(g.size(), 0u);

  const uint32_t function_size = total_n_voxels_ * sizeof(double);
  memcpy(f_, f[0]->data(), function_size);
  memcpy(g_, g[0]->data(), function_size);

  // Perform the two FFTs on the discretized signals.
  VLOG(1) << "Performing FFT on the first point cloud.";
  fftw_execute(f_plan_);
  VLOG(1) << "Performing FFT on the second point cloud.";
  fftw_execute(g_plan_);

  // Correlate the signals in the frequency domain.
  complexMulVec(F_, G_, C_);

  // Perform the IFFT on the correlation tensor.
  VLOG(1) << "Performing IFFT on correlation.";
  fftw_execute(c_plan_);
  return c_;
}

}  // namespace correlation
