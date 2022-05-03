#include "phaser/backend/correlation/spatial-correlation.h"

#include <emmintrin.h>
#include <glog/logging.h>
#include <numeric>

#include "phaser/common/signal-utils.h"

namespace phaser_core {

SpatialCorrelation::SpatialCorrelation(
    const uint32_t n_voxels, const uint32_t zero_padding)
    : total_n_voxels_(n_voxels * n_voxels * n_voxels),
      n_voxels_per_dim_(n_voxels),
      zero_padding_(zero_padding) {
  // If padding is set calculate the ratio to the original size.
  const uint32_t padding_per_dim = 2u * zero_padding;
  const double ratio_n_padding_per_dim =
      (n_voxels + padding_per_dim) / static_cast<double>(n_voxels);
  const double total_n_padding_factor = ratio_n_padding_per_dim *
                                        ratio_n_padding_per_dim *
                                        ratio_n_padding_per_dim;
  total_n_voxels_padded_ =
      static_cast<uint32_t>(total_n_voxels_ * total_n_padding_factor);
  VLOG(1) << "Initializing spatial correlation with padding " << zero_padding
          << " (ratio: " << ratio_n_padding_per_dim
          << ", factor: " << total_n_padding_factor << ").";
  VLOG(1) << "padded size: " << total_n_voxels_padded_;

  const uint32_t n_fftw_size = sizeof(fftw_complex) * total_n_voxels_;
  F_ = static_cast<fftw_complex*>(fftw_malloc(n_fftw_size));
  G_ = static_cast<fftw_complex*>(fftw_malloc(n_fftw_size));
  C_ = static_cast<fftw_complex*>(
      fftw_malloc(sizeof(fftw_complex) * total_n_voxels_padded_));

  for (uint32_t i = 0u; i < total_n_voxels_; ++i) {
    F_[i][0] = 0.0;
    F_[i][1] = 0.0;
    G_[i][0] = 0.0;
    G_[i][1] = 0.0;
  }
  for (uint32_t i = 0u; i < total_n_voxels_padded_; ++i) {
    C_[i][0] = 0.0;
    C_[i][1] = 0.0;
  }

  c_ = new double[total_n_voxels_padded_]{};
  f_ = new double[total_n_voxels_]{};
  g_ = new double[total_n_voxels_]{};

  // Create the FFTW plans for two FFTs and one IFFT.
  f_plan_ =
      fftw_plan_dft_r2c_3d(n_voxels, n_voxels, n_voxels, f_, F_, FFTW_ESTIMATE);

  g_plan_ =
      fftw_plan_dft_r2c_3d(n_voxels, n_voxels, n_voxels, g_, G_, FFTW_ESTIMATE);

  c_plan_ = fftw_plan_dft_c2r_3d(
      n_voxels + padding_per_dim, n_voxels + padding_per_dim,
      n_voxels + padding_per_dim, C_, c_, FFTW_ESTIMATE);
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
  for (uint32_t idx = 0u; idx < n_points - 1; idx += 2) {
    const uint32_t i = indices[idx];
    const uint32_t j = indices[idx + 1];
    const uint32_t padded_i =
        zero_padding_ != 0u ? computeZeroPaddedIndex(i) : i;
    const uint32_t padded_j =
        zero_padding_ != 0u ? computeZeroPaddedIndex(j) : j;

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
    C[padded_i][0] = vec_C_real[0];
    C[padded_i][1] = vec_C_img[0];
    C[padded_j][0] = vec_C_real[1];
    C[padded_j][1] = vec_C_img[1];
  }
}

void SpatialCorrelation::complexMulSeqUsingIndices(
    const std::vector<uint32_t>& indices, fftw_complex* F, fftw_complex* G,
    fftw_complex* C) {
  CHECK(!indices.empty());
  VLOG(3) << "Performing correlation using: " << indices.size() << " samples.";
  const uint32_t n_points = indices.size();
#pragma omp parallel for num_threads(2)
  for (uint32_t i = 0u; i < n_points; ++i) {
    const uint32_t idx = indices[i];
    const uint32_t padded_idx =
        zero_padding_ != 0u ? computeZeroPaddedIndex(idx) : idx;
    CHECK_LT(idx, total_n_voxels_);
    CHECK_GE(idx, 0);
    CHECK_LT(padded_idx, total_n_voxels_padded_);
    CHECK_GE(padded_idx, 0);

    C[padded_idx][0] = F[idx][0] * G[idx][0] - F[idx][1] * (-G[idx][1]);
    C[padded_idx][1] = F[idx][0] * (-G[idx][1]) + F[idx][1] * G[idx][0];
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
  complexMulSeq(F_, G_, C_);

  double max_val = 0;
  uint32_t idx = 0u;
  for (size_t i = 0u; i < total_n_voxels_padded_; ++i) {
    double energy = sqrt(C_[i][0] * C_[i][0] + C_[i][1] * C_[i][1]);
    if (energy > max_val) {
      max_val = energy;
      idx = i;
    }
  }
  const uint32_t padding_per_dim = 2u * zero_padding_;
  std::array<uint32_t, 3> ijk = common::SignalUtils::Ind2Sub(
      idx, n_voxels_per_dim_ + padding_per_dim,
      n_voxels_per_dim_ + padding_per_dim);

  // Perform the IFFT on the correlation tensor.
  VLOG(1) << "Performing IFFT on correlation.";
  fftw_execute(c_plan_);

  if (zero_padding_ != 0u) {
    // Not sure yet why we have to do this.
    setCorrelationToZero();
  }

  return c_;
}

void SpatialCorrelation::setCorrelationToZero() {
  for (uint32_t i = 0u; i < total_n_voxels_padded_; ++i) {
    C_[i][0] = 0.0;
    C_[i][1] = 0.0;
  }
}

uint32_t SpatialCorrelation::getZeroPadding() const {
  return zero_padding_;
}

uint32_t SpatialCorrelation::getCorrelationSize() const {
  return total_n_voxels_padded_;
}

uint32_t SpatialCorrelation::computeZeroPaddedIndex(const uint32_t idx) {
  std::array<uint32_t, 3> ijk =
      common::SignalUtils::Ind2Sub(idx, n_voxels_per_dim_, n_voxels_per_dim_);
  const uint32_t padded_voxels_per_dim = n_voxels_per_dim_ + 2 * zero_padding_;
  return common::SignalUtils::Sub2Ind(
      ijk[0] + zero_padding_, ijk[1] + zero_padding_, ijk[2] + zero_padding_,
      padded_voxels_per_dim, padded_voxels_per_dim);
}

}  // namespace phaser_core
