#ifndef PHASER_BACKEND_CORRELATION_SPATIAL_CORRELATION_H_
#define PHASER_BACKEND_CORRELATION_SPATIAL_CORRELATION_H_

#include <complex>
#include <vector>

#include <fftw3/fftw3.h>

#include "phaser/backend/correlation/base-spatial-correlation.h"

namespace correlation {

class SpatialCorrelation : public BaseSpatialCorrelation {
 public:
  explicit SpatialCorrelation(
      const uint32_t n_voxels, const uint32_t zero_padding = 0);
  virtual ~SpatialCorrelation();
  double* correlateSignals(
      const std::vector<Eigen::VectorXd*>& f,
      const std::vector<Eigen::VectorXd*>& g) override;

  uint32_t getZeroPadding() const;

 protected:
  void complexMulSeq(fftw_complex* F, fftw_complex* G, fftw_complex* C);
  void complexMulVec(fftw_complex* F, fftw_complex* G, fftw_complex* C);
  void complexMulSeqUsingIndices(
      const std::vector<uint32_t>& indices, fftw_complex* F, fftw_complex* G,
      fftw_complex* C);
  void complexMulVecUsingIndices(
      const std::vector<uint32_t>& indices, fftw_complex* F, fftw_complex* G,
      fftw_complex* C);
  uint32_t computeZeroPaddedIndex(const uint32_t idx);

  fftw_plan f_plan_;
  fftw_plan g_plan_;
  fftw_plan c_plan_;
  fftw_complex *F_, *G_, *C_;
  double* c_;
  double* f_;
  double* g_;
  Eigen::VectorXd hist_;
  const uint32_t total_n_voxels_;
  const uint32_t n_voxels_per_dim_;
  const uint32_t zero_padding_;
};

}  // namespace correlation

#endif  // PHASER_BACKEND_CORRELATION_SPATIAL_CORRELATION_H_
