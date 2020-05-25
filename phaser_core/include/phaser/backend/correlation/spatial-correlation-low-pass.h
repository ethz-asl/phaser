#ifndef PHASER_BACKEND_CORRELATION_SPATIAL_CORRELATION_LOW_PASS_H_
#define PHASER_BACKEND_CORRELATION_SPATIAL_CORRELATION_LOW_PASS_H_

#include <vector>

#include "phaser/backend/correlation/spatial-correlation.h"

namespace phaser_core {

class SpatialCorrelationLowPass : public SpatialCorrelation {
 public:
  explicit SpatialCorrelationLowPass(
      const uint32_t n_voxels, const uint32_t zero_padding);
  explicit SpatialCorrelationLowPass(
      const uint32_t n_voxels, const uint32_t zero_padding,
      const uint32_t lower_bound, const uint32_t upper_bound);
  virtual ~SpatialCorrelationLowPass() = default;
  double* correlateSignals(
      const std::vector<Eigen::VectorXd*>& f,
      const std::vector<Eigen::VectorXd*>& g) override;

  void shiftSignals(fftw_complex* F, fftw_complex* G);
  void inverseShiftSignals(fftw_complex* C);

  uint32_t getLowerBound() const noexcept;
  uint32_t getUpperBound() const noexcept;
  uint32_t getNumberOfIndices() const noexcept;

 private:
  void computeIndicesBasedOnBounds();

  uint32_t low_pass_lower_bound_;
  uint32_t low_pass_upper_bound_;
  std::vector<uint32_t> linear_indices_;
};

}  // namespace phaser_core

#endif  // PHASER_BACKEND_CORRELATION_SPATIAL_CORRELATION_LOW_PASS_H_
