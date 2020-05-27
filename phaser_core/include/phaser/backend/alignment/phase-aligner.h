#ifndef PHASER_BACKEND_ALIGNMENT_PHASE_ALIGNER_H_
#define PHASER_BACKEND_ALIGNMENT_PHASE_ALIGNER_H_

#include <array>
#include <vector>

#include <fftw3/fftw3.h>

#include "phaser/backend/alignment/base-aligner.h"
#include "phaser/backend/correlation/base-spatial-correlation.h"

namespace phaser_core {

class PhaseAligner : public BaseAligner {
 public:
  PhaseAligner();

  void alignRegistered(
      const model::PointCloud& cloud_prev,
      const std::vector<model::FunctionValue>& f_prev,
      const model::PointCloud& cloud_reg,
      const std::vector<model::FunctionValue>& f_reg) override;

  std::vector<double> getCorrelation() const override;
  uint32_t getNumberOfVoxels() const noexcept;
  uint32_t getLowerBound() const noexcept;
  uint32_t getUpperBound() const noexcept;
  double computeTranslationFromIndex(double index) const;
  std::array<uint32_t, 3> ind2sub(const uint32_t lin_index) const;

 private:
  void discretizePointcloud(
      const model::PointCloud& cloud, const std::vector<Eigen::VectorXd*>& f,
      Eigen::VectorXd* hist) const;
  uint32_t sub2ind(
      const uint32_t i, const uint32_t j, const uint32_t k, const uint32_t rows,
      const uint32_t cols) const;
  std::array<uint32_t, 3> ind2sub(
      const uint32_t lin_index, const uint32_t rows, const uint32_t cols) const;
  void normalizeSignal(const Eigen::VectorXd& hist, Eigen::VectorXd* f) const;

  Eigen::VectorXd f_intensities_;
  Eigen::VectorXd f_ranges_;
  Eigen::VectorXd f_reflectivity_;
  Eigen::VectorXd f_ambient_;
  Eigen::VectorXd g_intensities_;
  Eigen::VectorXd g_ranges_;
  Eigen::VectorXd g_reflectivity_;
  Eigen::VectorXd g_ambient_;

  Eigen::VectorXd hist_;
  const uint32_t n_voxels_;
  const uint32_t total_n_voxels_;
  const int lower_bound_;
  const int upper_bound_;
  Eigen::VectorXf edges_;
  BaseSpatialCorrelationPtr spatial_correlation_;
  std::vector<double> previous_correlation_;
};

}  // namespace phaser_core

#endif  // PHASER_BACKEND_ALIGNMENT_PHASE_ALIGNER_H_
