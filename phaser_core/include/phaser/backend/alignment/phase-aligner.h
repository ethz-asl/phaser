#ifndef PHASER_BACKEND_ALIGNMENT_PHASE_ALIGNER_H_
#define PHASER_BACKEND_ALIGNMENT_PHASE_ALIGNER_H_

#include <array>
#include <fftw3/fftw3.h>
#include <vector>

#include "phaser/backend/alignment/base-aligner.h"
#include "phaser/backend/correlation/spatial-correlation.h"

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

 private:
  void discretizePointcloud(
      const model::PointCloud& cloud, const std::vector<Eigen::VectorXd*>& f,
      Eigen::VectorXd* hist) const;
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
  SpatialCorrelationPtr spatial_correlation_;
  std::vector<double> previous_correlation_;
};

}  // namespace phaser_core

#endif  // PHASER_BACKEND_ALIGNMENT_PHASE_ALIGNER_H_
