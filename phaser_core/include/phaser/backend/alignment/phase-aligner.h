#ifndef PACKLO_BACKEND_ALIGNMENT_PHASE_ALIGNER_H_
#define PACKLO_BACKEND_ALIGNMENT_PHASE_ALIGNER_H_

#include "phaser/backend/alignment/base-aligner.h"
#include <fftw3/fftw3.h>

#include <array>
#include <vector>

namespace alignment {

class PhaseAligner : public BaseAligner {
 public:
  PhaseAligner();
  ~PhaseAligner();

  void alignRegistered(
      const model::PointCloud& cloud_prev,
      const std::vector<model::FunctionValue>& f_prev,
      const model::PointCloud& cloud_reg,
      const std::vector<model::FunctionValue>& f_reg,
      common::Vector_t* xyz) override;

  std::vector<double> getCorrelation() const override;
  double computeTranslationFromIndex(double index) const;
  std::array<uint32_t, 3> ind2sub(const uint32_t lin_index) const;

 private:
  void discretizePointcloud(
      const model::PointCloud& cloud, Eigen::VectorXd* f,
      Eigen::VectorXd* hist) const;
  uint32_t sub2ind(
      const uint32_t i, const uint32_t j, const uint32_t k, const uint32_t rows,
      const uint32_t cols) const;
  std::array<uint32_t, 3> ind2sub(
      const uint32_t lin_index, const uint32_t rows, const uint32_t cols) const;

  fftw_plan f_plan_;
  fftw_plan g_plan_;
  fftw_plan c_plan_;
  fftw_complex *F_, *G_, *C_;
  double* c_;
  Eigen::VectorXd f_;
  Eigen::VectorXd g_;
  Eigen::VectorXd hist_;
  const uint32_t n_voxels_;
  std::vector<double> durations_;
};

}  // namespace alignment

#endif  // PACKLO_BACKEND_ALIGNMENT_PHASE_ALIGNER_H_
