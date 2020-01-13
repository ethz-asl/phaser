#ifndef PACKLO_BACKEND_ALIGNMENT_PHASE_ALIGNER_H_
#define PACKLO_BACKEND_ALIGNMENT_PHASE_ALIGNER_H_

#include "packlo/backend/alignment/base-aligner.h"
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
  std::array<uint16_t, 3> ind2sub(const int lin_index) const;

 private:
  void discretizePointcloud(
      const model::PointCloud& cloud, Eigen::VectorXd* f,
      Eigen::VectorXd* hist) const;
  std::size_t sub2ind(
      const std::size_t i, const std::size_t j, const std::size_t k,
      const uint32_t rows, const uint32_t cols) const;
  std::array<uint16_t, 3> ind2sub(
      const int lin_index, const uint32_t rows, const uint32_t cols) const;

  fftw_plan f_plan_;
  fftw_plan g_plan_;
  fftw_plan c_plan_;
  fftw_complex *F_, *G_, *C_;
  double* c_;
  Eigen::VectorXd f_;
  Eigen::VectorXd g_;
  Eigen::VectorXd hist_;
  const uint32_t n_voxels_;
};

}  // namespace alignment

#endif  // PACKLO_BACKEND_ALIGNMENT_PHASE_ALIGNER_H_
