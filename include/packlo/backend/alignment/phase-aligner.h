#pragma once

#include "packlo/backend/alignment/base-aligner.h"
#include <fftw3.h>

#include <array>

namespace alignment {

class PhaseAligner : public BaseAligner {
  public:
    PhaseAligner();
    ~PhaseAligner();

    virtual common::Vector_t alignRegistered(
      const model::PointCloud& cloud_prev, 
      const std::vector<model::FunctionValue>& f_prev, 
      const model::PointCloud& cloud_reg,
      const std::vector<model::FunctionValue>& f_reg) override;
  private: 
    void discretizePointcloud(
        const model::PointCloud& cloud, Eigen::VectorXd& f,
        Eigen::VectorXd& hist) const;
    std::size_t sub2ind(const std::size_t i, const std::size_t j, 
        const std::size_t k, const uint32_t rows, 
        const uint32_t cols) const;
    std::array<uint16_t, 3> ind2sub(const int lin_index, 
        const uint32_t rows, const uint32_t cols) const;
    double computeTranslationFromIndex(double index);

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

} // namespace alignment
