#pragma once

#include "packlo/backend/alignment/base-aligner.h"

#include <array>

namespace alignment {

class PhaseAligner : public BaseAligner {
  public:
    virtual common::Vector_t alignRegistered(
      const model::PointCloud& cloud_prev, 
      const std::vector<model::FunctionValue>& f_prev, 
      const model::PointCloud& cloud_reg,
      const std::vector<model::FunctionValue>& f_reg) override;
  private: 
    Eigen::VectorXd discretizePointcloud(
        const model::PointCloud& cloud) const;
    std::size_t sub2ind(const std::size_t i, const std::size_t j, 
        const std::size_t k, const uint32_t rows, 
        const uint32_t cols) const;
    std::array<uint16_t, 3> ind2sub(const int lin_index, 
        const uint32_t rows, const uint32_t cols) const;
    double computeTranslationFromIndex(double index);
};

} // namespace alignment
