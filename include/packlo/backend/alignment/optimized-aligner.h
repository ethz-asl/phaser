#pragma once

#include "packlo/backend/alignment/base-aligner.h"
#include "packlo/backend/alignment/objective.h"

namespace alignment {

class OptimizedAligner : public BaseAligner {
  public:
    virtual common::Vector_t alignRegistered(
      const model::PointCloud& cloud_prev, 
      const std::vector<model::FunctionValue>& f_prev, 
      const model::PointCloud& cloud_reg,
      const std::vector<model::FunctionValue>& f_reg) override;
  private:
    Objective objective_;
};

} // namespace alignment
