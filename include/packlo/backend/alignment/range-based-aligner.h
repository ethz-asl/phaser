#pragma once

#include "packlo/backend/alignment/base-aligner.h"

namespace alignment {

class RangeBasedAligner : public BaseAligner {
  public:
    virtual void alignRegistered(
      const model::PointCloud& cloud_prev, 
      const std::vector<model::FunctionValue>& f_prev, 
      const model::PointCloud& cloud_reg,
      const std::vector<model::FunctionValue>& f_reg, 
      common::Vector_t* xyz) override;
};

} // namespace alignment
