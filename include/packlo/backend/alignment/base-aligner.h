#pragma once

#include "packlo/model/point-cloud.h"
#include "packlo/common/statistics-manager.h"

#include <memory>
#include <vector>

namespace alignment {

class BaseAligner {
  public:
    virtual common::Vector_t alignRegistered(
      const model::PointCloud& cloud_prev, 
      const std::vector<model::FunctionValue>& f_prev, 
      const model::PointCloud& cloud_reg,
      const std::vector<model::FunctionValue>& f_reg) = 0;
};

using BaseAlignerPtr = std::unique_ptr<BaseAligner>;

} // namespace alignment
