#ifndef PHASER_BACKEND_ALIGNMENT_BASE_ALIGNER_H_
#define PHASER_BACKEND_ALIGNMENT_BASE_ALIGNER_H_

#include <memory>
#include <vector>

#include "phaser/common/statistics-manager.h"
#include "phaser/model/point-cloud.h"

namespace phaser_core {

class BaseAligner {
 public:
  virtual void alignRegistered(
      const model::PointCloud& cloud_prev,
      const std::vector<model::FunctionValue>& f_prev,
      const model::PointCloud& cloud_reg,
      const std::vector<model::FunctionValue>& f_reg) = 0;

  virtual std::vector<double> getCorrelation() const = 0;
};

using BaseAlignerPtr = std::unique_ptr<BaseAligner>;

}  // namespace phaser_core

#endif  // PHASER_BACKEND_ALIGNMENT_BASE_ALIGNER_H_
