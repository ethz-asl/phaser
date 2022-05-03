#ifndef PHASER_BACKEND_UNCERTAINTY_BASE_EVAL_H_
#define PHASER_BACKEND_UNCERTAINTY_BASE_EVAL_H_

#include <memory>
#include <vector>

#include "phaser/backend/alignment/base-aligner.h"
#include "phaser/backend/correlation/spherical-correlation.h"
#include "phaser/distribution/base-distribution.h"

namespace phaser_core {

class BaseEval {
 public:
  virtual common::BaseDistributionPtr evaluateCorrelationFromTranslation(
      const uint32_t n_voxels, const int discretize_lower_bound,
      const int discretize_upper_bound, const std::vector<double>& corr) = 0;
  virtual common::BaseDistributionPtr evaluateCorrelationFromRotation(
      const uint32_t bw, const std::vector<double>& corr) = 0;
};

using BaseEvalPtr = std::unique_ptr<BaseEval>;

}  // namespace phaser_core

#endif  // PHASER_BACKEND_UNCERTAINTY_BASE_EVAL_H_
