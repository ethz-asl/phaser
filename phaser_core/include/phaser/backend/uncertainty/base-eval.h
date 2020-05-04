#ifndef PHASER_BACKEND_UNCERTAINTY_BASE_EVAL_H_
#define PHASER_BACKEND_UNCERTAINTY_BASE_EVAL_H_

#include "phaser/backend/alignment/base-aligner.h"
#include "phaser/backend/correlation/spherical-correlation.h"
#include "phaser/distribution/base-distribution.h"

#include <memory>
#include <vector>

namespace uncertainty {

class BaseEval {
 public:
  virtual common::BaseDistributionPtr evaluateCorrelationFromTranslation() = 0;
  virtual common::BaseDistributionPtr evaluateCorrelationFromRotation(
      const uint32_t bw, const std::vector<double>& corr) = 0;
};

using BaseEvalPtr = std::unique_ptr<BaseEval>;

}  // namespace uncertainty

#endif  // PHASER_BACKEND_UNCERTAINTY_BASE_EVAL_H_
