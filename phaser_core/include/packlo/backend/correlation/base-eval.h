#ifndef PACKLO_BACKEND_CORRELATION_BASE_EVAL_H_
#define PACKLO_BACKEND_CORRELATION_BASE_EVAL_H_

#include <memory>
#include <vector>
#include "packlo/backend/alignment/base-aligner.h"
#include "packlo/distribution/base-distribution.h"

namespace correlation {

class BaseEval {
 public:
  virtual common::BaseDistributionPtr evaluateCorrelationFromTranslation(
      const alignment::BaseAligner& aligner) = 0;

  virtual void evaluateCorrelationFromRotation(
      const std::vector<double>& corr) = 0;
};

using BaseEvalPtr = std::unique_ptr<BaseEval>;

}  // namespace correlation

#endif  // PACKLO_BACKEND_CORRELATION_BASE_EVAL_H_
