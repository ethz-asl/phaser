#ifndef PACKLO_BACKEND_CORRELATION_BASE_EVAL_H_
#define PACKLO_BACKEND_CORRELATION_BASE_EVAL_H_

#include "packlo/backend/alignment/base-aligner.h"
#include "packlo/backend/correlation/spherical-correlation.h"
#include "packlo/distribution/base-distribution.h"

#include <memory>
#include <vector>

namespace correlation {

class BaseEval {
 public:
  virtual common::BaseDistributionPtr evaluateCorrelation(
      const alignment::BaseAligner& aligner,
      const backend::SphericalCorrelation& sph) = 0;

  virtual common::BaseDistributionPtr evaluateCorrelationFromTranslation(
      const alignment::BaseAligner& aligner) = 0;

  virtual void evaluateCorrelationFromRotation(
      const std::vector<double>& corr) = 0;
};

using BaseEvalPtr = std::unique_ptr<BaseEval>;

}  // namespace correlation

#endif  // PACKLO_BACKEND_CORRELATION_BASE_EVAL_H_
