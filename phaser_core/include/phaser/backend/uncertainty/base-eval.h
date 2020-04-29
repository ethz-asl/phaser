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
  BaseEval(
      const alignment::BaseAligner& aligner,
      const backend::SphericalCorrelation& sph);
  virtual common::BaseDistributionPtr evaluateCorrelation(
      const alignment::BaseAligner& aligner,
      const backend::SphericalCorrelation& sph) = 0;

  virtual common::BaseDistributionPtr evaluateCorrelationFromTranslation() = 0;
  virtual common::BaseDistributionPtr evaluateCorrelationFromRotation() = 0;

  virtual void evaluateCorrelationFromRotation(
      const std::vector<double>& corr) = 0;

 protected:
  const alignment::BaseAligner& aligner_;
  const backend::SphericalCorrelation& sph_;
};

using BaseEvalPtr = std::unique_ptr<BaseEval>;

}  // namespace uncertainty

#endif  // PHASER_BACKEND_UNCERTAINTY_BASE_EVAL_H_
