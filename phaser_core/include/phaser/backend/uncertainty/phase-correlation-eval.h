#ifndef PHASER_BACKEND_UNCERTAINTY_PHASE_CORRELATION_EVAL_H_
#define PHASER_BACKEND_UNCERTAINTY_PHASE_CORRELATION_EVAL_H_

#include <memory>

#include "phaser/backend/alignment/phase-aligner.h"
#include "phaser/backend/correlation/spherical-correlation.h"
#include "phaser/backend/uncertainty/base-eval.h"

namespace phaser_core {

class PhaseCorrelationEval {
 public:
  explicit PhaseCorrelationEval(
      BaseEvalPtr&& rotation, BaseEvalPtr&& positional);

  common::BaseDistributionPtr calcRotationUncertainty(
      const SphericalCorrelation& sph_corr);
  common::BaseDistributionPtr calcTranslationUncertainty(
      const phaser_core::PhaseAligner& aligner);

  BaseEval& getRotationEval();
  BaseEval& getPositionEval();

 private:
  BaseEvalPtr rotation_eval_;
  BaseEvalPtr positional_eval_;
};

using PhaseCorrelationEvalPtr = std::unique_ptr<PhaseCorrelationEval>;

}  // namespace phaser_core

#endif  // PHASER_BACKEND_UNCERTAINTY_PHASE_CORRELATION_EVAL_H_
