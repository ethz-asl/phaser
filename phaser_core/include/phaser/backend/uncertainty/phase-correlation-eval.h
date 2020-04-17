#ifndef PACKLO_BACKEND_UNCERTAINTY_PHASE_CORRELATION_EVAL_H_
#define PACKLO_BACKEND_UNCERTAINTY_PHASE_CORRELATION_EVAL_H_

#include "phaser/backend/uncertainty/base-eval.h"

#include <memory>

namespace uncertainty {

class PhaseCorrelationEval {
 public:
  explicit PhaseCorrelationEval(
      BaseEvalPtr&& rotation, BaseEvalPtr&& positional);

  common::BaseDistributionPtr calcRotationUncertainty();
  common::BaseDistributionPtr calcTranslationUncertainty();

  BaseEval& getRotationEval();
  BaseEval& getPositionEval();

 private:
  BaseEvalPtr rotation_eval_;
  BaseEvalPtr positional_eval_;
};

using PhaseCorrelationEvalPtr = std::unique_ptr<PhaseCorrelationEval>;

}  // namespace uncertainty

#endif  // PACKLO_BACKEND_UNCERTAINTY_PHASE_CORRELATION_EVAL_H_
