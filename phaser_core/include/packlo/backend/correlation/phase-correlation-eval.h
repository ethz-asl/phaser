#ifndef PACKLO_BACKEND_CORRELATION_PHASE_CORRELATION_EVAL_H_
#define PACKLO_BACKEND_CORRELATION_PHASE_CORRELATION_EVAL_H_

#include "packlo/backend/correlation/base-eval.h"

#include <memory>

namespace correlation {

class PhaseCorrelationEval {
 public:
  explicit PhaseCorrelationEval(
      BaseEvalPtr&& rotation, BaseEvalPtr&& positional);

  common::BaseDistributionPtr calcRotationUncertainty();
  common::BaseDistributionPtr calcTranslationUncertainty();

  const BaseEval& getRotationEval() const;
  const BaseEval& getPositionEval() const;

 private:
  BaseEvalPtr rotation_eval_;
  BaseEvalPtr positional_eval_;
};

using PhaseCorrelationEvalPtr = std::unique_ptr<PhaseCorrelationEval>;

}  // namespace correlation

#endif  // PACKLO_BACKEND_CORRELATION_PHASE_CORRELATION_EVAL_H_
