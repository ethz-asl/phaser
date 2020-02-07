#include "phaser/backend/correlation/phase-correlation-eval.h"

#include <glog/logging.h>

namespace correlation {

PhaseCorrelationEval::PhaseCorrelationEval(
    BaseEvalPtr&& rotation, BaseEvalPtr&& positional)
    : rotation_eval_(std::move(rotation)),
      positional_eval_(std::move(positional)) {}

common::BaseDistributionPtr PhaseCorrelationEval::calcRotationUncertainty() {
  return rotation_eval_->evaluateCorrelationFromRotation();
}

common::BaseDistributionPtr PhaseCorrelationEval::calcTranslationUncertainty() {
  return positional_eval_->evaluateCorrelationFromTranslation();
}

BaseEval& PhaseCorrelationEval::getRotationEval() {
  CHECK_NOTNULL(rotation_eval_);
  return *rotation_eval_;
}

BaseEval& PhaseCorrelationEval::getPositionEval() {
  CHECK_NOTNULL(positional_eval_);
  return *positional_eval_;
}

}  // namespace correlation
