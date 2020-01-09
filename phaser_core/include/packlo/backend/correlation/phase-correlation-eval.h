#ifndef PACKLO_BACKEND_CORRELATION_PHASE_CORRELATION_EVAL_H_
#define PACKLO_BACKEND_CORRELATION_PHASE_CORRELATION_EVAL_H_

#include "packlo/backend/correlation/base-eval.h"

namespace correlation {

class PhaseCorrelationEval {
 public:
  explicit PhaseCorrelationEval(
      BaseEvalPtr&& rotation, BaseEvalPtr&& positional);

 private:
  BaseEvalPtr rotation_eval_;
  BaseEvalPtr positional_eval_;
};

}  // namespace correlation

#endif  // PACKLO_BACKEND_CORRELATION_PHASE_CORRELATION_EVAL_H_
