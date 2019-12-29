#ifndef PACKLO_BACKEND_CORRELATION_BASE_EVAL_H_
#define PACKLO_BACKEND_CORRELATION_BASE_EVAL_H_

#include <vector>
#include <memory>

namespace correlation {

class BaseEval {
 public:
  virtual void evaluateCorrelationFromTranslation(
      const std::vector<double>& corr) = 0;

  virtual void evaluateCorrelationFromRotation(
      const std::vector<double>& corr) = 0;
};

using BaseEvalPtr = std::unique_ptr<BaseEval>;

}  // namespace correlation

#endif  // PACKLO_BACKEND_CORRELATION_BASE_EVAL_H_
