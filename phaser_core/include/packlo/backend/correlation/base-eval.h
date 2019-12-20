#pragma once

#include <vector>
#include <memory>

namespace correlation {

class BaseEval {
  public:

    virtual void evaluateCorrelationFromTranslation(
        const std::vector<double>& corr) = 0;
};

using BaseEvalPtr = std::unique_ptr<BaseEval>;

} // namespace correlation
