#pragma once

#include <vector>

namespace correlation {

class BaseEval {
  public:

    virtual void evaluateCorrelationFromTranslation(
        const std::vector<double>& corr) = 0;
};

} // namespace correlation
