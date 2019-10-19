#pragma once

#include "packlo/backend/correlation/base-eval.h"
#include <vector>

namespace correlation {

class ZScoreEval : public BaseEval {
  public:
    virtual void evaluateCorrelationFromTranslation(
        const std::vector<double>& corr) override;

  private:
    void calculateSmoothedZScore(const std::vector<double>& input, 
        std::vector<int>* signals) const;
    double mean(const std::vector<double>& vec) const;
    double stdDev(const std::vector<double>& vec) const;
};

} // namespace correlation
