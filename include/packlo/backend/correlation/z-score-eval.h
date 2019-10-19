#pragma once

#include "packlo/backend/correlation/base-eval.h"
#include <cstdint>
#include <vector>

namespace correlation {

class ZScoreEval : public BaseEval {
  public:
    ZScoreEval();

    virtual void evaluateCorrelationFromTranslation(
        const std::vector<double>& corr) override;

  private:
    void calculateSmoothedZScore(const std::vector<double>& input, 
        std::vector<uint32_t>* signals) const;
    double mean(const std::vector<double>& vec) const;
    double stdDev(const double mean, const std::vector<double>& vec) const;
};

} // namespace correlation
