#pragma once

#include "packlo/backend/correlation/base-eval.h"
#include "packlo/common/statistics-manager.h"

#include <cstdint>
#include <vector>
#include <set>

namespace correlation {

class ZScoreEval : public BaseEval {
  public:
    ZScoreEval();

    virtual void evaluateCorrelationFromTranslation(
        const std::vector<double>& corr) override;

  private:
    void calculateSmoothedZScore(std::vector<double>& input,
        const double lag, const double threshold, const double influence,
        std::set<uint32_t>* signals) const;
    double stdDev(const double mean, const std::vector<double>& vec, 
        uint32_t from, uint32_t to) const;

    common::StatisticsManager manager_;
};

} // namespace correlation
