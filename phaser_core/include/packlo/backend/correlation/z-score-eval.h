#ifndef PACKLO_BACKEND_CORRELATION_Z_SCORE_EVAL_H_
#define PACKLO_BACKEND_CORRELATION_Z_SCORE_EVAL_H_

#include "packlo/backend/correlation/base-eval.h"
#include "packlo/backend/correlation/z-score-peak-extraction.h"
#include "packlo/common/statistics-manager.h"

#include <cstdint>
#include <set>
#include <utility>
#include <vector>

namespace correlation {

class ZScoreEval : public BaseEval {
 public:
  ZScoreEval();

  void evaluateCorrelationFromTranslation(
      const std::vector<double>& corr) override;

 private:
  std::pair<double, double> fitSmoothedNormalDist(
      const std::set<uint32_t>& signals,
      const std::vector<double>& input) const;

  common::StatisticsManager manager_;
  ZScorePeakExtraction peak_extraction_;
};

}  // namespace correlation

#endif  // PACKLO_BACKEND_CORRELATION_Z_SCORE_EVAL_H_
