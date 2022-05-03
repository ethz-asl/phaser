#ifndef PHASER_BACKEND_UNCERTAINTY_Z_SCORE_EVAL_H_
#define PHASER_BACKEND_UNCERTAINTY_Z_SCORE_EVAL_H_

#include <cstdint>
#include <set>
#include <utility>
#include <vector>

#include "phaser/backend/uncertainty/base-eval.h"
#include "phaser/backend/uncertainty/z-score-peak-extraction.h"
#include "phaser/common/statistics-manager.h"

namespace phaser_core {

class ZScoreEval : public BaseEval {
 public:
  ZScoreEval();

  common::BaseDistributionPtr evaluateCorrelationFromTranslation(
      const uint32_t n_voxels, const int discretize_lower_bound,
      const int discretize_upper_bound,
      const std::vector<double>& corr) override;
  common::BaseDistributionPtr evaluateCorrelationFromRotation(
      const uint32_t bw, const std::vector<double>& corr) override;

  ZScorePeakExtraction& getPeakExtraction();
  virtual common::BaseDistributionPtr evaluatePeakBasedCorrelation(
      const uint32_t bw, const std::set<uint32_t>& signals,
      const std::vector<double>& normalized_corr) const;
  virtual common::BaseDistributionPtr evaluatePeakBasedCorrelation(
      const uint32_t n_voxels, const int discretize_lower_bound,
      const int discretize_upper_bound, const std::set<uint32_t>& signals,
      const std::vector<double>& normalized_corr) const;

 private:
  void evaluateCorrelationVector(
      const std::vector<double>& corr, std::set<uint32_t>* signals,
      std::vector<double>* n_corr_ds);

  common::StatisticsManager manager_;
  ZScorePeakExtraction peak_extraction_;
};

}  // namespace phaser_core

#endif  // PHASER_BACKEND_UNCERTAINTY_Z_SCORE_EVAL_H_
