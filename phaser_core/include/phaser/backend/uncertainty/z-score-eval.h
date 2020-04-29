#ifndef PHASER_BACKEND_UNCERTAINTY_Z_SCORE_EVAL_H_
#define PHASER_BACKEND_UNCERTAINTY_Z_SCORE_EVAL_H_

#include "phaser/backend/uncertainty/base-eval.h"
#include "phaser/backend/uncertainty/z-score-peak-extraction.h"
#include "phaser/common/statistics-manager.h"

#include <cstdint>
#include <set>
#include <utility>
#include <vector>

namespace uncertainty {

class ZScoreEval : public BaseEval {
 public:
  ZScoreEval(
      const alignment::BaseAligner& aligner,
      const correlation::SphericalCorrelation& sph);

  common::BaseDistributionPtr evaluateCorrelation(
      const alignment::BaseAligner& aligner,
      const correlation::SphericalCorrelation& sph) override;

  common::BaseDistributionPtr evaluateCorrelationFromTranslation() override;
  common::BaseDistributionPtr evaluateCorrelationFromRotation() override;

  void evaluateCorrelationFromRotation(
      const std::vector<double>& corr) override;

  ZScorePeakExtraction& getPeakExtraction();

  virtual common::BaseDistributionPtr evaluatePeakBasedCorrelation(
      const alignment::BaseAligner& aligner,
      const correlation::SphericalCorrelation& sph,
      const std::set<uint32_t>& signals,
      const std::vector<double>& normalized_corr) const = 0;

 private:
  void evaluateCorrelationVector(
      const std::vector<double>& corr, std::set<uint32_t>* signals,
      std::vector<double>* n_corr_ds);
  std::pair<double, double> fitSmoothedNormalDist(
      const std::set<uint32_t>& signals,
      const std::vector<double>& input) const;
  std::pair<Eigen::VectorXd, Eigen::MatrixXd> fitTranslationalNormalDist(
      const alignment::BaseAligner& aligner,
      const std::set<uint32_t>& signals) const;

  common::StatisticsManager manager_;
  ZScorePeakExtraction peak_extraction_;
};

}  // namespace uncertainty

#endif  // PHASER_BACKEND_UNCERTAINTY_Z_SCORE_EVAL_H_
