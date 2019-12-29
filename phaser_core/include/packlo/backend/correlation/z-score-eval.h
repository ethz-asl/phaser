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

  common::BaseDistributionPtr evaluateCorrelation(
      const alignment::BaseAligner& aligner,
      const backend::SphericalCorrelation& sph) override;

  common::BaseDistributionPtr evaluateCorrelationFromTranslation(
      const alignment::BaseAligner& aligner) override;

  void evaluateCorrelationFromRotation(
      const std::vector<double>& corr) override;

  ZScorePeakExtraction& getPeakExtraction();

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

}  // namespace correlation

#endif  // PACKLO_BACKEND_CORRELATION_Z_SCORE_EVAL_H_
