#ifndef PACKLO_BACKEND_CORRELATION_GAUSSIAN_PEAK_BASED_EVAL_H_
#define PACKLO_BACKEND_CORRELATION_GAUSSIAN_PEAK_BASED_EVAL_H_

#include "packlo/backend/alignment/base-aligner.h"
#include "packlo/backend/correlation/z-score-eval.h"

#include <set>
#include <utility>
#include <vector>

namespace correlation {

class GaussianPeakBasedEval : public ZScoreEval {
 public:
  common::BaseDistributionPtr evaluatePeakBasedCorrelation(
      const alignment::BaseAligner& aligner, const std::set<uint32_t>& signals,
      const std::vector<double>& normalized_corr) const override;

 private:
  std::pair<Eigen::VectorXd, Eigen::MatrixXd> fitTranslationalNormalDist(
      const alignment::BaseAligner& aligner,
      const std::set<uint32_t>& signals) const;
};

}  // namespace correlation

#endif  // PACKLO_BACKEND_CORRELATION_GAUSSIAN_PEAK_BASED_EVAL_H_
