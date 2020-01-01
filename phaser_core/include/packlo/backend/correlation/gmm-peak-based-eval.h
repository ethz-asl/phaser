#ifndef PACKLO_BACKEND_CORRELATION_GMM_PEAK_BASED_EVAL_H_
#define PACKLO_BACKEND_CORRELATION_GMM_PEAK_BASED_EVAL_H_

#include "packlo/backend/alignment/base-aligner.h"
#include "packlo/backend/correlation/z-score-eval.h"
#include "packlo/model/gmm-parameters.h"

#include <Eigen/Dense>
#include <set>
#include <utility>
#include <vector>

namespace correlation {

class GmmPeakBasedEval : public ZScoreEval {
 public:
  common::BaseDistributionPtr evaluatePeakBasedCorrelation(
      const alignment::BaseAligner& aligner, const std::set<uint32_t>& signals,
      const std::vector<double>& normalized_corr) const override;

 private:
  model::GmmParameters fitTranslationalGmmDistribution(
      const alignment::BaseAligner& aligner,
      const std::set<uint32_t>& signals) const;
  void retrievePeakNeighbors(
      const uint32_t index, Eigen::ArrayXXd* samples,
      Eigen::VectorXd* gaussian_weights) const;
};

}  // namespace correlation

#endif  // PACKLO_BACKEND_CORRELATION_GMM_PEAK_BASED_EVAL_H_
