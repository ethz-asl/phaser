#ifndef PHASER_BACKEND_UNCERTAINTY_BMM_PEAK_BASED_EVAL_H_
#define PHASER_BACKEND_UNCERTAINTY_BMM_PEAK_BASED_EVAL_H_

#include "phaser/backend/uncertainty/z-score-eval.h"
#include "phaser/distribution/bingham-mixture.h"

#include <set>
#include <vector>

namespace uncertainty {

class BmmPeakBasedEval : public ZScoreEval {
 public:
  BmmPeakBasedEval(
      const alignment::BaseAligner& aligner,
      const correlation::SphericalCorrelation& sph);

  common::BaseDistributionPtr evaluateCorrelation(
      const alignment::BaseAligner& aligner,
      const correlation::SphericalCorrelation& sph) override;

  common::BaseDistributionPtr evaluatePeakBasedCorrelation(
      const alignment::BaseAligner& aligner,
      const correlation::SphericalCorrelation& sph,
      const std::set<uint32_t>& signals,
      const std::vector<double>& normalized_corr) const override;

 private:
  common::BinghamMixturePtr fitRotationalBinghamDistribution(
      const correlation::SphericalCorrelation& sph,
      const std::set<uint32_t>& signals,
      const std::vector<double>& norm_corr) const;
  void calculateStartEndNeighbor(
      const uint32_t index, const uint32_t n_corr, uint32_t* start,
      uint32_t* end) const;
  void retrievePeakNeighbors(
      const uint32_t start, const uint32_t end,
      const std::vector<double>& norm_corr,
      const correlation::SphericalCorrelation& sph, Eigen::MatrixXd* samples,
      Eigen::RowVectorXd* weights) const;
};

}  // namespace uncertainty

#endif  // PHASER_BACKEND_UNCERTAINTY_BMM_PEAK_BASED_EVAL_H_
