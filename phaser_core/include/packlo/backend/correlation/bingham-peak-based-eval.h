#ifndef PACKLO_BACKEND_CORRELATION_BINGHAM_PEAK_BASED_EVAL_H_
#define PACKLO_BACKEND_CORRELATION_BINGHAM_PEAK_BASED_EVAL_H_

#include "packlo/backend/alignment/base-aligner.h"
#include "packlo/backend/correlation/z-score-eval.h"
#include "packlo/distribution/bingham.h"

#include <set>
#include <vector>

namespace correlation {

class BinghamPeakBasedEval : public ZScoreEval {
 public:
  BinghamPeakBasedEval(
      const alignment::BaseAligner& aligner,
      const backend::SphericalCorrelation& sph);

  common::BaseDistributionPtr evaluateCorrelation(
      const alignment::BaseAligner& aligner,
      const backend::SphericalCorrelation& sph) override;

  common::BaseDistributionPtr evaluatePeakBasedCorrelation(
      const alignment::BaseAligner& aligner,
      const backend::SphericalCorrelation& sph,
      const std::set<uint32_t>& signals,
      const std::vector<double>& normalized_corr) const override;

 private:
  common::Bingham fitRotationalBinghamDistribution(
      const backend::SphericalCorrelation& sph,
      const std::set<uint32_t>& signals,
      const std::vector<double>& norm_corr) const;

  void calculateStartEndNeighbor(
      const uint32_t index, const uint32_t n_corr, uint32_t* start,
      uint32_t* end) const;

  void retrievePeakNeighbors(
      const uint32_t start, const uint32_t end,
      const std::vector<double>& norm_corr,
      const backend::SphericalCorrelation& sph, Eigen::MatrixXd* samples,
      Eigen::RowVectorXd* weights) const;
};

}  // namespace correlation

#endif  // PACKLO_BACKEND_CORRELATION_BINGHAM_PEAK_BASED_EVAL_H_
