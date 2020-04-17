#ifndef PACKLO_BACKEND_UNCERTAINTY_GAUSSIAN_PEAK_BASED_EVAL_H_
#define PACKLO_BACKEND_UNCERTAINTY_GAUSSIAN_PEAK_BASED_EVAL_H_

#include "phaser/backend/alignment/base-aligner.h"
#include "phaser/backend/uncertainty/z-score-eval.h"
#include "phaser/distribution/gaussian.h"

#include <set>
#include <utility>
#include <vector>

namespace uncertainty {

class GaussianPeakBasedEval : public ZScoreEval {
 public:
  GaussianPeakBasedEval(
      const alignment::BaseAligner& aligner,
      const backend::SphericalCorrelation& sph);

  common::BaseDistributionPtr evaluatePeakBasedCorrelation(
      const alignment::BaseAligner& aligner,
      const backend::SphericalCorrelation& sph,
      const std::set<uint32_t>& signals,
      const std::vector<double>& norm_corr) const override;

 private:
  common::Gaussian fitTranslationalNormalDist(
      const alignment::BaseAligner& aligner, const std::set<uint32_t>& signals,
      const std::vector<double>& norm_corr) const;

  void calculateStartEndNeighbor(
      const uint32_t index, const uint32_t n_corr, uint32_t* start,
      uint32_t* end) const;

  void retrievePeakNeighbors(
      const uint32_t start, const uint32_t end,
      const std::vector<double>& norm_corr, const alignment::BaseAligner& sph,
      Eigen::ArrayXXd* samples, Eigen::VectorXd* weights) const;
 private:
};

}  // namespace uncertainty

#endif  // PACKLO_BACKEND_UNCERTAINTY_GAUSSIAN_PEAK_BASED_EVAL_H_