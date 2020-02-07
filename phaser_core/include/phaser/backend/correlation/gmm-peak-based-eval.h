#ifndef PACKLO_BACKEND_CORRELATION_GMM_PEAK_BASED_EVAL_H_
#define PACKLO_BACKEND_CORRELATION_GMM_PEAK_BASED_EVAL_H_

#include "phaser/backend/alignment/base-aligner.h"
#include "phaser/backend/correlation/z-score-eval.h"
#include "phaser/distribution/gaussian-mixture.h"
#include "phaser/model/gmm-parameters.h"

#include <Eigen/Dense>
#include <set>
#include <utility>
#include <vector>

namespace correlation {

class GmmPeakBasedEval : public ZScoreEval {
 public:
  GmmPeakBasedEval(
      const alignment::BaseAligner& aligner,
      const backend::SphericalCorrelation& sph);
  common::BaseDistributionPtr evaluatePeakBasedCorrelation(
      const alignment::BaseAligner& aligner,
      const backend::SphericalCorrelation& sph,
      const std::set<uint32_t>& signals,
      const std::vector<double>& normalized_corr) const override;

 private:
  void calculateStartEndNeighbor(
      const uint32_t index, const uint32_t n_corr, uint32_t* start,
      uint32_t* end) const;
  void fitTranslationalGmmDistribution(
      const alignment::BaseAligner& aligner, const std::set<uint32_t>& signals,
      const std::vector<double>& n_corr, common::GaussianMixturePtr gm) const;
  void retrievePeakNeighbors(
      const uint32_t start, const uint32_t end,
      const std::vector<double>& n_corr, const alignment::BaseAligner& aligner,
      Eigen::MatrixXd* samples, Eigen::VectorXd* gaussian_weights) const;
};

}  // namespace correlation

#endif  // PACKLO_BACKEND_CORRELATION_GMM_PEAK_BASED_EVAL_H_
