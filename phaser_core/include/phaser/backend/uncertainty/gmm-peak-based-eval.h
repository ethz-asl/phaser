#ifndef PHASER_BACKEND_UNCERTAINTY_GMM_PEAK_BASED_EVAL_H_
#define PHASER_BACKEND_UNCERTAINTY_GMM_PEAK_BASED_EVAL_H_

#include <Eigen/Dense>
#include <set>
#include <utility>
#include <vector>

#include "phaser/backend/alignment/base-aligner.h"
#include "phaser/backend/uncertainty/z-score-eval.h"
#include "phaser/distribution/gaussian-mixture.h"
#include "phaser/model/gmm-parameters.h"

namespace phaser_core {

class GmmPeakBasedEval : public ZScoreEval {
 public:
  common::BaseDistributionPtr evaluatePeakBasedCorrelation(
      const uint32_t n_voxels, const int discretize_lower_bound,
      const int discretize_upper_bound, const std::set<uint32_t>& signals,
      const std::vector<double>& normalized_corr) const override;

 private:
  void calculateStartEndNeighbor(
      const uint32_t index, const uint32_t n_corr, uint32_t* start,
      uint32_t* end) const;
  void fitTranslationalGmmDistribution(
      const uint32_t n_voxels, const int discretize_lower_bound,
      const int discretize_upper_bound, const std::set<uint32_t>& signals,
      const std::vector<double>& n_corr, common::GaussianMixturePtr gm) const;
  void retrievePeakNeighbors(
      const uint32_t n_voxels, const int discretize_lower_bound,
      const int discretize_upper_bound, const uint32_t start,
      const uint32_t end, const std::vector<double>& n_corr,
      Eigen::MatrixXd* samples, Eigen::VectorXd* gaussian_weights) const;
};

}  // namespace phaser_core

#endif  // PHASER_BACKEND_UNCERTAINTY_GMM_PEAK_BASED_EVAL_H_
