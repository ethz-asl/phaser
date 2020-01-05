#include "packlo/backend/correlation/gmm-peak-based-eval.h"
#include "packlo/backend/alignment/phase-aligner.h"
#include "packlo/distribution/gaussian.h"

#include <glog/logging.h>
#include <vector>

namespace correlation {

DEFINE_int32(
    gmm_peak_neighbors, 2,
    "Determines the number of neighbors used for the GMM calculation.");

common::BaseDistributionPtr GmmPeakBasedEval::evaluatePeakBasedCorrelation(
    const alignment::BaseAligner& aligner, const std::set<uint32_t>& signals,
    const std::vector<double>& n_corr) const {
  common::GaussianMixturePtr gmm = std::make_shared<common::GaussianMixture>(
      fitTranslationalGmmDistribution(aligner, signals, n_corr));
  return std::dynamic_pointer_cast<common::BaseDistribution>(gmm);
}

common::GaussianMixture GmmPeakBasedEval::fitTranslationalGmmDistribution(
    const alignment::BaseAligner& aligner, const std::set<uint32_t>& signals,
    const std::vector<double>& n_corr) const {
  const uint32_t n_signals = signals.size();
  CHECK_GT(n_signals, 0);

  std::vector<common::Gaussian> peak_gaussians;
  Eigen::VectorXd gm_weights = Eigen::VectorXd::Zero(n_signals);
  for (uint32_t i = 0; i < n_signals; ++i) {
    Eigen::MatrixXd samples =
        Eigen::MatrixXd::Zero(3, 2 * FLAGS_gmm_peak_neighbors + 1);
    Eigen::VectorXd weights =
        Eigen::VectorXd::Zero(2 * FLAGS_gmm_peak_neighbors + 1);
    retrievePeakNeighbors(i, n_corr, aligner, &samples, &weights);
    common::Gaussian gauss(samples, weights);
    peak_gaussians.emplace_back(gauss);
    gm_weights(i) = n_corr.at(i);
  }
  gm_weights = gm_weights.array() / gm_weights.array().sum();
  return common::GaussianMixture(peak_gaussians, gm_weights);
}

void GmmPeakBasedEval::retrievePeakNeighbors(
    const uint32_t index, const std::vector<double>& n_corr,
    const alignment::BaseAligner& aligner,
    Eigen::MatrixXd* samples,
    Eigen::VectorXd* gaussian_weights) const {
  const uint32_t n_signals = n_corr.size();
  const uint32_t start = FLAGS_gmm_peak_neighbors > index
                             ? index
                             : index - FLAGS_gmm_peak_neighbors;
  const uint32_t end = FLAGS_gmm_peak_neighbors + index > n_signals
                           ? index
                           : index + FLAGS_gmm_peak_neighbors;
  const alignment::PhaseAligner& phase =
      dynamic_cast<const alignment::PhaseAligner&>(aligner);

  CHECK_GT(n_signals, 0);

  CHECK_EQ(samples->cols(), 2*FLAGS_gmm_peak_neighbors + 1);
  CHECK_EQ(gaussian_weights->rows(), 2*FLAGS_gmm_peak_neighbors + 1);
  CHECK_GE(start, 0);
  CHECK_LT(end, n_signals);
  CHECK_LE(start, end);
  VLOG(1) << "Checking neighbors from " << start << " to " << end;
  for (uint32_t i = start; i <= end; ++i) {
    std::array<uint16_t, 3> xyz = phase.ind2sub(i);
    (*samples)(0, i) =
        phase.computeTranslationFromIndex(static_cast<double>(xyz[0]));
    (*samples)(1, i) =
        phase.computeTranslationFromIndex(static_cast<double>(xyz[1]));
    (*samples)(2, i) =
        phase.computeTranslationFromIndex(static_cast<double>(xyz[2]));
    (*gaussian_weights)(i) = n_corr.at(i);
  }
  VLOG(1) << "gaussian weights: \n" << (*gaussian_weights);
  (*gaussian_weights) =
      gaussian_weights->array() / gaussian_weights->array().sum();
}

}  // namespace correlation
