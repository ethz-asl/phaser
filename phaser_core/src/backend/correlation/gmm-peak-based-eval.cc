#include "packlo/backend/correlation/gmm-peak-based-eval.h"
#include "packlo/backend/alignment/phase-aligner.h"
#include "packlo/backend/correlation/gaussian-peak-based-eval.h"
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
  common::GaussianMixturePtr gmm = std::make_shared<common::GaussianMixture>();
  fitTranslationalGmmDistribution(aligner, signals, n_corr, gmm);
  return gmm;
}

void GmmPeakBasedEval::fitTranslationalGmmDistribution(
    const alignment::BaseAligner& aligner, const std::set<uint32_t>& signals,
    const std::vector<double>& norm_corr, common::GaussianMixturePtr gm) const {
  const uint32_t n_signals = signals.size();
  const uint32_t n_corr = norm_corr.size();
  CHECK_GT(n_signals, 0);
  VLOG(1) << "Checking " << n_signals << " signals for evaluation";
  std::vector<common::Gaussian> peak_gaussians;
  Eigen::VectorXd gm_weights = Eigen::VectorXd::Zero(n_signals);
  uint32_t start, end;
  for (uint32_t i = 0; i < n_signals; ++i) {
    calculateStartEndNeighbor(i, n_corr, &start, &end);
    const uint32_t num_elements = end - start + 1;
    Eigen::MatrixXd samples = Eigen::MatrixXd::Zero(3, num_elements);
    Eigen::VectorXd weights = Eigen::VectorXd::Zero(num_elements);

    retrievePeakNeighbors(start, end, norm_corr, aligner, &samples, &weights);
    peak_gaussians.emplace_back(common::Gaussian(samples, weights));
    gm_weights(i) = norm_corr.at(i);
  }
  gm_weights = gm_weights.array() / gm_weights.array().sum();
  // auto test = std::make_shared<common::GaussianMixture>(peak_gaussians,
  // gm_weights);
  gm->initializeFromGaussians(peak_gaussians, gm_weights);
}
void GmmPeakBasedEval::calculateStartEndNeighbor(
    const uint32_t index, const uint32_t n_corr, uint32_t* start,
    uint32_t* end) const {
  *start = FLAGS_gmm_peak_neighbors > index ? index
                                            : index - FLAGS_gmm_peak_neighbors;
  *end = FLAGS_gmm_peak_neighbors + index > n_corr
             ? index
             : index + FLAGS_gmm_peak_neighbors;
}

void GmmPeakBasedEval::retrievePeakNeighbors(
    const uint32_t start, const uint32_t end,
    const std::vector<double>& norm_corr, const alignment::BaseAligner& aligner,
    Eigen::MatrixXd* samples, Eigen::VectorXd* gaussian_weights) const {
  CHECK_NOTNULL(samples);
  CHECK_NOTNULL(gaussian_weights);
  const uint32_t n_signals = norm_corr.size();
  const alignment::PhaseAligner& phase =
      dynamic_cast<const alignment::PhaseAligner&>(aligner);

  CHECK_GT(n_signals, 0);
  CHECK_GE(start, 0);
  CHECK_LT(end, n_signals);
  CHECK_LE(start, end);

  const uint32_t num_elements = end - start + 1;
  Eigen::VectorXd weights = Eigen::VectorXd::Zero(num_elements);
  VLOG(1) << "Checking neighbors from " << start << " to " << end;
  std::size_t k = 0;
  for (uint32_t i = start; i <= end; ++i) {
    std::array<uint16_t, 3> xyz = phase.ind2sub(i);
    (*samples)(0, k) =
        phase.computeTranslationFromIndex(static_cast<double>(xyz[0]));
    (*samples)(1, k) =
        phase.computeTranslationFromIndex(static_cast<double>(xyz[1]));
    (*samples)(2, k) =
        phase.computeTranslationFromIndex(static_cast<double>(xyz[2]));
    (*gaussian_weights)(k) = norm_corr.at(i);
    ++k;
  }
  const double weight_sum = gaussian_weights->array().sum();
  CHECK_GT(weight_sum, 0);
  (*gaussian_weights) = gaussian_weights->array() / weight_sum;
  VLOG(1) << "gaussian weights: \n" << (*gaussian_weights);
}

}  // namespace correlation
