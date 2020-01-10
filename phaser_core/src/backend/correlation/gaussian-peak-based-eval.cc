#include "packlo/backend/correlation/gaussian-peak-based-eval.h"
#include "packlo/backend/alignment/phase-aligner.h"
#include "packlo/distribution/gaussian.h"

#include <glog/logging.h>

DEFINE_int32(
    gaussian_peak_neighbors, 5,
    "Determines the number of neighbors used for the Bingham calculation.");

namespace correlation {

GaussianPeakBasedEval::GaussianPeakBasedEval(
    const alignment::BaseAligner& aligner,
    const backend::SphericalCorrelation& sph)
    : ZScoreEval(aligner, sph) {}

common::BaseDistributionPtr GaussianPeakBasedEval::evaluatePeakBasedCorrelation(
    const alignment::BaseAligner& aligner,
    const backend::SphericalCorrelation& sph, const std::set<uint32_t>& signals,
    const std::vector<double>& norm_corr) const {
  common::GaussianPtr gaussian = std::make_shared<common::Gaussian>(
      fitTranslationalNormalDist(aligner, signals, norm_corr));
  return gaussian;
}

void GaussianPeakBasedEval::calculateStartEndNeighbor(
    const uint32_t index, const uint32_t n_corr, uint32_t* start,
    uint32_t* end) const {
  CHECK_NOTNULL(start);
  CHECK_NOTNULL(end);
  *start = FLAGS_gaussian_peak_neighbors > index
               ? index
               : index - FLAGS_gaussian_peak_neighbors;
  *end = FLAGS_gaussian_peak_neighbors + index > n_corr
             ? index
             : index + FLAGS_gaussian_peak_neighbors;
}

common::Gaussian GaussianPeakBasedEval::fitTranslationalNormalDist(
    const alignment::BaseAligner& aligner, const std::set<uint32_t>& signals,
    const std::vector<double>& norm_corr) const {
  const uint32_t n_signals = signals.size();
  const uint32_t n_corr = norm_corr.size();
  CHECK_NE(n_signals, 0);
  VLOG(1) << "Checking " << n_signals << " signals for evaluation";

  // Find the max signal.
  std::set<uint32_t>::iterator max_signal = std::max_element(
      signals.begin(), signals.end(),
      [&norm_corr](const uint32_t lhs, const uint32_t rhs) {
        return norm_corr[lhs] < norm_corr[rhs];
      });
  CHECK(max_signal != signals.end());

  // Retrieve max peak neigbhors.
  uint32_t start, end;
  calculateStartEndNeighbor(*max_signal, n_corr, &start, &end);
  const uint32_t num_elements = end - start + 1u;
  Eigen::ArrayXXd samples = Eigen::MatrixXd::Zero(4, num_elements);
  Eigen::VectorXd weights = Eigen::VectorXd::Zero(num_elements);
  retrievePeakNeighbors(start, end, norm_corr, aligner, &samples, &weights);

  // Calculate mean and covariance.
  return common::Gaussian(samples, weights);
}

void GaussianPeakBasedEval::retrievePeakNeighbors(
    const uint32_t start, const uint32_t end,
    const std::vector<double>& norm_corr, const alignment::BaseAligner& aligner,
    Eigen::ArrayXXd* samples, Eigen::VectorXd* weights) const {
  const alignment::PhaseAligner& phase =
      dynamic_cast<const alignment::PhaseAligner&>(aligner);

  // Extract translational estimates.
  uint32_t k = 0u;
  for (uint32_t i = start; i <= end; ++i) {
    std::array<uint16_t, 3> xyz = phase.ind2sub(i);
    (*samples)(0, k) =
        phase.computeTranslationFromIndex(static_cast<double>(xyz[0]));
    (*samples)(1, k) =
        phase.computeTranslationFromIndex(static_cast<double>(xyz[1]));
    (*samples)(2, k) =
        phase.computeTranslationFromIndex(static_cast<double>(xyz[2]));
    (*weights)(k) = norm_corr.at(i);
    ++k;
  }
  const double weight_sum = weights->array().sum();
  CHECK_GT(weight_sum, 0);
  (*weights) = weights->array() / weight_sum;
}

}  // namespace correlation
