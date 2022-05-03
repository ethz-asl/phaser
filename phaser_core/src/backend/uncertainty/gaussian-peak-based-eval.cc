#include "phaser/backend/uncertainty/gaussian-peak-based-eval.h"

#include <glog/logging.h>

#include "phaser/common/signal-utils.h"
#include "phaser/common/translation-utils.h"
#include "phaser/distribution/gaussian.h"

DEFINE_int32(
    gaussian_peak_neighbors, 0,
    "Determines the number of neighbors used for the Bingham calculation.");

namespace phaser_core {

common::BaseDistributionPtr GaussianPeakBasedEval::evaluatePeakBasedCorrelation(
    const uint32_t n_voxels, const int discretize_lower_bound,
    const int discretize_upper_bound, const std::set<uint32_t>& signals,
    const std::vector<double>& norm_corr) const {
  common::GaussianPtr gaussian =
      std::make_shared<common::Gaussian>(fitTranslationalNormalDist(
          n_voxels, discretize_lower_bound, discretize_upper_bound, signals,
          norm_corr));
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
  *end = FLAGS_gaussian_peak_neighbors + index >= n_corr
             ? index
             : index + FLAGS_gaussian_peak_neighbors;
}

common::Gaussian GaussianPeakBasedEval::fitTranslationalNormalDist(
    const uint32_t n_voxels, const int discretize_lower_bound,
    const int discretize_upper_bound, const std::set<uint32_t>& signals,
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
  Eigen::ArrayXXd samples =
      Eigen::MatrixXd::Zero(3, 2 * FLAGS_gaussian_peak_neighbors + 1);
  Eigen::VectorXd weights =
      Eigen::VectorXd::Zero(2 * FLAGS_gaussian_peak_neighbors + 1);
  retrievePeakNeighbors(
      n_voxels, discretize_lower_bound, discretize_upper_bound, start, end,
      norm_corr, &samples, &weights);

  VLOG(1) << "samples:\n" << samples;
  VLOG(1) << "b_weights: " << weights.transpose();

  // Calculate mean and covariance.
  return common::Gaussian(samples, weights);
}

void GaussianPeakBasedEval::retrievePeakNeighbors(
    const uint32_t n_voxels, const int discretize_lower_bound,
    const int discretize_upper_bound, const uint32_t start, const uint32_t end,
    const std::vector<double>& norm_corr, Eigen::ArrayXXd* samples,
    Eigen::VectorXd* weights) const {
  VLOG(1) << "Checking neighbors from " << start << " to " << end;

  // Extract translational estimates.
  uint32_t k = 0u;
  for (uint32_t i = start; i <= end; ++i) {
    std::array<uint32_t, 3> xyz =
        common::SignalUtils::Ind2Sub(i, n_voxels, n_voxels);
    (*samples)(0, k) = common::TranslationUtils::ComputeTranslationFromIndex(
        static_cast<double>(xyz[0]), n_voxels, discretize_lower_bound,
        discretize_upper_bound);
    (*samples)(1, k) = common::TranslationUtils::ComputeTranslationFromIndex(
        static_cast<double>(xyz[1]), n_voxels, discretize_lower_bound,
        discretize_upper_bound);
    (*samples)(2, k) = common::TranslationUtils::ComputeTranslationFromIndex(
        static_cast<double>(xyz[2]), n_voxels, discretize_lower_bound,
        discretize_upper_bound);
    (*weights)(k) = norm_corr.at(i);
    ++k;
  }
  const double weight_sum = weights->array().sum();
  CHECK_GT(weight_sum, 0);
  (*weights) = weights->array() / weight_sum;
}

}  // namespace phaser_core
