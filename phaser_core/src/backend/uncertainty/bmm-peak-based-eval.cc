#include "phaser/backend/uncertainty/bmm-peak-based-eval.h"

#include <glog/logging.h>

#include "phaser/common/rotation-utils.h"

DEFINE_int32(
    bmm_peak_neighbors, 2,
    "Determines the number of neighbors used for the GMM calculation.");

namespace phaser_core {

common::BaseDistributionPtr BmmPeakBasedEval::evaluatePeakBasedCorrelation(
    const uint32_t bw, const std::set<uint32_t>& signals,
    const std::vector<double>& n_corr) const {
  return fitRotationalBinghamDistribution(bw, signals, n_corr);
}

common::BinghamMixturePtr BmmPeakBasedEval::fitRotationalBinghamDistribution(
    const uint32_t bw, const std::set<uint32_t>& signals,
    const std::vector<double>& norm_corr) const {
  const uint32_t n_signals = signals.size();
  const uint32_t n_corr = norm_corr.size();
  CHECK_GT(n_signals, 0);
  VLOG(1) << "Checking " << n_signals << " signals for evaluation";
  std::vector<common::Bingham> peak_binghams;
  Eigen::VectorXd bm_weights = Eigen::VectorXd::Zero(n_signals);
  uint32_t start, end, k = 0;
  // for (uint32_t i = 0u; i < n_signals; ++i) {
  for (uint32_t i : signals) {
    calculateStartEndNeighbor(i, n_corr, &start, &end);
    const uint32_t num_elements = end - start + 1;

    Eigen::MatrixXd samples = Eigen::MatrixXd::Zero(4, num_elements);
    Eigen::RowVectorXd weights = Eigen::RowVectorXd::Zero(num_elements);
    retrievePeakNeighbors(bw, start, end, norm_corr, &samples, &weights);
    peak_binghams.emplace_back(common::Bingham::fit(samples, weights));

    bm_weights(k) = norm_corr.at(i);
    ++k;
  }
  bm_weights = bm_weights.array() / bm_weights.array().sum();
  return std::make_shared<common::BinghamMixture>(peak_binghams, bm_weights);
}

void BmmPeakBasedEval::calculateStartEndNeighbor(
    const uint32_t index, const uint32_t n_corr, uint32_t* start,
    uint32_t* end) const {
  CHECK_NOTNULL(start);
  CHECK_NOTNULL(end);
  *start = FLAGS_bmm_peak_neighbors > index ? index
                                            : index - FLAGS_bmm_peak_neighbors;
  *end = FLAGS_bmm_peak_neighbors + index > n_corr
             ? index
             : index + FLAGS_bmm_peak_neighbors;
}

void BmmPeakBasedEval::retrievePeakNeighbors(
    const uint32_t bw, const uint32_t start, const uint32_t end,
    const std::vector<double>& norm_corr, Eigen::MatrixXd* samples,
    Eigen::RowVectorXd* weights) const {
  CHECK_NOTNULL(samples);
  CHECK_NOTNULL(weights);
  const uint32_t n_signals = norm_corr.size();

  CHECK_GT(n_signals, 0u);
  CHECK_GE(start, 0u);
  CHECK_LT(end, n_signals);
  CHECK_LE(start, end);

  VLOG(1) << "Checking neighbors from " << start << " to " << end;
  std::size_t k = 0u;
  for (uint32_t i = start; i <= end; ++i) {
    std::array<double, 3> zyz = common::RotationUtils::GetZYZFromIndex(i, bw);
    Eigen::Quaterniond q = common::RotationUtils::ConvertZYZtoQuaternion(zyz);
    (*samples)(0, k) = q.w();
    (*samples)(1, k) = q.x();
    (*samples)(2, k) = q.y();
    (*samples)(3, k) = q.z();
    (*weights)(k) = norm_corr.at(i);
    ++k;
  }
  const double weight_sum = weights->array().sum();
  CHECK_GT(weight_sum, 0);
  (*weights) = weights->array() / weight_sum;
}

}  // namespace phaser_core
