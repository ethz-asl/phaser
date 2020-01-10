#include "packlo/backend/correlation/bingham-peak-based-eval.h"
#include "packlo/common/rotation-utils.h"

#include <algorithm>
#include <glog/logging.h>

DEFINE_int32(
    bingham_peak_neighbors, 10,
    "Determines the number of neighbors used for the Bingham calculation.");

namespace correlation {

BinghamPeakBasedEval::BinghamPeakBasedEval(
    const alignment::BaseAligner& aligner,
    const backend::SphericalCorrelation& sph)
    : ZScoreEval(aligner, sph) {}

common::BaseDistributionPtr BinghamPeakBasedEval::evaluateCorrelation(
    const alignment::BaseAligner& aligner,
    const backend::SphericalCorrelation&) {
  return evaluateCorrelationFromTranslation();
}

common::BaseDistributionPtr BinghamPeakBasedEval::evaluatePeakBasedCorrelation(
    const alignment::BaseAligner& aligner,
    const backend::SphericalCorrelation& sph, const std::set<uint32_t>& signals,
    const std::vector<double>& normalized_corr) const {
  common::BinghamPtr bingham = std::make_shared<common::Bingham>(
      fitRotationalBinghamDistribution(sph, signals, normalized_corr));
  return bingham;
}

common::Bingham BinghamPeakBasedEval::fitRotationalBinghamDistribution(
    const backend::SphericalCorrelation& sph, const std::set<uint32_t>& signals,
    const std::vector<double>& norm_corr) const {
  const uint32_t n_signals = signals.size();
  const uint32_t n_corr = norm_corr.size();
  CHECK_NE(n_signals, 0);
  VLOG(1) << "Checking " << n_signals << " signals for evaluation";
  std::set<uint32_t>::iterator max_signal =
      std::max_element(signals.begin(), signals.end(),
      [&norm_corr](const uint32_t lhs, const uint32_t rhs) {
        return norm_corr[lhs] < norm_corr[rhs];
      });
  CHECK(max_signal != signals.end());

  uint32_t start, end;
  calculateStartEndNeighbor(*max_signal, n_corr, &start, &end);
  const uint32_t num_elements = end - start + 1u;
  Eigen::MatrixXd samples = Eigen::MatrixXd::Zero(4, num_elements);
  Eigen::RowVectorXd weights = Eigen::RowVectorXd::Zero(num_elements);
  retrievePeakNeighbors(start, end, norm_corr, sph, &samples, &weights);
  //VLOG(1) << "bingham samples:\n" << samples << "\nweights:\n" << weights;
  return common::Bingham::fit(samples, weights);
}

void BinghamPeakBasedEval::calculateStartEndNeighbor(
    const uint32_t index, const uint32_t n_corr, uint32_t* start,
    uint32_t* end) const {
  CHECK_NOTNULL(start);
  CHECK_NOTNULL(end);
  *start = FLAGS_bingham_peak_neighbors > index
               ? index
               : index - FLAGS_bingham_peak_neighbors;
  *end = FLAGS_bingham_peak_neighbors + index > n_corr
             ? index
             : index + FLAGS_bingham_peak_neighbors;
}

void BinghamPeakBasedEval::retrievePeakNeighbors(
    const uint32_t start, const uint32_t end,
    const std::vector<double>& norm_corr,
    const backend::SphericalCorrelation& sph, Eigen::MatrixXd* samples,
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
    std::array<double, 3> zyz = sph.getZYZFromIndex(i);
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
  VLOG(1) << "bingham weights: \n" << (*weights);
}

}  // namespace correlation