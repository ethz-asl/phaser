#include "packlo/backend/correlation/gmm-peak-based-eval.h"
#include "packlo/backend/alignment/phase-aligner.h"

#include <glog/logging.h>

namespace correlation {

DEFINE_int32(
    gmm_peak_neighbors, 2,
    "Determines the number of neighbors used for the GMM calculation.");

common::BaseDistributionPtr GmmPeakBasedEval::evaluatePeakBasedCorrelation(
    const alignment::BaseAligner& aligner, const std::set<uint32_t>& signals,
    const std::vector<double>& n_corr) const {}

model::GmmParameters GmmPeakBasedEval::fitTranslationalGmmDistribution(
    const alignment::BaseAligner& aligner,
    const std::set<uint32_t>& signals,
    const std::vector<double>& n_corr) const {
  const alignment::PhaseAligner& phase =
      dynamic_cast<const alignment::PhaseAligner&>(aligner);
  const uint32_t n_signals = signals.size();
  if (n_signals == 0) {
    return model::GmmParameters();
  }
  for (uint32_t i = 0; i < n_signals; ++i) {
    Eigen::MatrixXd samples(3, 2 * FLAGS_gmm_peak_neighbors + 1);
    Eigen::VectorXd weights(2 * FLAGS_gmm_peak_neighbors + 1);
    retrievePeakNeighbors(i, n_corr, aligner, &samples, &weights);

  }
  uint32_t i = 0u;
}

void GmmPeakBasedEval::retrievePeakNeighbors(
    const uint32_t index, const std::vector<double>& n_corr,
    const alignment::BaseAligner& aligner,
    Eigen::MatrixXd* samples,
    Eigen::VectorXd* gaussian_weights) const {
  const uint32_t start = index - FLAGS_gmm_peak_neighbors;
  const uint32_t end = index + FLAGS_gmm_peak_neighbors;
  const alignment::PhaseAligner& phase =
      dynamic_cast<const alignment::PhaseAligner&>(aligner);

  const uint32_t n_signals = n_corr.size();
  if (n_signals == 0) {
    return;
  }

  CHECK_EQ(samples->cols(), 2*FLAGS_gmm_peak_neighbors + 1);
  CHECK_EQ(gaussian_weights->rows(), 2*FLAGS_gmm_peak_neighbors + 1);
  CHECK(start >= 0);
  CHECK(end < n_signals);
  const uint32_t local_max_energy  = n_corr.at(index);
  for (uint32_t i = start; i < end; ++i) {
    std::array<uint16_t, 3> xyz = phase.ind2sub(i);
    (*samples)(0, i) =
        phase.computeTranslationFromIndex(static_cast<double>(xyz[0]));
    (*samples)(1, i) =
        phase.computeTranslationFromIndex(static_cast<double>(xyz[1]));
    (*samples)(2, i) =
        phase.computeTranslationFromIndex(static_cast<double>(xyz[2]));
    (*gaussian_weights)(i) = n_corr.at(i) / local_max_energy;
  }
}

}  // namespace correlation
