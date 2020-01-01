#include "packlo/backend/correlation/gmm-peak-based-eval.h"
#include "packlo/backend/alignment/phase-aligner.h"

#include <glog/logging.h>

namespace correlation {

DEFINE_int32(
    gmm_peak_neighbors, 2,
    "Determines the number of neighbors used for the GMM calculation.");

common::BaseDistributionPtr GmmPeakBasedEval::evaluatePeakBasedCorrelation(
    const alignment::BaseAligner& aligner, const std::set<uint32_t>& signals,
    const std::vector<double>&) const {}

model::GmmParameters GmmPeakBasedEval::fitTranslationalGmmDistribution(
    const alignment::BaseAligner& aligner,
    const std::set<uint32_t>& signals) const {
  const alignment::PhaseAligner& phase =
      dynamic_cast<const alignment::PhaseAligner&>(aligner);
  const uint32_t n_signals = signals.size();
  if (n_signals == 0) {
    return model::GmmParameters();
  }
  Eigen::ArrayXXd samples(3, n_signals);
  uint32_t i = 0u;
}

void GmmPeakBasedEval::retrievePeakNeighbors(
    const uint32_t index, Eigen::ArrayXXd* samples,
    Eigen::VectorXd* gaussian_weights) const {}

}  // namespace correlation
