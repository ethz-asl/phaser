#include "packlo/backend/correlation/z-score-eval.h"
#include "packlo/backend/alignment/phase-aligner.h"
#include "packlo/backend/correlation/signal-analysis.h"
#include "packlo/distribution/gaussian.h"

#include <cmath>
#include <numeric>
#include <algorithm>
#include <functional>

#include <glog/logging.h>

// Good values: 500, 5.75, 0.05
//              70, 3.75, 0.05
//              210, 5.66, 0.05

DEFINE_double(
    z_score_filter_threshold, 0.105,
    "Removes all correlation input below this value.");

namespace correlation {

ZScoreEval::ZScoreEval() : manager_("z-score") {}

common::BaseDistributionPtr ZScoreEval::evaluateCorrelation(
    const alignment::BaseAligner& aligner,
    const backend::SphericalCorrelation&) {
  return evaluateCorrelationFromTranslation(aligner);
}

common::BaseDistributionPtr ZScoreEval::evaluateCorrelationFromTranslation(
    const alignment::BaseAligner& aligner) {
  std::set<uint32_t> signals;
  std::vector<double> n_corr_ds;
  evaluateCorrelationVector(aligner.getCorrelation(), &signals, &n_corr_ds);

  // Evaluate correlation.
  Eigen::VectorXd mean;
  Eigen::MatrixXd cov;
  std::tie(mean, cov) = fitTranslationalNormalDist(aligner, signals);
  return std::make_shared<common::Gaussian>(std::move(mean), std::move(cov));
}

void ZScoreEval::evaluateCorrelationVector(
    const std::vector<double>& corr, std::set<uint32_t>* signals,
    std::vector<double>* n_corr_ds) {
  std::vector<double> n_corr;

  // Normalize correlation.
  double max = *std::max_element(corr.cbegin(), corr.cend());
  std::transform(
      corr.begin(), corr.end(), std::back_inserter(n_corr),
      [&](const double val) { return val / max; });

  // Filter values close to zero for speedup.
  std::copy_if(
      n_corr.cbegin(), n_corr.cend(), std::back_inserter(*n_corr_ds),
      std::bind(
          std::greater<double>(), std::placeholders::_1,
          FLAGS_z_score_filter_threshold));

  peak_extraction_.extractPeaks(*n_corr_ds, signals);
}

std::pair<double, double> ZScoreEval::fitSmoothedNormalDist(
    const std::set<uint32_t>& signals, const std::vector<double>& input) const {
  if (signals.empty()) return std::make_pair(0.0, 0.0);
  std::vector<double> peak_values;
  const double max = static_cast<double>(input.size());

  // Normalize peak distances.
  std::transform(
      signals.cbegin(), signals.cend(), std::back_inserter(peak_values),
      [&max](const uint32_t& signal) {
        return (static_cast<double>(signal)) / max;
      });

  // Calculate mean distances.
  const uint32_t n_values = peak_values.size();
  const double mean =
      std::accumulate(peak_values.cbegin(), peak_values.cend(), 0.0) / n_values;

  const double std = SignalAnalysis::stdDev(peak_values, mean, 0, n_values);
  return std::make_pair(mean, std);
}

std::pair<Eigen::VectorXd, Eigen::MatrixXd>
ZScoreEval::fitTranslationalNormalDist(
    const alignment::BaseAligner& aligner,
    const std::set<uint32_t>& signals) const {
  const alignment::PhaseAligner& phase =
      dynamic_cast<const alignment::PhaseAligner&>(aligner);
  const uint32_t n_signals = signals.size();
  Eigen::ArrayXXd samples(3, n_signals);
  uint32_t i = 0u;

  // Extract translational estimates.
  for (uint32_t signal_idx : signals) {
    std::array<uint16_t, 3> xyz = phase.ind2sub(signal_idx);
    samples(0, i) =
        phase.computeTranslationFromIndex(static_cast<double>(xyz[0]));
    samples(1, i) =
        phase.computeTranslationFromIndex(static_cast<double>(xyz[1]));
    samples(2, i) =
        phase.computeTranslationFromIndex(static_cast<double>(xyz[2]));
    ++i;
  }
  // Calculate mean and covariance.
  Eigen::VectorXd mean = samples.rowwise().mean();
  Eigen::VectorXd var =
      ((samples.colwise() - mean.array()).square().rowwise().sum() /
       (n_signals - 1));
  return std::make_pair(mean, var.asDiagonal());
}

void ZScoreEval::evaluateCorrelationFromRotation(
    const std::vector<double>& corr) {}

}  // namespace correlation
