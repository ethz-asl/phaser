#include "packlo/backend/correlation/z-score-eval.h"
#include "packlo/backend/alignment/phase-aligner.h"
#include "packlo/backend/correlation/signal-analysis.h"
 #include "packlo/distribution/bingham.h"
 #include "packlo/distribution/gaussian.h" 

#include "packlo/visualization/plotty-visualizer.h"

#include <cmath>
#include <numeric>
#include <algorithm>
#include <functional>
#include <sstream>
#include <fstream>

#include <glog/logging.h>

namespace correlation {

ZScoreEval::ZScoreEval(
    const alignment::BaseAligner& aligner,
    const backend::SphericalCorrelation& sph)
    : manager_("z-score"), BaseEval(aligner, sph) {}

common::BaseDistributionPtr ZScoreEval::evaluateCorrelation(
    const alignment::BaseAligner& aligner,
    const backend::SphericalCorrelation&) {
  return evaluateCorrelationFromTranslation();
}

static std::vector<Eigen::MatrixXd> trans_samples_;
static std::vector<Eigen::MatrixXd> trans_weights_;
common::BaseDistributionPtr ZScoreEval::evaluateCorrelationFromTranslation() {
  std::set<uint32_t> signals;
  std::vector<double> n_corr_ds;
  evaluateCorrelationVector(aligner_.getCorrelation(), &signals, &n_corr_ds);
  common::BaseDistributionPtr dist 
    = evaluatePeakBasedCorrelation(aligner_, sph_, signals, n_corr_ds);

  common::GaussianPtr g = std::dynamic_pointer_cast<common::Gaussian>(dist);
  trans_samples_.emplace_back(g->samples_);
  trans_weights_.emplace_back(g->weights_);

  std::fstream fs; 
  fs.open("trans_samples.txt", std::fstream::out);
  for (auto& sample : trans_samples_) {
    fs << sample << "\n";
  }
  fs.close();
  fs.open("trans_weights.txt", std::fstream::out);
  for (auto& weights : trans_weights_) {
    fs << weights << "\n";
  }

  return dist;
}

static std::vector<Eigen::MatrixXd> rot_samples_;
static std::vector<Eigen::MatrixXd> rot_weights_;
static uint32_t counter = 0u;
common::BaseDistributionPtr ZScoreEval::evaluateCorrelationFromRotation() {
  std::set<uint32_t> signals;
  std::vector<double> n_corr_ds;
  evaluateCorrelationVector(sph_.getCorrelation(), &signals, &n_corr_ds);
  common::BaseDistributionPtr dist 
    = evaluatePeakBasedCorrelation(aligner_, sph_, signals, n_corr_ds);

  // =============================== DEBUG 
  ++counter;
  common::BinghamPtr b = std::dynamic_pointer_cast<common::Bingham>(dist);
  VLOG(1) << "======== TRACES: ";                                               
  rot_samples_.emplace_back(b->samples_);
  rot_weights_.emplace_back(b->weights_);
  //if (counter > 115 != 0u) return dist; 

  std::fstream fs; 
  fs.open("rot_samples.txt", std::fstream::out);
  for (auto& sample : rot_samples_) {
    fs << sample << "\n";
  }
  fs.close();
  fs.open("rot_weights.txt", std::fstream::out);
  for (auto& weights : rot_weights_) {
    fs << weights << "\n";
  }
  VLOG(1) << "====================================================";
  // =====================================
  

  return dist;
}

void ZScoreEval::evaluateCorrelationVector(
    const std::vector<double>& corr, std::set<uint32_t>* signals,
    std::vector<double>* n_corr_ds) {
  // std::vector<double> n_corr;

  // Normalize correlation.
  auto max = std::max_element(corr.cbegin(), corr.cend());
  CHECK(max != corr.cend());
  /*
  std::transform(
      corr.begin(), corr.end(), std::back_inserter(*n_corr_ds),
      [&](const double val) { return val / *max; });
      */
  VLOG(1) << "max is at: " << *max;
  signals->insert(std::distance(corr.begin(), max));
  *n_corr_ds = corr;

  // Filter values close to zero for speedup.
  /*
  std::copy_if(
      n_corr.cbegin(), n_corr.cend(), std::back_inserter(*n_corr_ds),
      std::bind(
          std::greater<double>(), std::placeholders::_1,
          FLAGS_z_score_filter_threshold));
  */

  // visualization::PlottyVisualizer::getInstance().createPlotFor(*n_corr_ds);

  // peak_extraction_.extractPeaks(*n_corr_ds, signals);
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
  if (n_signals == 0) {
    return std::make_pair(
        Eigen::VectorXd::Zero(3), Eigen::MatrixXd::Identity(3, 3));
  }
  Eigen::ArrayXXd samples(3, n_signals);
  uint32_t i = 0u;

  // Extract translational estimates.
  for (uint32_t signal_idx : signals) {
    std::array<uint32_t, 3> xyz = phase.ind2sub(signal_idx);
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

ZScorePeakExtraction& ZScoreEval::getPeakExtraction() {
  return peak_extraction_;
}

void ZScoreEval::evaluateCorrelationFromRotation(
    const std::vector<double>& corr) {}

}  // namespace correlation
