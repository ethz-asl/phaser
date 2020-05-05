#include "phaser/backend/uncertainty/z-score-eval.h"
#include "phaser/backend/alignment/phase-aligner.h"
#include "phaser/backend/uncertainty/signal-analysis.h"
#include "phaser/distribution/bingham.h"
#include "phaser/distribution/gaussian.h"

#include "phaser/visualization/plotty-visualizer.h"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <functional>
#include <numeric>
#include <sstream>

#include <glog/logging.h>

namespace uncertainty {

ZScoreEval::ZScoreEval() : manager_("z-score") {}

common::BaseDistributionPtr ZScoreEval::evaluateCorrelationFromTranslation(
    const uint32_t n_voxels, const int discretize_lower_bound,
    const int discretize_upper_bound, const std::vector<double>& corr) {
  std::set<uint32_t> signals;
  std::vector<double> n_corr_ds;
  evaluateCorrelationVector(corr, &signals, &n_corr_ds);
  return evaluatePeakBasedCorrelation(
      n_voxels, discretize_lower_bound, discretize_upper_bound, signals,
      n_corr_ds);
}

common::BaseDistributionPtr ZScoreEval::evaluateCorrelationFromRotation(
    const uint32_t bw, const std::vector<double>& corr) {
  std::set<uint32_t> signals;
  std::vector<double> n_corr_ds;
  evaluateCorrelationVector(corr, &signals, &n_corr_ds);
  return evaluatePeakBasedCorrelation(bw, signals, n_corr_ds);
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

ZScorePeakExtraction& ZScoreEval::getPeakExtraction() {
  return peak_extraction_;
}

common::BaseDistributionPtr ZScoreEval::evaluatePeakBasedCorrelation(
    const uint32_t bw, const std::set<uint32_t>& signals,
    const std::vector<double>& normalized_corr) const {
  LOG(FATAL)
      << "Peak based eval using bw is not implemented for this correlation.";
}

common::BaseDistributionPtr ZScoreEval::evaluatePeakBasedCorrelation(
    const uint32_t n_voxels, const int discretize_lower_bound,
    const int discretize_upper_bound, const std::set<uint32_t>& signals,
    const std::vector<double>& normalized_corr) const {
  LOG(FATAL) << "Peak based eval using voxels and bounds is not implemented "
                "for this correlation.";
}

}  // namespace uncertainty
