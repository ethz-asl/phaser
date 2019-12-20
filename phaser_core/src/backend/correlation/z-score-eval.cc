#include "packlo/backend/correlation/z-score-eval.h"
#include "packlo/backend/correlation/signal-analysis.h"

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

void ZScoreEval::evaluateCorrelationFromTranslation(
      const std::vector<double>& corr) {
  std::set<uint32_t> signals;
  std::vector<double> n_corr;

  // Normalize correlation.
  double max = *std::max_element(corr.cbegin(), corr.cend());
  std::transform(
      corr.begin(), corr.end(), std::back_inserter(n_corr),
      [&](const double val) { return val / max; });

  // Filter values close to zero for speedup.
  std::vector<double> n_corr_ds;
  std::copy_if(
      n_corr.cbegin(), n_corr.cend(), std::back_inserter(n_corr_ds),
      std::bind(
          std::greater<double>(), std::placeholders::_1,
          FLAGS_z_score_filter_threshold));

  peak_extraction_.extractPeaks(n_corr_ds, &signals);

  // Evaluate correlation.
  double mean, std;
  std::tie(mean, std) = fitSmoothedNormalDist(signals, n_corr_ds);
  VLOG(1) << "finished z-score with var: " << std * std;
}

std::pair<double, double> ZScoreEval::fitSmoothedNormalDist(
    const std::set<uint32_t>& signals, const std::vector<double>& input) const {
  if (signals.empty()) return std::make_pair(0.0, 0.0);
  std::vector<double> peak_values;
  double max = static_cast<double>(input.size());

  // Normalize peak distances.
  std::transform(
      signals.cbegin(), signals.cend(), std::back_inserter(peak_values),
      [&max](const uint32_t& signal) { 
        return (static_cat<double>(signal)) / max; 
      });

  // Calculate mean distances.
  const uint32_t n_values = peak_values.size();
  const double mean =
      std::accumulate(peak_values.cbegin(), peak_values.cend(), 0.0) / n_values;

  const double std = SignalAnalysis::stdDev(peak_values, mean, 0, n_values);
  return std::make_pair(mean, std);
}

}  // namespace correlation
