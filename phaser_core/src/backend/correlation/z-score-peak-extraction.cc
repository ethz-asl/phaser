#include "packlo/backend/correlation/z-score-peak-extraction.h"
#include "packlo/backend/correlation/signal-analysis.h"

#include <algorithm>
#include <cmath>
#include <functional>
#include <numeric>

#include <glog/logging.h>

DEFINE_double(
    z_score_lag_percentile, 0.05,
    "The window used for smoothing the function.");
DEFINE_double(
    z_score_threshold, 4.66,
    "Defines the number of n-std requires to include a signal.");
DEFINE_double(
    z_score_influence, 0.05,
    "The influence of the current data point to the lag mean.");

namespace correlation {

ZScorePeakExtraction::ZScorePeakExtraction() : manager_("z-score") {
  CHECK_GT(FLAGS_z_score_lag_percentile, 0);
  CHECK_LT(FLAGS_z_score_lag_percentile, 1);
  CHECK_GT(FLAGS_z_score_threshold, 0);
  CHECK_GE(FLAGS_z_score_influence, 0);
  CHECK_LE(FLAGS_z_score_influence, 1.0);
}

void ZScorePeakExtraction::extractPeaks(
    const std::vector<double>& corr, std::set<uint32_t>* peaks) {
  const uint32_t lag = corr.size() * FLAGS_z_score_lag_percentile;
  VLOG(1) << "Calculating z-scores using a " << lag << " window.";
  std::vector<double> input(corr);
  calculateSmoothedZScore(
      &input, lag, FLAGS_z_score_threshold, FLAGS_z_score_influence, peaks);
  for (auto& signal : *peaks) {
    VLOG(1) << "peak at " << signal;
  }
}

void ZScorePeakExtraction::calculateSmoothedZScore(
    std::vector<double>* input, const double lag, const double threshold,
    const double influence, std::set<uint32_t>* signals) const {
  signals->clear();
  const uint32_t n_input = input->size();
  if (n_input <= lag + 2)
    return;
  std::vector<double> avgFilter(n_input, 0.0);
  std::vector<double> stdFilter(n_input, 0.0);

  // Calculate initial mean and covariance.
  const double m =
      std::accumulate(input->cbegin(), input->cend() + lag, 0.0) / lag;
  avgFilter[lag] = m;
  stdFilter[lag] = SignalAnalysis::stdDev(*input, m, 0, lag);

  // windowed iteration over the data points.
  const double invInfluence = 1.0 - influence;
  for (uint32_t i = lag + 1u; i < n_input; ++i) {
    const double currentinput = (*input)[i];
    const double prev_mean = avgFilter[i - 1];
    if (std::abs(currentinput - prev_mean) > threshold * stdFilter[i - 1]) {
      if (currentinput > prev_mean)
        signals->insert(i);

      // Update influence with current data point.
      (*input)[i] = influence * currentinput + invInfluence * (*input)[i - 1];
    }

    // Adjust the filters.
    avgFilter[i] =
        std::accumulate(input->cbegin() + (i - lag), input->cbegin() + i, 0.0) /
        lag;
    stdFilter[i] = SignalAnalysis::stdDev(*input, avgFilter[i], i - lag, i);
  }
}

}  // namespace correlation