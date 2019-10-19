#include "packlo/backend/correlation/z-score-eval.h"

#include <cmath>
#include <cstdint>
#include <numeric>
#include <algorithm>

#include <glog/logging.h>

DEFINE_double(z_score_lag, 3500, 
    "The window used for smoothing the function.");
DEFINE_double(z_score_threshold, 12.5, 
    "Defines the number of n-std requires to include a signal.");
DEFINE_double(z_score_influence, 0.2, 
    "The influence of the current data point to the lag mean.");

namespace correlation {

ZScoreEval::ZScoreEval() {
  CHECK_GT(FLAGS_z_score_lag, 0);
  CHECK_GT(FLAGS_z_score_threshold, 0);
  CHECK_GE(FLAGS_z_score_influence, 0);
  CHECK_LE(FLAGS_z_score_influence, 1.0);
}

void ZScoreEval::evaluateCorrelationFromTranslation(
      const std::vector<double>& corr) {
    std::vector<bool> signals(corr.size(), false);
    calculateSmoothedZScore(corr, &signals);
}

void ZScoreEval::calculateSmoothedZScore(const std::vector<double>& input, 
    std::vector<bool>* signals) const {
  const uint32_t n_input = input.size();
  if (n_input <= FLAGS_z_score_lag + 2) return;

  std::vector<double> filteredY(n_input, 0.0);
  std::vector<double> avgFilter(n_input, 0.0);
  std::vector<double> stdFilter(n_input, 0.0);
  const std::vector<double> subVecStart(input.begin(), 
      input.begin() + FLAGS_z_score_lag);
  const double m = mean(subVecStart);
  avgFilter[FLAGS_z_score_lag] = m;
  stdFilter[FLAGS_z_score_lag] = stdDev(m, subVecStart);

  const double invInfluence = 1.0 - FLAGS_z_score_influence;
  for (uint32_t i = FLAGS_z_score_lag + 1u; i < n_input; ++i)
  {
    const double currentinput = input[i];
    if (std::abs(currentinput - avgFilter[i - 1]) 
        > FLAGS_z_score_threshold * stdFilter[i - 1]) {
      if (currentinput > avgFilter[i - 1]) (*signals)[i] = true; 
      
      // Update influence with current data point.
      filteredY[i] = FLAGS_z_score_influence * currentinput 
        + invInfluence * filteredY[i - 1];
    }
    else 
      filteredY[i] = currentinput;

    // Adjust the filters.
    std::vector<double> subVec(filteredY.begin() + i - FLAGS_z_score_lag,
        filteredY.begin() + i);
    const double m = mean(subVec);
    avgFilter[i] = m;
    stdFilter[i] = stdDev(m, subVec);
  }
}

double ZScoreEval::mean(const std::vector<double>& vec) const {
  double sum = std::accumulate(std::begin(vec), std::end(vec), 0.0);
  return sum / static_cast<double>(vec.size());
}

double ZScoreEval::stdDev(const double mean, 
    const std::vector<double>& vec) const {
  double accum = 0.0;
  std::for_each (std::begin(vec), std::end(vec), [&](const double val) {
    accum += (val - mean) * (val - mean);
  });
  return sqrt(accum / static_cast<double>(vec.size()-1u));
}
}
