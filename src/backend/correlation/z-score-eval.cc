#include "packlo/backend/correlation/z-score-eval.h"
#include "packlo/visualization/plotty-visualizer.h"

#include <cmath>
#include <numeric>
#include <algorithm>
#include <functional>

#include <glog/logging.h>

DEFINE_double(z_score_lag, 2000, 
    "The window used for smoothing the function.");
DEFINE_double(z_score_threshold, 14.5, 
    "Defines the number of n-std requires to include a signal.");
DEFINE_double(z_score_influence, 0.2, 
    "The influence of the current data point to the lag mean.");

namespace correlation {

ZScoreEval::ZScoreEval() : manager_("z-score") {
  CHECK_GT(FLAGS_z_score_lag, 0);
  CHECK_GT(FLAGS_z_score_threshold, 0);
  CHECK_GE(FLAGS_z_score_influence, 0);
  CHECK_LE(FLAGS_z_score_influence, 1.0);
}

void ZScoreEval::evaluateCorrelationFromTranslation(
      const std::vector<double>& corr) {
    std::vector<uint32_t> signals;
    std::vector<double> corr_ds;
    VLOG(1) << "size before: " << corr.size();
    double max = *std::max_element(corr.cbegin(), corr.cend());

    std::transform(corr.begin(), corr.end(), std::back_inserter(corr_ds),
        [&] (const double val) {return val / max;});

    std::vector<double> n_corr_ds;
    std::copy_if(corr_ds.cbegin(), corr_ds.cend(), std::back_inserter(n_corr_ds), 
        std::bind(std::greater<double>(), std::placeholders::_1, 0.12));


    for (auto& ds : n_corr_ds) {
      manager_.emplaceValue("signal", ds);
    }

    visualization::PlottyVisualizer::getInstance()
     .createPlotFor(manager_, "signal");
    

    VLOG(1) << "size after: " << corr_ds.size();
    calculateSmoothedZScore(n_corr_ds, &signals);

    for (uint32_t peak : signals) {
      VLOG(1) << "Found peak at " << peak 
        << " with the value: " << corr[peak];
    }
    VLOG(1) << "================= done with z-score";
}

void ZScoreEval::calculateSmoothedZScore(const std::vector<double>& input, 
    std::vector<uint32_t>* signals) const {
  const uint32_t n_input = input.size();
  if (n_input <= FLAGS_z_score_lag + 2) return;

  std::vector<double> filteredY = input;
  std::vector<double> avgFilter(n_input, 0.0);
  std::vector<double> stdFilter(n_input, 0.0);
  const double m = std::accumulate(input.cbegin(), 
      input.cend()+FLAGS_z_score_lag, 0.0)
      / FLAGS_z_score_lag;
  //return sum / (to-from);
  //const double m = mean(input, 0, FLAGS_z_score_lag);
  avgFilter[FLAGS_z_score_lag] = m;
  stdFilter[FLAGS_z_score_lag] = stdDev(m, input, 0, FLAGS_z_score_lag);

  const double invInfluence = 1.0 - FLAGS_z_score_influence;
  for (uint32_t i = FLAGS_z_score_lag + 1u; i < n_input; ++i)
  {
    const double currentinput = input[i];
    const double prev_mean = avgFilter[i - 1];
    if (std::abs(currentinput - prev_mean) 
        > FLAGS_z_score_threshold * stdFilter[i - 1]) {
      if (currentinput > prev_mean)
        signals->emplace_back(i);
      
      // Update influence with current data point.
      filteredY[i] = FLAGS_z_score_influence * currentinput 
        + invInfluence * filteredY[i - 1];
    }

    // Adjust the filters.
    //const double m = mean(prev_mean, filteredY, i - FLAGS_z_score_lag, i);
    avgFilter[i] = prev_mean + (filteredY[i - FLAGS_z_score_lag-1] 
        + filteredY[i]) / FLAGS_z_score_lag;

    //if (avgFilter[i-1] - m > 0.01)
      //stdFilter[i] = stdDev(m, filteredY, i - FLAGS_z_score_lag, i);
    //else 
      stdFilter[i] = stdFilter[i-1];

    if (i%1500 == 0) 
      VLOG(1) << "current i = " << i << " mean: " << avgFilter[i] 
        << " mean diff: " << std::abs(avgFilter[i-1] - avgFilter[i])
        << " std: " << stdFilter[i] << " peak size: " << signals->size();
  }
}

double ZScoreEval::mean(const double prev_mean, const std::vector<double>& vec, 
    uint32_t from, uint32_t to) const {
  return prev_mean + (vec[from-1] + vec[to]) / FLAGS_z_score_lag;
  //double sum = std::accumulate(vec.cbegin()+from, vec.cend()+to, 0.0);
  //return sum / (to-from);
}

double ZScoreEval::stdDev(const double mean, 
    const std::vector<double>& vec, uint32_t from, uint32_t to) const {
  double accum = 0.0;
  std::for_each (vec.cbegin()+from, vec.cend()+to, [&](const double val) {
    const double tmp = val - mean;
    accum += tmp * tmp;
  });
  return sqrt(accum / (to-from-1u));
}
}
