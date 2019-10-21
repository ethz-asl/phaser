#include "packlo/backend/correlation/z-score-eval.h"
#include "packlo/visualization/plotty-visualizer.h"

#include <cmath>
#include <numeric>
#include <algorithm>
#include <functional>

#include <glog/logging.h>

// Good values: 500, 5.75, 0.05
//              70, 3.75, 0.05
//              210, 5.66, 0.05
DEFINE_double(z_score_lag, 210, 
    "The window used for smoothing the function.");
DEFINE_double(z_score_threshold, 5.66, 
    "Defines the number of n-std requires to include a signal.");
DEFINE_double(z_score_influence, 0.05, 
    "The influence of the current data point to the lag mean.");
DEFINE_double(z_score_filter_threshold, 0.105, 
    "Removes all correlation input below this value.");

namespace correlation {

ZScoreEval::ZScoreEval() : manager_("z-score") {
  CHECK_GT(FLAGS_z_score_lag, 0);
  CHECK_GT(FLAGS_z_score_threshold, 0);
  CHECK_GE(FLAGS_z_score_influence, 0);
  CHECK_LE(FLAGS_z_score_influence, 1.0);
}

void ZScoreEval::evaluateCorrelationFromTranslation(
      const std::vector<double>& corr) {
    std::set<uint32_t> signals;
    std::vector<double> n_corr;

    // Normalize correlation.
    double max = *std::max_element(corr.cbegin(), corr.cend());
    std::transform(corr.begin(), corr.end(), std::back_inserter(n_corr),
        [&] (const double val) {return val / max;});

    // Filter values close to zero for speedup.
    std::vector<double> n_corr_ds;
    std::copy_if(n_corr.cbegin(), n_corr.cend(), std::back_inserter(n_corr_ds), 
        std::bind(std::greater<double>(), std::placeholders::_1, 
          FLAGS_z_score_filter_threshold));

    // Find the peaks. 
    calculateSmoothedZScore(n_corr_ds, FLAGS_z_score_lag, 
        FLAGS_z_score_threshold, FLAGS_z_score_influence, &signals);

    // Evaluate correlation. 
    double mean, std;
    std::tie(mean, std) = fitSmoothedNormalDist(signals, n_corr_ds);

    // Visualization.
    for (auto& ds : n_corr_ds) 
      manager_.emplaceValue("downsampled_corr", ds);
    visualization::PlottyVisualizer::getInstance()
     .createPlotFor(manager_, "downsampled_corr");
    VLOG(1) << "finished z-score with var: " << std*std;
}

void ZScoreEval::calculateSmoothedZScore(std::vector<double>& input, 
    const double lag, const double threshold, const double influence,
    std::set<uint32_t>* signals) const {
  signals->clear();
  const uint32_t n_input = input.size();
  if (n_input <= lag + 2) return;
  std::vector<double> avgFilter(n_input, 0.0);
  std::vector<double> stdFilter(n_input, 0.0);

  // Calculate initial mean and covariance.
  const double m = std::accumulate(input.cbegin(), 
      input.cend() + lag, 0.0) / lag;
  avgFilter[lag] = m;
  stdFilter[lag] = stdDev(m, input, 0, lag);

  // windowed iteration over the data points. 
  const double invInfluence = 1.0 - influence;
  for (uint32_t i = lag + 1u; i < n_input; ++i)
  {
    const double currentinput = input[i];
    const double prev_mean = avgFilter[i - 1];
    if (std::abs(currentinput - prev_mean) > threshold * stdFilter[i - 1]) {
      if (currentinput > prev_mean)
        signals->insert(i);
      
      // Update influence with current data point.
      input[i] = influence * currentinput 
        + invInfluence * input[i - 1];
    } 

    // Adjust the filters.
    //avgFilter[i] = prev_mean + (input[i]-input[i - lag-1]) / lag;
    avgFilter[i] = std::accumulate(input.cbegin()+(i - lag), 
      input.cbegin()+i, 0.0) / lag;
    stdFilter[i] = stdDev(avgFilter[i], input, i - lag, i);
  }
}

double ZScoreEval::stdDev(const double mean, 
    const std::vector<double>& vec, uint32_t from, uint32_t to) const {
  double accum = 0.0;
  for (uint32_t i = from; i < to; ++i) {
    const double val = vec[i];
    if (val <= 1.0) {
      const double val_sub_mean = val - mean;
      accum += val_sub_mean * val_sub_mean;
    } 
  }
  VLOG(1) << "accum: " << accum;
  return std::sqrt(accum / (to-from-1u));
}

std::pair<double, double> ZScoreEval::fitSmoothedNormalDist(
    const std::set<uint32_t>& signals, 
    const std::vector<double>& input) const {
  std::vector<double> peak_values; 
  double sum2 = 0.0;  
  //double max = (double)*std::max_element(signals.begin(), signals.end());
  /*
  for (const uint32_t& signal : signals) {
    double norm_signal = ((double) signal) / max;
    peak_values.emplace_back(norm_signal);
    sum2 += norm_signal;
  }
  */
  std::transform(signals.cbegin(), signals.cend(), 
      std::back_inserter(peak_values), 
      [&max] (const uint32_t& signal) {return ((double) signal) / max; });
  VLOG(1) << "test accum: "
    << std::accumulate(peak_values.begin(), peak_values.end(), 0.0);

  for (double val : peak_values) {
    VLOG(1) << " peak_values: " << val;
  }
  const uint32_t n_values = peak_values.size();
  const double mean = 
    std::accumulate(peak_values.cbegin(), peak_values.cend(), 0.0) / n_values;

  VLOG(1) << "mean: " << mean;
  const double std = stdDev(mean, peak_values, 0, n_values);
  VLOG(1) << "std: " << std;
  return std::make_pair(mean, std);
}

} // namespace correlation
