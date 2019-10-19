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
    std::vector<double> n_corr_ds2;
    std::vector<double> n_corr_ds3;
    std::vector<double> n_corr_ds4;
    std::copy_if(corr_ds.cbegin(), corr_ds.cend(), std::back_inserter(n_corr_ds), 
        std::bind(std::greater<double>(), std::placeholders::_1, 0.105));
    std::copy_if(corr_ds.cbegin(), corr_ds.cend(), std::back_inserter(n_corr_ds2), 
        std::bind(std::greater<double>(), std::placeholders::_1, 0.105));
    std::copy_if(corr_ds.cbegin(), corr_ds.cend(), std::back_inserter(n_corr_ds3), 
        std::bind(std::greater<double>(), std::placeholders::_1, 0.105));
    std::copy_if(corr_ds.cbegin(), corr_ds.cend(), std::back_inserter(n_corr_ds4), 
        std::bind(std::greater<double>(), std::placeholders::_1, 0.105));

    for (auto& ds : n_corr_ds) {
      manager_.emplaceValue("signal", ds);
    }

    

    VLOG(1) << "size after: " << n_corr_ds.size();
    calculateSmoothedZScore(n_corr_ds, 500, 6.25, 0.05, &signals);
    for (uint32_t peak : signals) 
      VLOG(1) << "Peaks1 " << peak << " with the value: " << n_corr_ds4[peak];
    VLOG(1) << " ==== Peaks1 ";

    calculateSmoothedZScore(n_corr_ds2, 400, 5.5, 0.05, &signals);
    for (uint32_t peak : signals) 
      VLOG(1) << "Peaks2 " << peak << " with the value: " << n_corr_ds4[peak];
    VLOG(1) << " ==== Peaks2 ";

    calculateSmoothedZScore(n_corr_ds3, 350, 5.5, 0.05, &signals);
    for (uint32_t peak : signals) 
      VLOG(1) << "Peaks3 " << peak << " with the value: " << n_corr_ds4[peak];
    VLOG(1) << " ==== Peaks3 ";

    auto it_max = std::max_element(n_corr_ds4.begin(), n_corr_ds4.end());

    VLOG(1) << "max at: " << std::distance(n_corr_ds4.begin(), it_max) 
      << " with value: " << *it_max;

    //calculateSmoothedZScore(n_corr_ds, FLAGS_z_score_lag, 
        //FLAGS_z_score_threshold, FLAGS_z_score_influence, &signals);
    VLOG(1) << "================= done with z-score";
    visualization::PlottyVisualizer::getInstance()
     .createPlotFor(manager_, "signal");
}

void ZScoreEval::calculateSmoothedZScore(std::vector<double>& input, 
    const double lag, const double threshold, const double influence,
    std::vector<uint32_t>* signals) const {
  const uint32_t n_input = input.size();
  if (n_input <= lag + 2) return;

  std::vector<double> avgFilter(n_input, 0.0);
  std::vector<double> stdFilter(n_input, 0.0);
  const double m = std::accumulate(input.cbegin(), 
      input.cend()+lag, 0.0) / lag;

  //return sum / (to-from);
  //const double m = mean(input, 0, FLAGS_z_score_lag);
  avgFilter[lag] = m;
  stdFilter[lag] = stdDev(m, input, 0, lag);

  //VLOG(1) << "initial mean: " << m << " from " << 0 << " to " << lag;
  const double invInfluence = 1.0 - influence;
  for (uint32_t i = lag + 1u; i < n_input; ++i)
  {
    const double currentinput = input[i];
    const double prev_mean = avgFilter[i - 1];
   // VLOG(1) << "prev_mean: " << prev_mean << " i = " << i;
    if (std::abs(currentinput - prev_mean) > threshold * stdFilter[i - 1]) {
      if (currentinput > prev_mean)
        signals->emplace_back(i);
      
      // Update influence with current data point.
      input[i] = influence * currentinput 
        + invInfluence * input[i - 1];
    } 
      //avgFilter[i] = prev_mean + (input[i]-input[i - lag-1]) / lag;
      avgFilter[i] = std::accumulate(input.cbegin()+(i - lag), 
        input.cbegin()+i, 0.0) / lag;

    // Adjust the filters.
    //const double m = mean(prev_mean, filteredY, i - FLAGS_z_score_lag, i);
    //avgFilter[i] = m2;

    //if (avgFilter[i-1] - m > 0.01)
      stdFilter[i] = stdDev(avgFilter[i], input, i - lag, i);
    //else 
      //stdFilter[i] = stdFilter[i-1];

      /*
    if (i%200 == 0) 
      VLOG(1) << "current i = " << i << " mean: " << avgFilter[i] 
        << " diff: " << std::abs(currentinput - prev_mean)
        << " std: " << stdFilter[i] << " peak size: " << signals->size();
        */
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
  //VLOG(1) << "std::mean: " << mean;
  //VLOG(1) << "vec " << vec.size() << " from: " << from << " to: " << to;
  for (uint32_t i = from; i < to; ++i) {
    const double val = vec[i];
    if (val <= 1.0) {
    const double tmp = val - mean;
    //VLOG(1) << "stddev val: " << val;
    //VLOG(1) << "stddev val - mean: " << val - mean;
    //VLOG(1) << "stddev tmp: " << tmp;
    //VLOG(1) << "stddev tmp 8 tmp: " << tmp * tmp;
    //VLOG(1) << "stddev accum: " << accum;
    accum += tmp * tmp;
    } else
      VLOG(1) << "=========================== " <<i;
  }
  //VLOG(1) << "stddev: " << accum;
  double res =  std::sqrt(accum / (to-from-1u));
  //VLOG(1) << "stddev =  " << res;
  return res;
}
}
