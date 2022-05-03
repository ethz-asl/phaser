#include "phaser/backend/uncertainty/z-score-peak-extraction.h"

#include <Eigen/Dense>
#include <algorithm>
#include <cmath>
#include <functional>
#include <glog/logging.h>
#include <numeric>

#include "phaser/backend/uncertainty/signal-analysis.h"

DEFINE_double(
    z_score_lag_percentile, 0.05,
    "The window used for smoothing the function.");
DEFINE_int32(
    z_score_lag_max, 20, "The window used for smoothing the function.");
DEFINE_double(
    z_score_threshold, 4.33,
    "Defines the number of n-std requires to include a signal.");
DEFINE_double(
    z_score_influence, 0.05,
    "The influence of the current data point to the lag mean.");
DEFINE_double(
    z_score_filter_threshold, 0.405,
    "Removes all correlation input below this value.");

namespace phaser_core {

ZScorePeakExtraction::ZScorePeakExtraction()
    : manager_("z-score"),
      lag_percentile_(FLAGS_z_score_lag_percentile),
      score_threshold_(FLAGS_z_score_threshold),
      influence_(FLAGS_z_score_influence),
      lag_max_(FLAGS_z_score_lag_max) {
  CHECK_GT(lag_percentile_, 0);
  CHECK_LT(lag_percentile_, 1);
  CHECK_GT(score_threshold_, 0);
  CHECK_GE(influence_, 0);
  CHECK_LE(influence_, 1.0);
}

void ZScorePeakExtraction::extractPeaks(
    const std::vector<double>& corr, std::set<uint32_t>* peaks) {
  const double lag =
      std::min(lag_max_, static_cast<uint32_t>(corr.size() * lag_percentile_));
  VLOG(1) << "Calculating z-scores (" << corr.size() << " ) using a " << lag
          << " window.";
  std::vector<double> input(corr);
  calculateSmoothedZScore(&input, lag, score_threshold_, influence_, peaks);
}

void ZScorePeakExtraction::calculateSmoothedZScore(
    std::vector<double>* input, const double lag, const double threshold,
    const double influence, std::set<uint32_t>* signals) const {
  signals->clear();
  const uint32_t n_input = input->size();
  if (n_input <= lag + 2)
    return;
  // std::vector<double> avgFilter(n_input, 0.0);
  // std::vector<double> stdFilter(n_input, 0.0);

  Eigen::Map<Eigen::VectorXd> eInput(input->data(), n_input);
  // Calculate initial mean and covariance.
  // const double m =
  // std::accumulate(input->cbegin(), input->cbegin() + lag, 0.0) / lag;
  // avgFilter[lag] = m;
  // stdFilter[lag] = SignalAnalysis::stdDev(*input, m, 0, lag);
  double avg = eInput.block(0, 0, lag, 1).array().mean();
  // double dev = SignalAnalysis::stdDev(*input, m, 0, lag);
  const double lagm1 = lag - 1;
  double dev = std::sqrt(
      (eInput.block(0, 0, lag, 1).array() - avg).square().sum() / lagm1);

  // windowed iteration over the data points.
  const double invInfluence = 1.0 - influence;
  for (uint32_t i = lag + 1u; i < n_input; ++i) {
    const double currentinput = (*input)[i];
    if (currentinput <= FLAGS_z_score_filter_threshold)
      continue;
    // const double prev_mean = avgFilter[i - 1];
    if (std::abs(currentinput - avg) > threshold * dev) {
      if (currentinput >= avg)
        signals->insert(i);

      // Update influence with current data point.
      (*input)[i] = influence * currentinput + invInfluence * (*input)[i - 1];
    }

    // Adjust the filters.
    /*
    avgFilter[i] =
        std::accumulate(input->cbegin() + (i - lag), input->cbegin() + i, 0.0) /
        lag;
    stdFilter[i] = SignalAnalysis::stdDev(*input, avgFilter[i], i - lag, i);
    */

    // VLOG(1) << "calc mean ";
    /*
    avg =
        std::accumulate(input->cbegin() + (i - lag), input->cbegin() + i, 0.0) /
        lag;
  VLOG(1) << "avg3: " << avg;
  VLOG(1) << "avg4: "
    <<
    dev = SignalAnalysis::stdDev(*input, avg, i - lag, i);
    */
    const uint32_t start = i - lag;
    const Eigen::ArrayXd& input_lag = eInput.block(start, 0, lag, 1).array();
    avg = input_lag.mean();
    dev = std::sqrt((input_lag - avg).square().sum() / lagm1);
  }
}

double ZScorePeakExtraction::getLagPercentile() const {
  return lag_percentile_;
}

double& ZScorePeakExtraction::getLagPercentile() {
  return lag_percentile_;
}

double ZScorePeakExtraction::getScoreThreshold() const {
  return score_threshold_;
}

double& ZScorePeakExtraction::getScoreThreshold() {
  return score_threshold_;
}

double ZScorePeakExtraction::getInfluence() const {
  return influence_;
}

double& ZScorePeakExtraction::getInfluence() {
  return influence_;
}

uint32_t ZScorePeakExtraction::getMaxLag() const {
  return lag_max_;
}

uint32_t& ZScorePeakExtraction::getMaxLag() {
  return lag_max_;
}

}  // namespace phaser_core
