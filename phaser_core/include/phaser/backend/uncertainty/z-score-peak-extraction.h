#ifndef PHASER_BACKEND_UNCERTAINTY_Z_SCORE_PEAK_EXTRACTION_H_
#define PHASER_BACKEND_UNCERTAINTY_Z_SCORE_PEAK_EXTRACTION_H_

#include <cstdint>
#include <set>
#include <utility>
#include <vector>

#include "phaser/backend/uncertainty/base-peak-extraction.h"
#include "phaser/common/statistics-manager.h"

namespace phaser_core {

class ZScorePeakExtraction : public BasePeakExtraction {
 public:
  ZScorePeakExtraction();

  void extractPeaks(
      const std::vector<double>& corr, std::set<uint32_t>* peaks) override;

  double getLagPercentile() const;
  double& getLagPercentile();

  double getScoreThreshold() const;
  double& getScoreThreshold();

  double getInfluence() const;
  double& getInfluence();

  uint32_t getMaxLag() const;
  uint32_t& getMaxLag();

 private:
  void calculateSmoothedZScore(
      std::vector<double>* input, const double lag, const double threshold,
      const double influence, std::set<uint32_t>* signals) const;

  common::StatisticsManager manager_;
  double lag_percentile_;
  double score_threshold_;
  double influence_;
  uint32_t lag_max_;
};

}  // namespace phaser_core

#endif  // PHASER_BACKEND_UNCERTAINTY_Z_SCORE_PEAK_EXTRACTION_H_
