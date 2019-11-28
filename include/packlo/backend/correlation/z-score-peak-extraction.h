#ifndef INCLUDE_PACKLO_BACKEND_CORRELATION_Z_SCORE_PEAK_EXTRACTION_H_
#define INCLUDE_PACKLO_BACKEND_CORRELATION_Z_SCORE_PEAK_EXTRACTION_H_

#include "packlo/backend/correlation/base-peak-extraction.h"
#include "packlo/common/statistics-manager.h"

#include <cstdint>
#include <set>
#include <utility>
#include <vector>

namespace correlation {

class ZScorePeakExtraction : public BasePeakExtraction {
 public:
  ZScorePeakExtraction();

  void extractPeaks(
      const std::vector<double>& corr, std::set<uint32_t>* peaks) override;

 private:
  void calculateSmoothedZScore(
      const std::vector<double>& input, const double lag,
      const double threshold, const double influence,
      std::set<uint32_t>* signals) const;
  double stdDev(
      const double mean, const std::vector<double>& vec, uint32_t from,
      uint32_t to) const;
  std::pair<double, double> fitSmoothedNormalDist(
      const std::set<uint32_t>& signals,
      const std::vector<double>& input) const;

  common::StatisticsManager manager_;
};

}  // namespace correlation

#endif  // INCLUDE_PACKLO_BACKEND_CORRELATION_Z_SCORE_PEAK_EXTRACTION_H_
