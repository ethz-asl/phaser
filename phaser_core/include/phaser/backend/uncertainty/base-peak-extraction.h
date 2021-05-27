#ifndef PHASER_BACKEND_UNCERTAINTY_BASE_PEAK_EXTRACTION_H_
#define PHASER_BACKEND_UNCERTAINTY_BASE_PEAK_EXTRACTION_H_

#include <memory>
#include <set>
#include <vector>

namespace phaser_core {

class BasePeakExtraction {
  virtual void extractPeaks(
      const std::vector<double>& corr, std::set<uint32_t>* peaks) = 0;
};

using BasePeakExtractionPtr = std::unique_ptr<BasePeakExtraction>;

}  // namespace phaser_core

#endif  // PHASER_BACKEND_UNCERTAINTY_BASE_PEAK_EXTRACTION_H_
