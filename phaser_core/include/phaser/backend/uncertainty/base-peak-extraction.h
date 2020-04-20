#ifndef PACKLO_BACKEND_UNCERTAINTY_BASE_PEAK_EXTRACTION_H_
#define PACKLO_BACKEND_UNCERTAINTY_BASE_PEAK_EXTRACTION_H_

#include <memory>
#include <set>
#include <vector>

namespace uncertainty {

class BasePeakExtraction {
  virtual void extractPeaks(
      const std::vector<double>& corr, std::set<uint32_t>* peaks) = 0;
};

using BasePeakExtractionPtr = std::unique_ptr<BasePeakExtraction>;

}  // namespace uncertainty

#endif  // PACKLO_BACKEND_UNCERTAINTY_BASE_PEAK_EXTRACTION_H_
